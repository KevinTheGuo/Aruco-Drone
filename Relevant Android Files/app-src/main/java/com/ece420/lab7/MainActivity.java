package com.ece420.lab7;

import java.io.UnsupportedEncodingException;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.Map;

import android.app.PendingIntent;
import android.content.BroadcastReceiver;
import android.content.Context;
import android.content.Intent;
import android.content.IntentFilter;
import android.content.pm.ActivityInfo;
import android.content.pm.PackageManager;
import android.Manifest;
import android.hardware.Camera;
import android.os.Bundle;
import android.support.v4.app.ActivityCompat;
import android.support.v4.content.ContextCompat;
import android.support.v7.app.AppCompatActivity;
import android.util.Log;
import android.view.View;
import android.view.WindowManager;
import android.widget.Button;

import org.opencv.android.BaseLoaderCallback;
import org.opencv.android.CameraBridgeViewBase;
import org.opencv.android.LoaderCallbackInterface;
import org.opencv.android.OpenCVLoader;
import org.opencv.aruco.Aruco;
import org.opencv.aruco.Dictionary;
import org.opencv.calib3d.Calib3d;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import static org.opencv.aruco.Aruco.detectMarkers;
import static org.opencv.aruco.Aruco.drawAxis;
import static org.opencv.aruco.Aruco.drawDetectedMarkers;
import static org.opencv.aruco.Aruco.estimatePoseSingleMarkers;

import android.hardware.usb.UsbDevice;
import android.hardware.usb.UsbDeviceConnection;
import android.hardware.usb.UsbManager;
import com.felhr.usbserial.UsbSerialDevice;
import com.felhr.usbserial.UsbSerialInterface;

import android.os.Handler;


public class MainActivity extends AppCompatActivity implements CameraBridgeViewBase.CvCameraViewListener2 {
    // USB OTG stuff
    public final String ACTION_USB_PERMISSION = "com.ece420.lab7.USB_PERMISSION";
    UsbManager usbManager;
    UsbDevice device;
    UsbSerialDevice serialPort;
    UsbDeviceConnection connection;
    private Handler handler = new Handler();

    private static final String TAG = "MainActivity";
    private static float ALT_DIVISER = 1.0f;

    // UI Variables

    // Declare OpenCV based camera view base
    private CameraBridgeViewBase mOpenCvCameraView;
    private Dictionary dict;

    // Camera size
    private int myWidth;
    private int myHeight;

    // Mat to store RGBA and Grayscale camera preview frame
    private Mat mRgba;
    private Mat mGray;
    private Mat mRGB;
    private Mat ids;
    private java.util.List<Mat> corners;

    private double x_heading = 0.0f;
    private double y_heading = 0.0f;
    private double rotation = 0.0f;
    private double altitude_change = 0.0f;
    private int landing_mode = 0;

    private Button startButton, stopButton;

    float[] cameraCalibrationData = {244.8f, 0, 160.6f, 0, 245.2f,   117.8f, 0, 0, 1.0f};
    float[] distanceCoeffData = {0f,   0f,   0f, 0f,  0f};

    private Mat cameraCalib;
    private Mat distanceCoeff;

    // Private int which keeps track if we're sending serial or not
    private int serialOn = 0;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        getWindow().addFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON);
        setContentView(R.layout.activity_main);
        super.setRequestedOrientation (ActivityInfo.SCREEN_ORIENTATION_LANDSCAPE);

        // Request User Permission on Camera
        if (ContextCompat.checkSelfPermission(this, Manifest.permission.CAMERA) == PackageManager.PERMISSION_DENIED){
            ActivityCompat.requestPermissions(this, new String[] {Manifest.permission.CAMERA}, 1);}

        // OpenCV Loader and Avoid using OpenCV Manager
        if (!OpenCVLoader.initDebug()) {
            Log.e(this.getClass().getSimpleName(), "  OpenCVLoader.initDebug(), not working.");
        } else {
            Log.d(this.getClass().getSimpleName(), "  OpenCVLoader.initDebug(), working.");
        }

        // Setup OpenCV Camera View
        mOpenCvCameraView = (CameraBridgeViewBase) findViewById(R.id.opencv_camera_preview);
        // Use main camera with 0 or front camera with 1
        mOpenCvCameraView.setCameraIndex(0);
        // Force camera resolution, ignored since OpenCV automatically select best ones
        // mOpenCvCameraView.setMaxFrameSize(720, 1920);
        mOpenCvCameraView.setCvCameraViewListener(this);

        getWindow().setFlags(WindowManager.LayoutParams.FLAG_FULLSCREEN, WindowManager.LayoutParams.FLAG_FULLSCREEN);
        getWindow().addFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON);

        cameraCalib = new Mat(3, 3, CvType.CV_32F);
        cameraCalib.put(0, 0, cameraCalibrationData);

        distanceCoeff = new Mat(1, 5, CvType.CV_32F);
        distanceCoeff.put(0, 0 , distanceCoeffData);

        dict = Aruco.getPredefinedDictionary(Aruco.DICT_ARUCO_ORIGINAL);
        ids = new Mat();
        mRGB = new Mat();
        corners = new ArrayList<>();

        // USB OTG stuff
        startButton = (Button)findViewById((R.id.buttonStart));
        stopButton = (Button)findViewById((R.id.buttonStop));
        startButton.setEnabled(true);
        stopButton.setEnabled(false);
        usbManager = (UsbManager) getSystemService(this.USB_SERVICE);

        IntentFilter filter = new IntentFilter();
        filter.addAction(ACTION_USB_PERMISSION);
        filter.addAction(UsbManager.ACTION_USB_DEVICE_ATTACHED);
        filter.addAction(UsbManager.ACTION_USB_DEVICE_DETACHED);
        registerReceiver(broadcastReceiver, filter);

        Log.d("ARUCO_MATRIX", "Corners: " + corners.toString() + "Camera Calibration: " + cameraCalib.dump() + " Distance Coeffs: " + distanceCoeff.dump());
    }

    private Runnable runnable = new Runnable() {
        @Override
        public void run() {
      /* do what you need to do */
            String sendString;
            if (landing_mode == 1)
            {
                sendString = String.format("BLAZEIT: X%02d Y%02d R00 420", (int)x_heading, (int)y_heading);
            }
            else
            {
                sendString = String.format("BLAZEIT: X%02d Y%02d R00 Z00", (int)x_heading, (int)y_heading);
            }
            Log.d("USB_DEBUG", "Msg: " + sendString);


            if (serialOn == 1)
            {
//              String string = editText.getText().toString();
                sendString = "BLAZEIT: X00 Y00 R00 420";
                Log.d("USB_DEBUG", "FakeMsg: " + sendString);
                serialPort.write(sendString.getBytes());
                Imgproc.putText(mRGB, "Serial ON!" , new Point(300, 550), Core.FONT_HERSHEY_SIMPLEX, 1.5, new Scalar(255));
            }
      /* and here comes the "trick" */
            handler.postDelayed(this, 100);
        }
    };

    @Override
    protected void onResume() {
        super.onResume();
        if (!OpenCVLoader.initDebug()) {
            Log.d(TAG, "Internal OpenCV library not found. Using OpenCV Manager for initialization");
            OpenCVLoader.initAsync(OpenCVLoader.OPENCV_VERSION, this, mLoaderCallback);
        } else {
            Log.d(TAG, "OpenCV library found inside package. Using it!");
            mLoaderCallback.onManagerConnected(LoaderCallbackInterface.SUCCESS);
        }
    }

    @Override
    protected void onPause() {
        super.onPause();
        if (mOpenCvCameraView != null)
            mOpenCvCameraView.disableView();
    }

    @Override
    protected void onDestroy() {
        super.onDestroy();
        if (mOpenCvCameraView != null)
            mOpenCvCameraView.disableView();
    }

    private BaseLoaderCallback mLoaderCallback = new BaseLoaderCallback(this) {
        @Override
        public void onManagerConnected(int status) {
            switch (status) {
                case LoaderCallbackInterface.SUCCESS: {
                    Log.i(TAG, "OpenCV loaded successfully");
                    mOpenCvCameraView.enableView();
                }
                break;
                default: {
                    super.onManagerConnected(status);
                }
                break;
            }
        }
    };

    // OpenCV Camera Functionality Code
    @Override
    public void onCameraViewStarted(int width, int height) {
        mRgba = new Mat(height, width, CvType.CV_8UC4);
        mGray = new Mat(height, width, CvType.CV_8UC1);

        myWidth = width;
        myHeight = height;
    }

    @Override
    public void onCameraViewStopped() {
        mRgba.release();
        mGray.release();
    }

//    public rotatationMatrixToEulerAngles(Mat R)
//    {
//
//    }



    public static double[] MatrixToYawPitchRoll( Mat A )
    {
        double[] angle = new double[3];
        angle[1] = -Math.asin( A.get(2,0)[0] );  //Pitch
        if( A.get(2,0)[0] == 1 ) {
            angle[0] = 0.0;             //yaw = 0
            angle[2] = Math.atan2( -A.get(0,1)[0], -A.get(0,2)[0] );    //Roll
        }
        else if( A.get(2,0)[0] == -1 ){
            angle[0] = 0.0;             //yaw = 0
            angle[2] = Math.atan2( A.get(0,1)[0], A.get(0,2)[0]);    //Roll
        }
        //General solution
        else{
            angle[0] = Math.atan2(  A.get(1,0)[0], A.get(0,0)[0]);
            angle[2] = Math.atan2(  A.get(2,1)[0], A.get(2,2)[0]);
        }
        return angle;   //Euler angles in order yaw, pitch, roll
    }



    @Override
    public Mat onCameraFrame(CameraBridgeViewBase.CvCameraViewFrame inputFrame) {

        // Timer
        //long start = Core.getTickCount();
        // Grab camera frame in rgba and grayscale format
        mRgba = inputFrame.rgba();

        Imgproc.cvtColor(mRgba, mRGB, 1);
        detectMarkers(mRGB, dict, corners, ids);
        drawDetectedMarkers(mRGB, corners, ids, new Scalar(0, 255, 0));

        x_heading = 0.0f;
        y_heading = 0.0f;
        rotation = 0.0f;
        altitude_change = 0.0f;


        if(corners.size() > 0)
        {
            Mat rvec = new Mat();
            Mat tvec = new Mat();
            ArrayList<Mat> firstCorners = new ArrayList<>();
            firstCorners.add(corners.get(0));
            estimatePoseSingleMarkers(firstCorners, 1, cameraCalib, distanceCoeff, rvec, tvec);
            Log.d("ARUCO_MATRIX", "Corners: " + corners.toString() + "Rvec: " + rvec.dump() + "(Size: " + rvec.size().toString() + ") " + " Tvec: " + tvec.dump());
            Log.d("ARUCO_ID", "Ids: " + ids.dump());
            drawAxis(mRGB, cameraCalib, distanceCoeff, rvec, tvec, 1);

            Mat rotation_matrix = new Mat();
            Mat jacobian = new Mat();

            Calib3d.Rodrigues(rvec, rotation_matrix, jacobian);
            double [] eulerAngles = new double[3];
            //Log.d("ARUCO_MATRIX", "Rotation Matrix: " + rotation_matrix.dump() + "Second data point maybe?: " + rotation_matrix.get(0, 1)[0]);
            eulerAngles = MatrixToYawPitchRoll(rotation_matrix);

//            double a = tvec.get(0, 0)[0];
//            double a1 = tvec.get(0, 0)[1];
//            double b = tvec.get(0, 1)[0];
//            double c = tvec.get(0, 2)[0];
            String t_x = "Translate X:" + tvec.get(0, 0)[0];
            String t_y = "Translate Y:" + tvec.get(0, 0)[1];
            String t_z = "Translate Z:" + tvec.get(0, 0)[2];

            String eulerX = "Rotate X:" + eulerAngles[0];
            String eulerY = "Rotate Y:" + eulerAngles[1];
            String eulerZ = "Rotate Z:" + eulerAngles[2];

            Imgproc.putText(mRGB, t_x, new Point(10, 30), Core.FONT_HERSHEY_SIMPLEX, 0.8, new Scalar(255));
            Imgproc.putText(mRGB, t_y, new Point(10, 50), Core.FONT_HERSHEY_SIMPLEX, 0.8, new Scalar(255));
            Imgproc.putText(mRGB, t_z, new Point(10, 70), Core.FONT_HERSHEY_SIMPLEX, 0.8, new Scalar(255));

            Imgproc.putText(mRGB, eulerX, new Point(10, 110), Core.FONT_HERSHEY_SIMPLEX, 0.8, new Scalar(255));
            Imgproc.putText(mRGB, eulerY, new Point(10, 130), Core.FONT_HERSHEY_SIMPLEX, 0.8, new Scalar(255));
            Imgproc.putText(mRGB, eulerZ, new Point(10, 150), Core.FONT_HERSHEY_SIMPLEX, 0.8, new Scalar(255));

            int id = (int)ids.get(0, 0)[0];

            // Move command
            if((id >= 120) && (id <= 129))
            {
                x_heading = Math.sin(eulerAngles[2]);
                y_heading = Math.cos(eulerAngles[2]);
            }
            else if((id >= 220) && (id <= 229)) // rotation
            {
                rotation = -eulerAngles[2];
            }
            else if((id >= 320) && (id <= 329)) // move to marker
            {
                x_heading = (tvec.get(0, 0)[0]-tvec.get(0, 0)[2]*2) * 10;
                y_heading = (tvec.get(0, 0)[1]-tvec.get(0, 0)[2]) * 10;
            }
            else if((id >= 420) && (id <= 429)) // change altitude
            {
                float desired_altitude = id - 420;
                altitude_change = -(tvec.get(0, 0)[2]/ALT_DIVISER - desired_altitude);
                if (id == 422)
                {
                    x_heading = (tvec.get(0, 0)[0]-tvec.get(0, 0)[2]*2) * 10;
                    y_heading = (tvec.get(0, 0)[1]-tvec.get(0, 0)[2]) * 10;
                    landing_mode = 1;
                }
            }


            Imgproc.putText(mRGB, "X heading: " + x_heading, new Point(10, 190), Core.FONT_HERSHEY_SIMPLEX, 0.8, new Scalar(255));
            Imgproc.putText(mRGB, "Y heading: " + y_heading, new Point(10, 220), Core.FONT_HERSHEY_SIMPLEX, 0.8, new Scalar(255));
            Imgproc.putText(mRGB, "Rotation: " + rotation, new Point(10, 250), Core.FONT_HERSHEY_SIMPLEX, 0.8, new Scalar(255));
            Imgproc.putText(mRGB, "Altitude Change: " + altitude_change, new Point(10, 270), Core.FONT_HERSHEY_SIMPLEX, 0.8, new Scalar(255));


//            Imgproc.arrowedLine(mRGB, new Point(0.5 * myWidth,0.5 * myHeight), new Point((320+20*x_heading),(240-20*y_heading)), new Scalar(255));
//
//
//            Imgproc.arrowedLine(mRGB, new Point(0.5 * myWidth, 0.416 * myHeight), new Point((0.5 * myWidth+15*rotation), 0.47 * myHeight), new Scalar(255));
//            Imgproc.arrowedLine(mRGB, new Point(0.56 * myWidth, 0.5 * myHeight), new Point(0.05 * myWidth,240+15*rotation), new Scalar(255));
//            Imgproc.arrowedLine(mRGB, new Point(0.5 * myWidth, 0.58 * myHeight), new Point((0.5 * myWidth - 15 * rotation), 0.57 * myHeight), new Scalar(255));
//            Imgproc.arrowedLine(mRGB, new Point(0.4375 * myWidth, 0.5 * myHeight), new Point(0.43 * myWidth, 240-15 * rotation), new Scalar(255));
//            //cv2.arrowedLine(frame, (320,200), (int(320+15*rotation),int(200)), (255,50,50), 4)
//           // cv2.arrowedLine(frame, (360,240), (int(360),int(240+15*rotation)), (255,50,50), 4)
//           // cv2.arrowedLine(frame, (320,280), (int(320-15*rotation),int(280)), (255,50,50), 4)
//            //cv2.arrowedLine(frame, (280,240), (int(280),int(240-15*rotation)), (255,50,50), 4)
//
//            Imgproc.arrowedLine(mRGB, new Point(0.09375 * myWidth, 0.125 * myHeight), new Point((0.09375 * myWidth-10*altitude_change), (0.125 * myHeight-10*altitude_change)), new Scalar(255));
//            Imgproc.arrowedLine(mRGB, new Point(0.09375 * myWidth, 0.875 * myHeight), new Point((0.09375 * myWidth-10*altitude_change), (0.875 * myHeight+10*altitude_change)), new Scalar(255));
//            Imgproc.arrowedLine(mRGB, new Point(0.90625 * myWidth, 0.125 * myHeight), new Point((0.90625 * myWidth+10*altitude_change), (0.125 * myHeight-10*altitude_change)), new Scalar(255));
//            Imgproc.arrowedLine(mRGB, new Point(0.90625 * myWidth, 0.875 * myHeight), new Point((0.90625 * myWidth+10*altitude_change), (0.875 * myHeight+10*altitude_change)), new Scalar(255));

            // format message
            if (x_heading > 49)
                x_heading = 49;
            else if (x_heading < -49)
                x_heading = -49;

            if (y_heading > 49)
                y_heading = 49;
            else if (y_heading < -49)
                y_heading = -49;
            x_heading += 50;
            y_heading += 50;
//

            // send to USB
//            sendString = "BLAZEIT: X00 Y00 R00 Z00";

        }

        // Returned frame will be displayed on the screen
        return mRGB;
    }

    // USB OTG stuff here


    UsbSerialInterface.UsbReadCallback mCallback = new UsbSerialInterface.UsbReadCallback() {
        //Defining a Callback which triggers whenever data is read. We don't actually care about this one.
        @Override
        public void onReceivedData(byte[] arg0) {
            String data = null;
            try {
                data = new String(arg0, "UTF-8");
                data.concat("/n");
            } catch (UnsupportedEncodingException e) {
                e.printStackTrace();
            }
        }
    };

    private final BroadcastReceiver broadcastReceiver = new BroadcastReceiver() { //Broadcast Receiver to automatically start and stop the Serial connection.
        @Override
        public void onReceive(Context context, Intent intent) {
            if (intent.getAction().equals(ACTION_USB_PERMISSION)) {
                boolean granted = intent.getExtras().getBoolean(UsbManager.EXTRA_PERMISSION_GRANTED);
                if (granted) {
                    connection = usbManager.openDevice(device);
                    serialPort = UsbSerialDevice.createUsbSerialDevice(device, connection);
                    if (serialPort != null) {
                        if (serialPort.open()) { //Set Serial Connection Parameters.
                            serialPort.setBaudRate(38400);
                            serialPort.setDataBits(UsbSerialInterface.DATA_BITS_8);
                            serialPort.setStopBits(UsbSerialInterface.STOP_BITS_1);
                            serialPort.setParity(UsbSerialInterface.PARITY_NONE);
                            serialPort.setFlowControl(UsbSerialInterface.FLOW_CONTROL_OFF);
                            serialPort.read(mCallback);

                        } else {
                            Log.d("SERIAL", "PORT NOT OPEN");
                        }
                    } else {
                        Log.d("SERIAL", "PORT IS NULL");
                    }
                } else {
                    Log.d("SERIAL", "PERM NOT GRANTED");
                }
            } else if (intent.getAction().equals(UsbManager.ACTION_USB_DEVICE_ATTACHED)) {
                onClickStart(startButton);
            } else if (intent.getAction().equals(UsbManager.ACTION_USB_DEVICE_DETACHED)) {
                onClickStop(stopButton);

            }
        };
    };

    public void onClickStart(View view) {
        HashMap<String, UsbDevice> usbDevices = usbManager.getDeviceList();
        if (!usbDevices.isEmpty()) {
            boolean keep = true;
            for (Map.Entry<String, UsbDevice> entry : usbDevices.entrySet()) {
                device = entry.getValue();
                int deviceVID = device.getVendorId();
//                tvAppend(textView,"\nVendor ID of connected device: " + deviceVID + "\n");
                if ((deviceVID == 0x1A86) || (deviceVID == 0x2341))  //RobotDyn Vendor ID
                {
//                    tvAppend(textView,"\nConnecting to device!\n");
                    PendingIntent pi = PendingIntent.getBroadcast(this, 0, new Intent(ACTION_USB_PERMISSION), 0);
                    usbManager.requestPermission(device, pi);
                    keep = false;
                    serialOn = 1;
                    startButton.setEnabled(false);
                    stopButton.setEnabled(true);
                } else {
//                    tvAppend(textView,"\nRejecting device...\n");
                    connection = null;
                    device = null;
                }
                if (!keep)
                    break;
            }
        }
    }

    public void onClickStop(View view) {
        serialPort.close();
        startButton.setEnabled(true);
        stopButton.setEnabled(false);
        serialOn = 0;
    }
}