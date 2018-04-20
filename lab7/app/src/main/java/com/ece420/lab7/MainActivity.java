package com.ece420.lab7;

import java.util.ArrayList;

import android.content.pm.ActivityInfo;
import android.content.pm.PackageManager;
import android.Manifest;
import android.os.Bundle;
import android.support.annotation.NonNull;
import android.support.v4.app.ActivityCompat;
import android.support.v4.content.ContextCompat;
import android.support.v7.app.AppCompatActivity;
import android.util.Log;
import android.view.View;
import android.view.WindowManager;
import android.widget.Button;
import android.widget.SeekBar;
import android.widget.TextView;

import org.opencv.android.BaseLoaderCallback;
import org.opencv.android.CameraBridgeViewBase;
import org.opencv.android.LoaderCallbackInterface;
import org.opencv.android.OpenCVLoader;
import org.opencv.aruco.Aruco;
import org.opencv.aruco.Dictionary;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import static org.opencv.aruco.Aruco.detectMarkers;
import static org.opencv.aruco.Aruco.drawAxis;
import static org.opencv.aruco.Aruco.drawDetectedMarkers;
import static org.opencv.aruco.Aruco.estimatePoseSingleMarkers;

public class MainActivity extends AppCompatActivity implements CameraBridgeViewBase.CvCameraViewListener2 {

    private static final String TAG = "MainActivity";

    // UI Variables
    private Button controlButton;

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

    float[] cameraCalibrationData = {314.076860f, 0, 374.00695f, 0, 2154.70543f,   641.573192f, 0, 0, 1.0f};
    float[] distanceCoeffData = {0.396141271f,   12.2948727f,   .0837707176f, -.0457993412f,  -60.1430200f};

    private Mat cameraCalib;
    private Mat distanceCoeff;

    // KCF Tracker variables
//    private TrackerKCF myTracker;
//    private Rect2d myROI = new Rect2d(0,0,0,0);
//    private int myROIWidth = 70;
//    private int myROIHeight = 70;
//    private Scalar myROIColor = new Scalar(0,0,0);
//    private int tracking_flag = -1;

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
        // mOpenCvCameraView.setMaxFrameSize(1280, 720);
        mOpenCvCameraView.setCvCameraViewListener(this);

        cameraCalib = new Mat(3, 3, CvType.CV_32F);
        cameraCalib.put(0, 0, cameraCalibrationData);

        distanceCoeff = new Mat(1, 5, CvType.CV_32F);
        distanceCoeff.put(0, 0 , distanceCoeffData);
        dict = Aruco.getPredefinedDictionary(Aruco.DICT_ARUCO_ORIGINAL);
        ids = new Mat();
        mRGB = new Mat();
        corners = new ArrayList<>();

    }

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

    @Override
    public Mat onCameraFrame(CameraBridgeViewBase.CvCameraViewFrame inputFrame) {

        // Timer
        //long start = Core.getTickCount();
        // Grab camera frame in rgba and grayscale format
        mRgba = inputFrame.rgba();


        Imgproc.cvtColor(mRgba, mRGB, 1);
        // Grab camera frame in gray format
        //mGray = inputFrame.gray();
        //frame = cv2.aruco.drawDetectedMarkers(frame, corners, ids

        detectMarkers(mRGB, dict, corners, ids);
        drawDetectedMarkers(mRGB, corners, ids, new Scalar(0, 255, 0));

        if(corners.size() > 0)
        {
            //public static void estimatePoseSingleMarkers(List<Mat> corners, float markerLength, Mat cameraMatrix, Mat distCoeffs, Mat rvecs, Mat tvecs)
            Mat rvecs = new Mat();
            Mat tvecs = new Mat();
            ArrayList<Mat> firstCorners = new ArrayList<>();
            firstCorners.add(corners.get(0));
            estimatePoseSingleMarkers(firstCorners, 1, cameraCalib, distanceCoeff, rvecs, tvecs);
            Log.d("ARUCO_MATRIX", "Corners: " + corners.toString() + "Rvecs: " + rvecs.dump() + " Tvecs: " + tvecs.dump());
            drawAxis(mRGB, cameraCalib, distanceCoeff, rvecs, tvecs, 1);
        }

        // Action based on tracking flag
//        if(tracking_flag == -1){
//            // Update myROI to keep the window to the center
//            myROI.x = myWidth / 2 - myROIWidth / 2;
//            myROI.y = myHeight / 2 - myROIHeight / 2;
//            myROI.width = myROIWidth;
//            myROI.height = myROIHeight;
//        }
//        else if(tracking_flag == 0){
//            // Initialize KCF Tracker and Start Tracking
//            // 1. Create a KCF Tracker
//            // 2. Initialize KCF Tracker with grayscale image and ROI
//            // 3. Modify tracking flag to start tracking
//            // ******************** START YOUR CODE HERE ******************** //
//            boolean successful = myTracker.init(mGray, myROI);
//            if(successful)
//                tracking_flag = 1;
//            else
//                Log.d(TAG, "Failed to initialize the tracker");
//
//
//            // ******************** END YOUR CODE HERE ******************** //
//        }
//        else{
//            // Update tracking result is succeed
//            // If failed, print text "Tracking failure occurred!" at top left corner of the frame
//            // Calculate and display "FPS@fps_value" at top right corner of the frame
//            // ******************** START YOUR CODE HERE ******************** //
//            boolean successful = myTracker.update(mGray, myROI);
//            int currentFPS = (int)(Core.getTickFrequency() / (Core.getTickCount() - start));
//            String fps = "FPS = " + currentFPS;
//            Imgproc.putText(mRgba, fps, new Point(10, 30), Core.FONT_HERSHEY_COMPLEX, 1.0, new Scalar(255));
//
//            if(!successful)
//            {
//                Imgproc.putText(mRgba, "LOST TARGET", new Point(100, 30), Core.FONT_HERSHEY_COMPLEX, 1.0, new Scalar(255));
//            }
//
//            // ******************** END YOUR CODE HERE ******************** //
//        }

        // Draw a rectangle on to the current frame
//        Imgproc.rectangle(mRgba,
//                          new Point(myROI.x, myROI.y),
//                          new Point(myROI.x + myROI.width, myROI.y + myROI.height),
//                          myROIColor,
//                4);

        // Returned frame will be displayed on the screen
        return mRGB;
    }
}