/* This sketch deals with a receiever, receiving instructions from a base station, which it will try
 *  to match the orientation of. Pitch, roll, and heading are all taken into account. 
 *  VERSION 2.6 IS FOR APM COMPATIBILITY. Automatic AUX and YAW control
 *  Has transceiving capability- listens for messages from base, then acknowledges with telemetry info
 */
 
// includes for i2c and communication stuff
#include "Wire.h"

// includes for GPS
#include <TinyGPS++.h>
#include <SoftwareSerial.h> 

// includes for radio communication
#include <SPI.h>
#include "RF24.h"

// includes for sensors
#include "BMP085.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include <HMC5883L.h>

// include software PWM to control inputs to the ArduCopter (because we're out of PWM outputs). Apparently this kills Timer1 too.
#include <Servo.h>

// SETTINGS WE MAY NEED TO CHANGE
#define XGyroOffset      -157
#define YGyroOffset       84
#define ZGyroOffset       29
#define ZAccelOffset      411
float declinationAngle = (-3.0 + (11.0 / 60.0)) / (180 / M_PI);   // this is declination angle for UIUC! Change if you are somewhere else.

/*  PINOUT TO:
 *  
 *  ArduCopter:
 *  AUX1- pin 5 (yellow) - TIMER0, but we use software PWM, because if it goes crazy, it's not too bad
 *  YAW(rudder)- pin 6 (green) - TIMER0, but we use software PWM, because if it goes crazy, it's not too bad
 *  PITCH(elevator)- pin 9 (blue) - TIMER1
 *  ROLL(aileron)- pin 10 (purple) - TIMER1
 *  THROTTLE- pin 3 (white) - TIMER2
 *  
 *  
 *  Radio:
 *  CE- pin 7 (orange)
 *  CSN- pin 8 (yellow)
 *  SCK- pin 13 (green)
 *  MOSI- pin 11 (blue)
 *  MISO- pin 12 (purple)
 *  
 */

 // Define pin variables
#define AUX1_PIN      5
#define YAW_PIN       6
#define PITCH_PIN     9
#define ROLL_PIN      10
#define THROTTLE_PIN  3
// Create servo objects for our softwarePWM pins
Servo AUX1_PWM;
Servo YAW_PWM;
Servo PITCH_PWM;
Servo ROLL_PWM;

// initialize objects for each of our sensors
BMP085 barometer;
MPU6050 mpu;
HMC5883L compass;

// MPU control and status variables
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// MPU orientation and motion   variables
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// MPU interruption detection
volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}

// global variables for the mpu
float currYaw, currPitch, currRoll, prevYaw, prevPitch, prevRoll;   // holds orientation

// global variable for the magnetometer
float currHeading, prevHeading;          //  holds current and prev heading
int antiGlitch = 5;     // check this flag to see how long we can keep throwing out bad values
int desiredHeading;    // variables to control the quadcopter

// Heartbeat variable
unsigned long lastMessageMillis = 0;

// the Radio object
RF24 radio(7,8);

// Declare addresses for radio transmission. Address[0] is used for quad writing, base reading, and Address[1] used for quad reading, base writing
byte addresses[][6] = {"quadw","basew"};

// Define structs for input and output
struct controlPackage{
  int throttle = 0;    
  int roll = 0;
  int pitch = 0;
  int yaw = 0;
  int aux1 = 0;
  int info = 0;  
  int checksum = 0;
};
struct telemetryPackage{
//  float latitude;      // currently unused
//  float longitude;     // currently unused
  float alt = 0.0;
  int heading = 0;
  int info = 0;   
  int checksum = 0;
};
struct controlPackage controls;
struct telemetryPackage telemetry;

// Defined PWM middle-value:
int midVal = 93;

// Aruco, blaze-it related variables
int blaze_it = 0;  // Variable to hold status of controller; 1 if accepting serial commands (from phone), and 0 if taking commands from base.
int blaze_X = 0, blaze_Y = 0, blaze_R = 0, blaze_Z = 0;   // Variables to hold received commands from phone
int landing_mode = 0;       // if this is set, then aux1 will be set to enable landing mode

void setup() {
  Serial.begin(38400);
  Serial.println(F("Welcome to the the Quad Receiver!"));

  // join I2C bus 
  Wire.begin();
  Wire.setClock(4200);   // blazeit!
  
  // set our radio stuff
  radio.begin();
  radio.setDataRate(RF24_250KBPS);
  radio.setChannel(108);
  radio.setPALevel(RF24_PA_MAX); // we might need to do this or not
  radio.enableAckPayload();                     // Allow optional ack payloads
  radio.enableDynamicPayloads();                // Ack payloads are dynamic payloads
  radio.openWritingPipe(addresses[0]);
  radio.openReadingPipe(1,addresses[1]);
  radio.startListening();
  radio.writeAckPayload(1,&telemetry,sizeof(telemetry));

  // initialize sensors!
  mpu.dmpInitialize();    // initialize our mpu with the dmp!   DONT NEED MPU RIGHT NOW, PROBABLY NEVER WILL
  mpu.setXGyroOffset(XGyroOffset);     // set offsets from test
  mpu.setYGyroOffset(YGyroOffset);
  mpu.setZGyroOffset(ZGyroOffset);
  mpu.setZAccelOffset(ZAccelOffset);
  mpu.setDMPEnabled(true);    // enable our DMP!! this takes load off our arduino and gives nice results!
  packetSize = mpu.dmpGetFIFOPacketSize();    // get DMP packet size for later comparison

  compass.begin();    // initialize our compass!
  compass.setRange(HMC5883L_RANGE_1_3GA);  // Set measurement range
  compass.setMeasurementMode(HMC5883L_CONTINOUS);  // Set measurement mode
  compass.setDataRate(HMC5883L_DATARATE_30HZ);  // Set data rate
  compass.setSamples(HMC5883L_SAMPLES_8);  // Set number of samples averaged
  compass.setOffset(0,0);   // Set calibration offset. See HMC5883L_calibration.ino. Format is x,y

  barometer.bmp085Calibration();        // calibrate our pressure sensor

  // Initialize our PWM outputs 
  pinMode(THROTTLE_PIN, OUTPUT);   // sets the pin as output. We use our precious hardware PWM for throttle because this one is super important
  ROLL_PWM.attach(ROLL_PIN);
  PITCH_PWM.attach(PITCH_PIN);
  AUX1_PWM.attach(AUX1_PIN);      // Set these as software PWM
  YAW_PWM.attach(YAW_PIN);

  // Let's see how well we do with a modified Timer0. This fixes the frequency problem with pins 5 and 6, but modifies millis
//  TCCR0A = _BV(COM0A1) | _BV(COM0B1) | _BV(WGM00);  disabled this because now we use softwarePWM on those pins

  lastMessageMillis = millis();   // initialize our heartbeat variable to current time
}

void loop() 
{
  // grab barometer information
  float temperature = barometer.bmp085GetTemperature(); //MUST be called first?
  float pressure = barometer.bmp085GetPressure();
  float altitude = barometer.calcAltitude(pressure);  // so this doesn't work because these rely on delay(), which is messed up right now.

  // grab compass information
  Vector mag = compass.readNormalize();
  prevHeading = currHeading;
  currHeading = processHeading(mag);

  // grab MPU information
  fifoCount = mpu.getFIFOCount();     // get current FIFO count
  while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();        // wait for correct available data length
  mpu.getFIFOBytes(fifoBuffer, packetSize);       // read a packet from FIFO 
  mpu.resetFIFO();    // CLEAR THE BUFFER!!
  fifoCount -= packetSize;      // track FIFO count here in case there is > 1 packet available

  // now we can grab yaw, pitch and roll from the dmp buffer
  mpu.dmpGetQuaternion(&q, fifoBuffer);
  mpu.dmpGetGravity(&gravity, &q);
  mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
  mpu.dmpGetAccel(&aa, fifoBuffer);

  // Update our telemetry package
  telemetry.alt = altitude;
  telemetry.heading = currHeading;
//  telemetry.info = 0;
  telemetry.checksum = telemetry.alt + telemetry.heading + telemetry.info;
  radio.writeAckPayload(1,&telemetry,sizeof(telemetry));

  // Check for any new serial commands.
  // input string format is: "BLAZEIT: X00 Y00 R00 Z00"
  // if the quadcopter is in landing mode, it will instead send "BLAZEIT: X00 Y00 R00 420"
  // where the two digits after X, Y, R, and Z correspond to roll, pitch, yaw, and altitude change, accordingly.
  if (Serial.available() > 0)
  {
    String cmd = Serial.readString();
    telemetry.info = 1;
    Serial.println("Received message!");
    Serial.println(cmd);
    if (cmd.startsWith("BLAZEIT:"))
    {
      telemetry.info = 2;
      String x_cmd = cmd.substring(10,13);
      String y_cmd = cmd.substring(14,17);
      String r_cmd = cmd.substring(18,21);
      String z_cmd = cmd.substring(21,25);
      blaze_X = x_cmd.toInt() - 50;
      blaze_Y = y_cmd.toInt() - 50;
      blaze_R = r_cmd.toInt();
      telemetry.info = 3;
  
      if (z_cmd.startsWith("Z"))  // regular altitude change command
        blaze_Z = z_cmd.substring(1,3).toInt();
      else if (z_cmd.startsWith("420"))   // landing mode command
      {
          landing_mode = 1;
          blaze_Z = 0;
      }

      Serial.println(blaze_X);
      Serial.println(blaze_Y);
      Serial.println(blaze_R);
      Serial.println(blaze_Z);
      Serial.println(landing_mode);
    }
    
//    if (blaze_it == 0)
//    {
//      blaze_X = 0;
//      blaze_Y = 0;
//      blaze_R = 0;
//      blaze_Z = 0;
//      landing_mode = 0;
//      telemetry.info = -1;
//    }

    // empty serial buffer
    while(Serial.available() > 0) {
        char t = Serial.read();
    }
  }

  // Check for any new radio commands
  while(radio.available())
  {
    lastMessageMillis = millis();   // update our timestamp for last message received
    radio.read(&controls, sizeof(controls));

    // Verify that the message is valid by literally checking the sum (hahaha)
    if (controls.checksum != controls.throttle + controls.roll + controls.pitch + controls.yaw + controls.aux1 + controls.info)
    {
      Serial.println(F("ERR: CheXum Invalid! Ignoring all commands"));
      break;
    }

    // write our stuff in if we have a change!
    ROLL_PWM.write(controls.roll*0.25 + 0 + midVal);
    PITCH_PWM.write(controls.pitch*0.25 + 0 + midVal);
    YAW_PWM.write(controls.yaw*0.5 + 0 + midVal);

//    if (landing_mode)   // if landing mode, set Aux and throttle manually, otherwise use default
//    {
//      AUX1_PWM.write(130);
//      analogWrite(THROTTLE_PIN, 0);  
//    }
//    else
//    {
      AUX1_PWM.write(controls.aux1*0.75 + midVal*0.6); 
      analogWrite(THROTTLE_PIN, controls.throttle + 0 + midVal*1.5);  
//    }


    Serial.print(F("throttle: "));
    Serial.print(controls.throttle + midVal*1.5);    
    Serial.print(F("  roll: "));
    Serial.print(controls.roll*0.25 + midVal);
    Serial.print(F("  pitch: "));
    Serial.print(controls.pitch*0.25 + midVal);
    Serial.print(F("  heading: "));
    Serial.print(currHeading);
    Serial.print(F("  yaw: "));
    Serial.print(controls.yaw*0.75 + midVal);
    Serial.print(F("  aux1: "));
    Serial.print(controls.aux1*0.75 + midVal*0.6);   
    Serial.print(F("  alt: "));
    Serial.println(altitude,2);
  }

  // If we've lost connection, stop moving. 
  if (millis() - lastMessageMillis > 500)    // check if it's more than half a second since we last received a radio message
  {
    ROLL_PWM.write(midVal);   // zero out roll, pitch, and yaw, but keep all other signals the same
    PITCH_PWM.write(midVal);
    YAW_PWM.write(midVal);
    
//    Serial.println(F("ERR: Lost connection!"));
  }

  // Check for other additional info from the base station. 
  if (controls.info == 1)  // if we need to arm the quadcopter. This requires throttle to be low already.
  {
    YAW_PWM.write(45 + midVal);
    delay(2500);
    YAW_PWM.write(controls.yaw*0.75 + midVal);
  }
  else if (controls.info == -1)  // disarm the quadcopter
  {
    YAW_PWM.write(-45 + midVal);
    delay(5000);
    YAW_PWM.write(controls.yaw*0.75 + midVal);
  }
  else if (controls.info == 42) // set quadcopter to "phone-command" mode, and accept input from serial as well as the base.
  {
    blaze_it = 1;
    Serial.println(F("WE BLAZIN NOW"));
  }
  else if (controls.info == -42) // set quadcopter back to regular mode, accepting input from base station only.
  {
    blaze_it = blaze_X = blaze_Y = blaze_R = blaze_Z = landing_mode = 0;
    Serial.println(F("No longer blazin'"));
  }

}




