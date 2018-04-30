/* This sketch involves a base station, transmitting instructions a quadcopter
 *  Pitch and roll are determined by a joystick. Quadcopter keeps arm heading. A slider determines throttle.
 *  VERSION 2.6 IS FOR APM COMPATIBILITY. Automatic AUX and YAW control
 *  Is main writer, but receive acknowledgements containing telemetry information from quadcopter. 
 */
/* PINOUT:
 *  Analog output for throttle- A0
 */
// includes for i2c and communication stuff
#include "Wire.h"

// includes for radio communication
#include <SPI.h>
#include "RF24.h"

// Other Assorted Variables for Controlling the Quadcopter
int headingAtArm = 0;        // mildly temporary variable, set at arm, that ensures the quadcopter stays at same heading as it was when armed.
int isArmed = 0;              // lets quadcopter know if it's armed or not.
int blaze_it = 0;           // corresponds to variable on quadcopter which indicates if it's taking phone input or not

// our timer variable
unsigned long previousMillis = 0;

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

// Define the pins for our throttle and joystick
#define SLIDE_POT_PIN   A0
#define JOYSTICK_X_PIN  A1
#define JOYSTICK_Y_PIN  A2

void setup() {  
  Serial.begin(38400);
  Serial.println(F("Welcome to the the Quad Controller"));
  
  Serial.println(F("------------------------------------------------------------------------------------------------"));
  Serial.println(F("Use the joystick to control pitch and roll! Use the slider to control the throttle "));
  Serial.println(F("Quadcopter tries to hold the orientation at arm. Use keys 3/4 to control aux1."));
  Serial.println(F("To ARM the quadcopter, make sure throttle is at 0, and press 'a'"));
  Serial.println(F("To enter LAND mode, press 'l'"));
  Serial.println(F("To enter EMERGENCY SHUTDOWN mode, just press 'e'"));
  Serial.println(F("------------------------------------------------------------------------------------------------"));
  
  // set our radio stuff
  radio.begin();
  radio.setDataRate(RF24_250KBPS);
  radio.setChannel(108);
  radio.setPALevel(RF24_PA_MAX); // we might need to do this or not
  radio.enableAckPayload();                     // Allow optional ack payloads
  radio.enableDynamicPayloads();                // Ack payloads are dynamic payloads
  radio.openWritingPipe(addresses[1]);
  radio.openReadingPipe(1,addresses[0]);
  radio.stopListening();

  // and initialize the slide potentiometer and joystick inputs!
  pinMode(SLIDE_POT_PIN, INPUT);
  pinMode(JOYSTICK_X_PIN, INPUT);
  pinMode(JOYSTICK_Y_PIN, INPUT);
}

void loop() 
{
  // check for serial input
  if (Serial.available() > 0)
  {
    int serialIn = Serial.read();
    if ((serialIn == '3') && (controls.aux1 > 0))           // 51 THIS IS "3" 
    {
      controls.aux1 = controls.aux1 - 10;
      Serial.print(F("AUX1 down to "));      
      Serial.println(controls.aux1);
    }
    else if ((serialIn == '4') && (controls.aux1 < 100))           // 52 THIS IS "4"
    {
      controls.aux1 = controls.aux1 + 10;
      Serial.print(F("AUX1 up to "));         
      Serial.println(controls.aux1);
    }
    else if ((serialIn == '5') && (controls.yaw > -80))        // YAW LEFT
    {
      controls.yaw = controls.yaw - 10;
      Serial.print(F("YAW down to "));      
      Serial.println(controls.yaw);
    }
    else if ((serialIn == '6') && (controls.yaw < 80))         // YaW RIGHT
    {
      controls.yaw = controls.yaw + 10;
      Serial.print(F("YAW up to "));         
      Serial.println(controls.yaw);  
    }
    else if ((serialIn == 'a') && (controls.throttle == 0))      // Save current heading, and arm the quadcopter  
    {
      controls.info = 1;
      controls.checksum = controls.throttle + controls.roll + controls.pitch + controls.yaw + controls.aux1 + controls.info;
      radio.write(&controls, sizeof(controls));
      controls.info = 0;
      headingAtArm = telemetry.heading;
      previousMillis = millis();
      isArmed = 1;
      Serial.println(F("ARMING QUADCOPTER!!"));
    }
    else if ((serialIn == 'd') && (controls.throttle < 20))      // disarm the quadcopter
    {
      controls.info = -1;
      controls.checksum = controls.throttle + controls.roll + controls.pitch + controls.yaw + controls.aux1 + controls.info;
      radio.write(&controls, sizeof(controls));
      controls.info = 0;
      previousMillis = millis();
      isArmed = 0;
      controls.yaw = 0;
      Serial.println(F("DISARMING QUADCOPTER..."));
    }
    else if (serialIn == 'b')
    {
      if (blaze_it == 0)
      {
        controls.info = 42;
        Serial.println(F("420 BLAZEIT!"));
        blaze_it = 1;
      }
      else
      {
        controls.info = -42;
        Serial.println(F("No longer blazing it.."));
        blaze_it = 0;
      }
      controls.checksum = controls.throttle + controls.roll + controls.pitch + controls.yaw + controls.aux1 + controls.info;
      radio.write(&controls, sizeof(controls));
      controls.info = 0;
      previousMillis = millis();
    }
    else if (serialIn == 'l')
    {
      controls.aux1 = 90;
      Serial.println(F("STARTING LANDING SEQUENCE"));
    }
    else if (serialIn > 42) // ignore newlines and stuff
      Serial.print(F("INVALID COMMAND- "));
  }

  // now we can translate our orientation to radio signals! do some quick maths
  int throttleIn = analogRead(SLIDE_POT_PIN);  // read from analog 0
  if (throttleIn > 900)             // At high levels, it will taper out, so alleviate that a bit
    controls.throttle = throttleIn/8 - 37;      
  else
    controls.throttle = throttleIn/12;
//  controls.roll = analogRead(JOYSTICK_X_PIN)/3 - 170;
//  controls.pitch = analogRead(JOYSTICK_Y_PIN)/3 - 170;
  controls.roll = analogRead(JOYSTICK_X_PIN)/10 - 52;
  controls.pitch = analogRead(JOYSTICK_Y_PIN)/10 - 52;
  
  if((millis() - previousMillis) > 100)   // wait .1 seconds between displaying/transmitting
  {
    Serial.print(F("Mode: "));
    if(controls.aux1 <= 10)
      Serial.print(F("STABILIZE"));
    else if(controls.aux1 < 90)
      Serial.print(F("ALT HOLD"));
    else
      Serial.print(F("LAND"));
      
    Serial.print(F("  Throttle: "));
    Serial.print(controls.throttle);    
    Serial.print(F("  Roll: "));
    Serial.print(controls.roll);
    Serial.print(F("  Pitch: "));
    Serial.print(controls.pitch);
    Serial.print(F("  Quad Heading: "));
    Serial.print(telemetry.heading);
    Serial.print(F("  Arm Heading: "));
    Serial.print(headingAtArm);
    Serial.print(F("  Yaw: "));
    Serial.print(controls.yaw);
    Serial.print(F("  Quad Alt: "));
    Serial.print(telemetry.alt);
    Serial.print(F("  BLAZE IT: "));
    Serial.print(blaze_it);
    Serial.print(F("  Telemetry: "));
    Serial.print(telemetry.info);

    if ((controls.yaw >= 60) && (controls.throttle == 0))
      Serial.print(F(" -  ARMING!!!"));
    Serial.println("");
    
    controls.checksum = controls.throttle + controls.roll + controls.pitch + controls.yaw + controls.aux1 + controls.info;
    radio.write(&controls, sizeof(controls));
    previousMillis = millis();

    while (radio.available())    // If an ack with payload was received
    {                      
       radio.read(&telemetry, sizeof(telemetry));                  // Read it, and display the response time
       if (telemetry.checksum != int(telemetry.alt + telemetry.heading + telemetry.info))
         Serial.println(F("  Warning: Telemetry Checksum failed"));
       else if (isArmed)
       {
         int headingDiff = getHeadingDiff(headingAtArm, telemetry.heading);  // we now have a new desired heading! For now, set the comparison header to a constant.
//         controls.yaw = headingDiff/2;
       }
    }
  }
}


// Get a signed difference between two headings
int getHeadingDiff(float initial, float dest)
{
  if (initial > 360 || initial < 0 || dest > 360 || dest < 0)
  {
    Serial.print(F("Heading Diff Error!"));
    return 0;
  }

  int signed_diff = dest - initial;
  int abs_diff = abs(dest - initial);

  if (abs_diff <= 180)
    return abs_diff == 180 ? abs_diff : signed_diff;
  else if (dest-initial > 0)
    return abs_diff - 360;
  else
      return 360 - abs_diff;
}
