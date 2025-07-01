/*
  Pi Pico 2 W WiFi Station Demo
  pico2w_wifi_test.ino
  Use WiFi library to connect Pico 2 W to WiFi in station mode
*/

// include the WiFi Library
#include <WiFi.h>
#include <Wire.h>
#include "AS5600.h"
#include <BasicLinearAlgebra.h>
#include "math.h"
using namespace BLA;

// used for I2C communication with the AS5600 magnetic encoders
AS5600 as5600_0(&Wire);
AS5600 as5600_1(&Wire1);

// replace with your network credentials for connecting the Pico 2 W to wifi
const char* ssid = "REPLACE_WITH_SSID";
const char* password = "REPLACE_WITH_PASSWORD";

# define RIGHT_MOTOR_FORWARD GP0
# define RIGHT_MOTOR_BACKWARD GP1
# define LEFT_MOTOR_FORWARD GP2
# define LEFT_MOTOR_BACKWARD GP3

# define TOP_FLEX_SENSOR GP4
# define BOTTOM_FLEX_SENSOR GP5

# define TOP_ENCODER GP6
# define BOTTOM_ENCODER GP7


// FLEX SENSOR PARAMETERS
float rawFlexReadingBottom; // Unchanged output for bottom flex sensor
float rawFlexReadingTop; // Unchanged output for top flex sensor

float calibratedFlexReadingBottom; // Manipulated flex sensor data for bottom
float calibratedFlexReadingTop; // Manipulated flex sensor data for top

// technically not strain, but a calculated vector magnitude for each tether based on the flex sensor readings
float strainBottom;
float strainTop;

float STRAIGHT = 0; // The angle, in degrees, that is used to calibrate the flex sensor
                   // for when it is not bent
float BEND = 90; // The angle, in degrees, that is used to calibrate the flex sensor
                 // for when it is bent

const float STRAIGHT_BOTTOM = 420; // used for calibration and manipulation of raw data
const float STRAIGHT_TOP = 410; // used for calibration and manipulation of raw data
const float BEND_BOTTOM = 300; // used for calibration and manipulation of raw data
const float BEND_TOP = 300; // used for calibration and manipulation of raw data


// ENCODER ANGLE PARAMETERS
float rawAngleReadingBottom; // raw angle reading for bottom encoder
float rawAngleReadingTop; // raw angle reading for top encoder

float thetaBottom; // calibrated encoder angle for bottom tether
float thetaTop; // calibrated encoder angle for top tether

float deltaMeasured; // The difference between calibratedAngleDataBottom and calibratedAngleDataTop
float deltaError; // The error value between the desired and measured deltas

// values used for calibration of the encoders at each specified angles
// DIFFERENT FOR EACH ROBOT, FIX LATER
float encBottom_0 = 360; // value used for calibration for the bottom encoder and desired angle of 0 degrees
float encBottom_90 = 262;
float encBottom_180 = 176;
float encBottom_270 = 86;
float encBottom_360 = 0;

float encTop_0 = 210;
float encTop_90 = 122;
float encTop_180_first = 30;
float encTop_180_second = 390;
float encTop_270 = 300;
float encTop_360 = 210;


// HEADING VECTORS
float headingMagnitude; // magnitude of the robot's current heading vector
float headingDirection; // direction of the robot's current heading vector


// RESULTANT VECTORS
float resultantMagnitude; // magnitude of the next-step resultant vector
float resultantDirection; // direction of the next-step resultant vector


// PID OVER HEADING PARAMETERS
float dt; 
float errorE; 
float derivative; 
float PID_control_O; 
float previousError; 


// Goal Angle and Upper Stop Angle
float deltaDesired = 90; // desired tether angle difference between thetaBottom and thetaTop
float lowerAngle = 45;
float upperAngle = 80;


// Flex Sensor Range Angles 
float flexSensorBottomMin = 60; 
float flexSensorBottomMax = 70;
float flexSensorTopMin = 60; 
float flexSensorTopMax = 70;


// Array Initialization, for storing the history of data
const int maxnew = 5000; 
float timeR[maxnew];
float calibratedFlexDataBottom_Array[maxnew]; 
float calibratedFlexDataTop_Array[maxnew];     
float calibratedAngleDataBottom_Array[maxnew]; 
float calibratedAngleDataTop_Array[maxnew];     
float deltaMeasured_Array[maxnew]; 
float deltaD_Array[maxnew]; // I DON'T KNOW WHAT DELTAD IS, MUST FIND OUT
float resultantMagnitude_Array[maxnew]; 
float resultantDirection_Array[maxnew]; 
float directionDirection_Array[maxnew]; 


void setup() {

  Serial.begin(115200); // start the Serial Monitor
  WiFi.mode(WIFI_STA); // operate in WiFi Station mode
  WiFi.begin(ssid, password); // start WiFi with supplied parameters

  Wire.begin();
  Wire1.begin();

  pinMode(RIGHT_MOTOR_FORWARD, OUTPUT);
  pinMode(RIGHT_MOTOR_BACKWARD, OUTPUT);
  pinMode(LEFT_MOTOR_FORWARD, OUTPUT);
  pinMode(RIGHT_MOTOR_BACKWARD, OUTPUT);

  pinMode(TOP_FLEX_SENSOR, INPUT);
  pinMode(BOTTOM_FLEX_SENSOR, INPUT);

  as5600_0.begin();  //  set direction pin
  as5600_0.setDirection(AS5600_CLOCK_WISE);
  Serial.println(as5600_0.getAddress(),HEX);
  Serial.print("Device 0: ");
  Serial.println(as5600_0.isConnected() ? "connected" : "not connected");
  delay(1000);

  as5600_1.begin();  //  set direction pin
  as5600_1.setDirection(AS5600_CLOCK_WISE);
  Serial.println(as5600_1.getAddress(),HEX);
  Serial.print("Device 1: ");
  Serial.println(as5600_1.isConnected() ? "connected" : "not connected");
  delay(1000);

//   // print periods on monitor while establishing connection
//   while (WiFi.status() != WL_CONNECTED) {
//     delay(500);
//     Serial.print(".");
//     delay(500);
//   }

//   // connection established
//   Serial.println("");
//   Serial.print("Pico 2 W is connected to WiFi network ");
//   Serial.println(WiFi.SSID());

//   // print IP Address
//   Serial.print("Assigned IP Address: ");
//   Serial.println(WiFi.localIP());

}


void SPIN(float theta0, float phiDes, float deltaD) 
{
  Serial.print("theta0: "); Serial.println(theta0 * 180/M_PI);
  Serial.print("phiDes: "); Serial.println(phiDes * 180/M_PI);
  float Kp = 1;//16; float Kd = 1; 
  float endTime = millis(); float previousTime = millis(); 
  
  while (theta0*180/M_PI > (phiDes*180/M_PI)+5 || theta0*180/M_PI < (phiDes*180/M_PI)-5)
  {
    Serial.println("START:");
    // Angle Formation //////////////////////////////////////////////////////////////////
    ang_T0 = as5600_0.readAngle() * AS5600_RAW_TO_RADIANS * 180/M_PI;
    ang_T1 = as5600_1.readAngle() * AS5600_RAW_TO_RADIANS * 180/M_PI;

  //// ROBOT 4 
    // Calibrating the magnetic encoder for tether angles
    if(ang_T0 > T0_90 && ang_T0 <= T0_0) {theta0 = map(ang_T0, T0_0, T0_90, 0, 90);}
    else if (ang_T0 > T0_180 && ang_T0 <= T0_90) {theta0 = map(ang_T0, T0_90, T0_180, 90, 180);}
    else if (ang_T0 > T0_270 && ang_T0 <= T0_180) {theta0 = map(ang_T0, T0_180, T0_270, 180, 270);}
    else if (ang_T0 > T0_360 && ang_T0 <= T0_270) {theta0 = map(ang_T0, T0_270, T0_360, 270, 360);}

    //Serial.println(ang_T1);
    if (ang_T1 < T1_180first && ang_T1 >= 0) {ang_T1 = ang_T1 + 360;}
    if (ang_T1 > T1_90 && ang_T1 <= T1_0) {theta1 = map(ang_T1, T1_0, T1_90, 0, 90) + theta0;}
    else if (ang_T1 > T1_180first && ang_T1 <= T1_90) {theta1 = map(ang_T1, T1_90, T1_180first, 90, 180) + theta0;}
    else if (ang_T1 > T1_270 && ang_T1 <= T1_180second) {theta1 = map(ang_T1, T1_180second, T1_270, 180, 270) + theta0;}
    else if (ang_T1 > T1_360 && ang_T1 <= T1_270) {theta1 = map(ang_T1, T1_270, T1_360, 270, 360) + theta0;}
    if (theta1 > 360) {theta1 = theta1 - 360;}


    theta0 = theta0 * M_PI/180;
    theta1 = theta1 * M_PI/180;

    if (theta0 < theta1) {deltaM = theta1 - theta0;}
    else if (theta0 > theta1 && theta0 >= (270 * M_PI/180) && theta0 <= (360 * M_PI/180) && theta1 >= 0 && theta1 <= (270 * M_PI/180))
    {deltaM = (360*M_PI/180) - theta0 + theta1;}
    else {deltaM = (360*M_PI/180) - abs(theta1 - theta0);}

    // PID Control
    dt = (millis() - previousTime) / 1000;

    errorE = -1*(phiDes - theta0);
    if (errorE > M_PI) {errorE = -1*(2*M_PI - errorE);}
    if (errorE < -M_PI) {errorE = 1*(2*M_PI + errorE);}

    derivative = (errorE - previousError) / dt;
    if (derivative > 10) {derivative = 10;}
    else if (derivative < -10) {derivative = -10;}
    
    PID_control_O = Kp*errorE;// + Kd*derivative;  
    if (PID_control_O > 50) {PID_control_O = 50;}
    else if (PID_control_O < -50) {PID_control_O = -50;}
    previousError = errorE;
    previousTime = millis();
    
    // Motor Control
    int lowerR = 145;
    int lowerL = 145;
    RIGHT_MOTOR(lowerR);                   
    LEFT_MOTOR(lowerL); 

    int flexADC_0 = analogRead(A2);
    float ang_cant0 = map(flexADC_0, STRAIGHT_0, BEND_0, 40, 90);
    int flexADC_1 = analogRead(A3);
    float ang_cant1 = map(flexADC_1, STRAIGHT_1, BEND_1, 40, 90);

    Serial.print("PID_control_O: "); Serial.print(PID_control_O); Serial.println(); 
    Serial.print("ang_cant0: "); Serial.print(ang_cant0); Serial.print("  "); 
    Serial.print("ang_cant1: "); Serial.print(ang_cant1); Serial.println(); Serial.println();
  /*
    // Stop Robot if flex sensor angle is less than 60 or greater than 85 degrees
    if (ang_cant0 < 50 || ang_cant1 < 60 || ang_cant0 > 85 || ang_cant1 > 85)  
    {
      Serial.println("STOPPED ROBOT: ");
      analogWrite(A16, 0);
      analogWrite(A15, 0);   
      analogWrite(13, 0);
      analogWrite(A14, 0);   
      delay(1000);
    };
*/
    delay(250); 
  }  

  Serial.println("DONE:");     
  analogWrite(A16, 0);
  analogWrite(A15, 0);   
  analogWrite(13, 0);
  analogWrite(A14, 0);   
  delay(1000);
}

void DRIVE()
{ 
  analogWrite(A16, Right);
  analogWrite(A15, 0);   
  analogWrite(13, Left);
  analogWrite(A14, 0);  
  delay(100); 

  analogWrite(A16, 0);
  analogWrite(A15, 0);   
  analogWrite(13, 0);
  analogWrite(A14, 0);  
  delay(1000); 
}


//Function to momag_VE Right Motor 
void RIGHT_MOTOR(int lower) {
  // Pin A16 - Right, Forward
  if (PID_control_O > 0) {
    if (PID_control_O >= lower && PID_control_O <= 255) {
      analogWrite(A16, PID_control_O);
      analogWrite(A15, 0);
    }
    else if (PID_control_O >= 0 && PID_control_O < lower) {
      analogWrite(A16, lower);
      analogWrite(A15, 0);
    }
    else if (PID_control_O > 255) {
      analogWrite(A16, 255);
      analogWrite(A15, 0);
    }
  }
  // Pin A15 - Right, Backward
  if (PID_control_O < 0) {
    if (abs(PID_control_O) >= lower && abs(PID_control_O) <= 255) {
      analogWrite(A16, 0);
      analogWrite(A15, PID_control_O);
    }
    else if (abs(PID_control_O) >= 0 && abs(PID_control_O) < lower) {
      analogWrite(A16, 0);
      analogWrite(A15, lower);
    }
    else if (abs(PID_control_O) > 255) {
      analogWrite(A16, 0);
      analogWrite(A15, 255);
    }
  }
}


//Function to momag_VE Left Motor
void LEFT_MOTOR(int lower) {
  // Pin 13 - Left, Forward
  if (PID_control_O < 0) {
    if (abs(PID_control_O) >= lower && abs(PID_control_O) <= 255) {
      analogWrite(13, abs(PID_control_O));
      analogWrite(A14, 0);
    }
    else if (abs(PID_control_O) >= 0 && abs(PID_control_O) < lower) {
      analogWrite(13, lower);
      analogWrite(A14, 0);
    }
    else if (abs(PID_control_O) > 255) {
      analogWrite(13, 255);
      analogWrite(A14, 0);
    }
  }
  // Pin A14 - Left, Backward
  if (PID_control_O > 0) {
    if (abs(PID_control_O) >= lower && abs(PID_control_O) <= 255) {
      analogWrite(13, 0);
      analogWrite(A14, PID_control_O);
    }
    else if (abs(PID_control_O) >= 0 && abs(PID_control_O) < lower) {
      analogWrite(13, 0);
      analogWrite(A14, lower);
    }
    else if (abs(PID_control_O) > 255) {
      analogWrite(13, 0);
      analogWrite(A14, 255);
    }
  }
}


void loop() {

  delay(2000);

  // print IP Address
  Serial.print("Assigned IP Address: ");
  Serial.println(WiFi.localIP());

}
