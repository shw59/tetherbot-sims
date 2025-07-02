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
AS5600 bottomEncoder(&Wire);
AS5600 topEncoder(&Wire1);

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
// TODO: make these values robot-specific, make choosable based on name of robot
float encBottom0 = 360; // value used for calibration for the bottom encoder and desired angle of 0 degrees
float encBottom90 = 262;
float encBottom180 = 176;
float encBottom270 = 86;
float encBottom360 = 0;

float encTop0 = 210;
float encTop90 = 122;
float encTop180_first = 30;
float encTop180_second = 390;
float encTop270 = 300;
float encTop360 = 210;


// HEADING VECTORS
float desHeadingMagnitude; // magnitude of robot's desired heading vector
float desHeadingDirection; // direction of the robot's desired heading vector


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
float deltaMeasuredeasured_Array[maxnew]; 
float deltaD_Array[maxnew]; // I DON'T KNOW WHAT DELTAD IS, MUST FIND OUT
float resultantMagnitude_Array[maxnew]; 
float resultantDirection_Array[maxnew]; 
float directionDirection_Array[maxnew]; 


void setup() {
  Serial.begin(115200);

  Wire.begin();
  Wire1.begin();

  pinMode(RIGHT_MOTOR_FORWARD, OUTPUT);
  pinMode(RIGHT_MOTOR_BACKWARD, OUTPUT);
  pinMode(LEFT_MOTOR_FORWARD, OUTPUT);
  pinMode(RIGHT_MOTOR_BACKWARD, OUTPUT);

  pinMode(TOP_FLEX_SENSOR, INPUT);
  pinMode(BOTTOM_FLEX_SENSOR, INPUT);

  bottomEncoder.begin();  // set direction pin
  bottomEncoder.setDirection(AS5600_CLOCK_WISE);
  Serial.println(bottomEncoder.getAddress(),HEX);
  Serial.print("Device 0: ");
  Serial.println(bottomEncoder.isConnected() ? "connected" : "not connected");
  delay(1000);

  topEncoder.begin();  // set direction pin
  topEncoder.setDirection(AS5600_CLOCK_WISE);
  Serial.println(topEncoder.getAddress(),HEX);
  Serial.print("Device 1: ");
  Serial.println(topEncoder.isConnected() ? "connected" : "not connected");
  delay(1000);
}


void loop() {




  delay(2000);

  trueAngleDataBottom = bottomEncoder.readAngle() * AS5600_RAW_TO_RADIANS * 180/M_PI;
  trueAngleDataTop = topEncoder.readAngle() * AS5600_RAW_TO_RADIANS * 180/M_PI;
  Serial.print("The bottom's measured angle is: "); Serial.print(trueAngleDataBottom); Serial.print("  "); Serial.print("The tops's measured angle is: "); Serial.print(trueAngleDataTop); Serial.println();


  // Calibrates the angle of the bottom encoder so that it makes sense on 
  // a scale of 0-360 counter-clockwise off of the +x-axis
  // The calibration values differs from agent to agent on
  if((rawAngleReadingBottom > encBottom90) && (rawAngleReadingBottom <= encBottom0)) {
    calibratedAngleDataBottom = map(rawAngleReadingBottom, encBottom0, encBottom90, 0, 90);
  }
  else if ((rawAngleReadingBottom > encBottom180) && (rawAngleReadingBottom <= encBottom90)) {
    calibratedAngleDataBottom = map(rawAngleReadingBottom, encBottom90, encBottom180, 90, 180);
  }
  else if ((rawAngleReadingBottom > encBottom270) && (rawAngleReadingBottom <= encBottom180)) {
    calibratedAngleDataBottom = map(rawAngleReadingBottom, encBottom180, encBottom270, 180, 270);
  }
  else if ((rawAngleReadingBottom > encBottom360) && (rawAngleReadingBottom <= encBottom270)) {
    calibratedAngleDataBottom = map(rawAngleReadingBottom, encBottom270, encBottom360, 270, 360);
  }

  // Calibrates the angle of the top encoder so that it makes sense on 
  // a scale of 0-360 counter-clockwise off of the +x-axis
  // The calibration values differs from agent to agent on
  if((rawAngleReadingTop < encTop_180_first) && (rawAngleReadingTop <= 0)) {
    rawAngleReadingTop = rawAngleReadingTop + 360;
  }
  if ((rawAngleReadingTop > encTop_90) && (rawAngleReadingTop <= rawAngleReadingBottom)) {
    calibratedAngleDataTop = map(rawAngleReadingTop, rawAngleReadingBottom, encTop_90, 0, 90) + calibratedAngleDataBottom;
  }
  else if ((rawAngleReadingTop > encTop_180_first) && (rawAngleReadingTop <= encTop_90)) {
    calibratedAngleDataTop = map(rawAngleReadingTop, encTop_90, encTop_180_first, 90, 180) + calibratedAngleDataBottom;
  }
  else if ((rawAngleReadingTop > encBottom_360) && (rawAngleReadingTop <= encBottom_270)) {
    calibratedAngleDataBottom = map(rawAngleReadingTop, encBottom_270, encBottom_360, 270, 360);
  }



}


void SPIN(float thetaBottom, float desHeadingDirection, float deltaD) {
  float Kp = 1;//16; float Kd = 1; 
  float endTime = millis(); float previousTime = millis(); 

  while (thetaBottom*180/M_PI > (desHeadingDirection*180/M_PI)+5 || thetaBottom*180/M_PI < (desHeadingDirection*180/M_PI)-5)
  {
    Serial.println("START:");
    // Angle Formation //////////////////////////////////////////////////////////////////
    rawAngleReadingBottom = bottomEncoder.readAngle() * AS5600_RAW_TO_RADIANS * 180/M_PI;
    rawAngleReadingTop = topEncoder.readAngle() * AS5600_RAW_TO_RADIANS * 180/M_PI;

  //// ROBOT 4 
    // Calibrating the magnetic encoder for tether angles
    if(rawAngleReadingBottom > encBottom90 && rawAngleReadingBottom <= encBottom0) {theta0 = map(rawAngleReadingBottom, encBottom0, encBottom90, 0, 90);}
    else if (rawAngleReadingBottom > encBottom180 && rawAngleReadingBottom <= encBottom90) {theta0 = map(rawAngleReadingBottom, encBottom90, encBottom180, 90, 180);}
    else if (rawAngleReadingBottom > encBottom270 && rawAngleReadingBottom <= encBottom180) {theta0 = map(rawAngleReadingBottom, encBottom180, encBottom270, 180, 270);}
    else if (rawAngleReadingBottom > encBottom360 && rawAngleReadingBottom <= encBottom270) {theta0 = map(rawAngleReadingBottom, encBottom270, encBottom360, 270, 360);}

    //Serial.println(ang_T1);
    if (rawAngleReadingTop < T1_180first && rawAngleReadingTop >= 0) {rawAngleReadingTop = rawAngleReadingTop + 360;}
    if (rawAngleReadingTop > T1_90 && rawAngleReadingTop <= T1_0) {thetaTop = map(rawAngleReadingTop, T1_0, T1_90, 0, 90) + theta0;}
    else if (rawAngleReadingTop > T1_180first && rawAngleReadingTop <= T1_90) {thetaTop = map(rawAngleReadingTop, T1_90, T1_180first, 90, 180) + theta0;}
    else if (rawAngleReadingTop > T1_270 && rawAngleReadingTop <= T1_180second) {thetaTop = map(rawAngleReadingTop, T1_180second, T1_270, 180, 270) + theta0;}
    else if (rawAngleReadingTop > T1_360 && rawAngleReadingTop <= T1_270) {thetaTop = map(rawAngleReadingTop, T1_270, T1_360, 270, 360) + theta0;}
    if (thetaTop > 360) {thetaTop = thetaTop - 360;}


    thetaBottom = theta0 * M_PI/180;
    thetaTop = thetaTop * M_PI/180;

    if (thetaBottom < thetaTop) {deltaMeasured = thetaTop - thetaBottom;}
    else if (thetaBottom > thetaTop && thetaBottom >= (270 * M_PI/180) && thetaBottom <= (360 * M_PI/180) && thetaTop >= 0 && thetaTop <= (270 * M_PI/180))
    {deltaMeasured = (360*M_PI/180) - thetaBottom + thetaTop;}
    else {deltaMeasured = (360*M_PI/180) - abs(thetaTop - thetaBottom);}

    // PID Control
    dt = (millis() - previousTime) / 1000;

    errorE = -1*(desHeadingDirection - thetaBottom);
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
    delay(250); 
  }  

  Serial.println("DONE:");     
  analogWrite(RIGHT_MOTOR_FORWARD, 0);
  analogWrite(RIGHT_MOTOR_BACKWARD, 0);
  analogWrite(LEFT_MOTOR_FORWARD, 0);
  analogWrite(LEFT_MOTOR_BACKWARD, 0);
  delay(1000);
}


void DRIVE() { 

  // TODO: implement PID control for speed

  analogWrite(RIGHT_MOTOR_FORWARD, Right);
  analogWrite(RIGHT_MOTOR_BACKWARD, 0);
  analogWrite(LEFT_MOTOR_FORWARD, Left);
  analogWrite(LEFT_MOTOR_BACKWARD, 0);
  delay(100);

  analogWrite(RIGHT_MOTOR_FORWARD, 0);
  analogWrite(RIGHT_MOTOR_BACKWARD, 0);
  analogWrite(LEFT_MOTOR_FORWARD, 0);
  analogWrite(LEFT_MOTOR_BACKWARD, 0);
  delay(1000); 
}


//Function to momag_VE Right Motor 
void RIGHT_MOTOR(int lower) {
  // Pin A16 - Right, Forward
  if (PID_control_O > 0) {
    if (PID_control_O >= lower && PID_control_O <= 255) {
      analogWrite(RIGHT_MOTOR_FORWARD, PID_control_O);
      analogWrite(RIGHT_MOTOR_BACKWARD, 0);
    }
    else if (PID_control_O >= 0 && PID_control_O < lower) {
      analogWrite(RIGHT_MOTOR_FORWARD, lower);
      analogWrite(RIGHT_MOTOR_BACKWARD, 0);
    }
    else if (PID_control_O > 255) {
      analogWrite(RIGHT_MOTOR_FORWARD, 255);
      analogWrite(RIGHT_MOTOR_BACKWARD, 0);
    }
  }
  // Pin A15 - Right, Backward
  if (PID_control_O < 0) {
    if (abs(PID_control_O) >= lower && abs(PID_control_O) <= 255) {
      analogWrite(RIGHT_MOTOR_FORWARD, 0);
      analogWrite(RIGHT_MOTOR_BACKWARD, PID_control_O);
    }
    else if (abs(PID_control_O) >= 0 && abs(PID_control_O) < lower) {
      analogWrite(RIGHT_MOTOR_FORWARD, 0);
      analogWrite(RIGHT_MOTOR_BACKWARD, lower);
    }
    else if (abs(PID_control_O) > 255) {
      analogWrite(RIGHT_MOTOR_FORWARD, 0);
      analogWrite(RIGHT_MOTOR_BACKWARD, 255);
    }
  }
}


//Function to momag_VE Left Motor
void LEFT_MOTOR(int lower) {
  // Pin 13 - Left, Forward
  if (PID_control_O < 0) {
    if (abs(PID_control_O) >= lower && abs(PID_control_O) <= 255) {
      analogWrite(LEFT_MOTOR_FORWARD, abs(PID_control_O));
      analogWrite(LEFT_MOTOR_BACKWARD, 0);
    }
    else if (abs(PID_control_O) >= 0 && abs(PID_control_O) < lower) {
      analogWrite(LEFT_MOTOR_FORWARD, lower);
      analogWrite(LEFT_MOTOR_BACKWARD, 0);
    }
    else if (abs(PID_control_O) > 255) {
      analogWrite(LEFT_MOTOR_FORWARD, 255);
      analogWrite(LEFT_MOTOR_BACKWARD, 0);
    }
  }
  // Pin A14 - Left, Backward
  if (PID_control_O > 0) {
    if (abs(PID_control_O) >= lower && abs(PID_control_O) <= 255) {
      analogWrite(LEFT_MOTOR_FORWARD, 0);
      analogWrite(LEFT_MOTOR_BACKWARD, PID_control_O);
    }
    else if (abs(PID_control_O) >= 0 && abs(PID_control_O) < lower) {
      analogWrite(LEFT_MOTOR_FORWARD, 0);
      analogWrite(LEFT_MOTOR_BACKWARD, lower);
    }
    else if (abs(PID_control_O) > 255) {
      analogWrite(LEFT_MOTOR_FORWARD, 0);
      analogWrite(LEFT_MOTOR_BACKWARD, 255);
    }
  }
}

