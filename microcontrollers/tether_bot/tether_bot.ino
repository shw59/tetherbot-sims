/*
tether_bot.ino

This code is to be flashed onto the Raspberry Pi Pico 2 W onboard the tetherbot. Make sure to change the encoder and flex sensor
calibration values, as well as select which tether(s) the robot uses to match the specific tetherbot before uploading. 
*/

#include <WiFi.h>
#include <Wire.h>
#include "AS5600.h"
#include <BasicLinearAlgebra.h>
#include "math.h"
using namespace BLA;


// define pins for motors and sensors 
// note: the encoder pins are already predefined in the AS5600 library based on the I2C address)=
# define RIGHT_MOTOR_FORWARD GP0
# define RIGHT_MOTOR_BACKWARD GP1
# define LEFT_MOTOR_FORWARD GP2
# define LEFT_MOTOR_BACKWARD GP3

# define TOP_FLEX_SENSOR GP4
# define BOTTOM_FLEX_SENSOR GP5

// initialize the AS5600 encoder objects for I2C communication
AS5600 bottomEncoder(&Wire);
AS5600 topEncoder(&Wire1);

enum TetherBotState {
  IDLE,
  SPINNING,
  DRIVING // driving forward
};

struct Tether {
  const int pinFlexSensor; // pin for the flex sensor
  AS5600 encoder; // AS5600 magnetic encoder object

  // flex sensor calibration values
  int straight;
  int bent;

  // encoder calibration raw values at specific angles
  float enc0;
  float enc90;
  float enc180;
  float enc270;
  float enc360;

  // calibrated and calculation-usable angles from flex sensor and encoder
  float flexAngle;
  float theta;

  // magnitude and direction of the next-step strain vector
  float vectorStrainMag;
  float vectorStrainDir;

  // initialize tether with flex sensor and encoder calibration values
  Tether(int pinFlexSensor, AS5600 encoder, int straight, int bent, float enc0, float enc90, float enc180, float enc270, float enc360) {
    this->pinFlexSensor = pinFlexSensor;
    this->encoder = encoder;
    this->straight = straight;
    this->bent = bent;
    this->enc0 = enc0;
    this->enc90 = enc90;
    this->enc180 = enc180;
    this->enc270 = enc270;
    this->enc360 = enc360;
  }

  void readTetherSensors() {
    // read raw flex sensor values
    int rawFlexAngle = analogRead(pinFlexSensor);

    // calibrate raw flex sensor values to angles suitable for strain calculations
    flexAngle = map(rawFlexAngle, straight, bent, 0, 90);

    // read raw encoder angles
    int rawEncAngle = encoder.readAngle() * AS5600_RAW_TO_RADIANS * 180/M_PI;

    // calibrate raw encoder angles to angles suitable for tether angle calculations
    // TODO: maybe make the calibration values into an array and loop through it or something to check the ranges
    if (rawEncAngle < enc90 && rawEncAngle >= 0) {
      rawEncAngle = rawEncAngle + 360;
    }
    if (rawEncAngle > enc90 && rawEncAngle <= enc0) {
      theta = map(rawEncAngle, enc0, enc90, 0, 90);
    }
    else if (rawEncAngle > enc180 && rawEncAngle <= enc90second) {
      theta = map(rawEncAngle, enc90second, enc180, 90, 180);
    }
    else if (rawEncAngle > enc270 && rawEncAngle <= enc180) {
      theta = map(rawEncAngle, enc180, enc270, 180, 270);
    }
    else if (rawEncAngle > enc360 && rawEncAngle <= enc270) {
      theta = map(rawEncAngle, enc270, enc360, 270, 360);
    }
  }

};

// initialize current state of the robot
TetherBotState currState = IDLE;

// TODO: make both tethers' calibration values specific to robot (make choosable)
// also make option for either one or both tethers to be intialized/used
Tether tetherBottom(
  BOTTOM_FLEX_SENSOR,
  bottomEncoder,
  420, // straight
  300, // bent
  360, // enc0
  262, // enc90
  176, // enc180
  86,  // enc270
  0    // enc360
);

Tether tetherTop(
  TOP_FLEX_SENSOR,
  topEncoder,
  410, // straight
  300, // bent
  210, // enc0
  122, // enc90
  30,  // enc180_first
  390, // enc180_second
  300  // enc270
);


// TODO: most of the following variables could probably become local variables, but we can sort that out once we implement everything

float deltaMeasured; // The difference between calibratedAngleDataBottom and calibratedAngleDataTop
float deltaError; // The error value between the desired and measured deltas

float vectorAngleMag; // magnitude of the calculated next-step angle vector
float vectorAngleDir; // direction of the calculated next-step angle vector


// HEADING VECTORS (PID corrected)
float desHeadingMagnitude; // magnitude of robot's next-step heading vector
float desHeadingDirection; // direction of the robot's next-step heading vector


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
float goalFlexAngle = 90; // the desired angle of the flex sensor to maintan desired tether strain
float lowerAngle = 45;
float upperAngle = 80;


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
  readSensors(); // read all sensors and update sensor data variables

  switch (currState) {
    case IDLE:
      // TODO: compute next step vector + direction, then set current state to SPINNING or DRIVING depending on next step
    case SPINNING:
      // TODO: do PID control feedback on heading here and set next state to DRIVING once robot is facing the desired direction
    case DRIVING:
      // TODO: do PID control feedback on speed/position here and set next state to IDLE once robot has reached the desired goals in terms of tether angle/strain
  }

}

void readSensors() {
  // read tether sensors including encoders and flex sensors and update data accordingly
  tetherBottom.readTetherSensors();
  tetherTop.readTetherSensors();

  // TODO: if we add close-range sensors or other sensors in the future, read them here as well

}

// TODO: get rid of the while loop and just have the if statement stuff for PID, then call it from loop() under SPINNING state
// should probably rename it to PID feedback on heading or somethin like that
void spin(float thetaBottom, float desHeadingDirection, float deltaD) {
  float Kp = 1;//16; float Kd = 1; 
  float endTime = millis(); float previousTime = millis(); 

  while (thetaBottom*180/M_PI > (desHeadingDirection*180/M_PI)+5 || thetaBottom*180/M_PI < (desHeadingDirection*180/M_PI)-5)
  {
    Serial.println("START:");
    // Angle Formation //////////////////////////////////////////////////////////////////
    rawEncAngleBottom = bottomEncoder.readAngle() * AS5600_RAW_TO_RADIANS * 180/M_PI;
    rawEncAngleTop = topEncoder.readAngle() * AS5600_RAW_TO_RADIANS * 180/M_PI;

  //// ROBOT 4 
    // Calibrating the magnetic encoder for tether angles
    if(rawEncAngleBottom > encBottom90 && rawEncAngleBottom <= encBottom0) {theta0 = map(rawEncAngleBottom, encBottom0, encBottom90, 0, 90);}
    else if (rawEncAngleBottom > encBottom180 && rawEncAngleBottom <= encBottom90) {theta0 = map(rawEncAngleBottom, encBottom90, encBottom180, 90, 180);}
    else if (rawEncAngleBottom > encBottom270 && rawEncAngleBottom <= encBottom180) {theta0 = map(rawEncAngleBottom, encBottom180, encBottom270, 180, 270);}
    else if (rawEncAngleBottom > encBottom360 && rawEncAngleBottom <= encBottom270) {theta0 = map(rawEncAngleBottom, encBottom270, encBottom360, 270, 360);}

    //Serial.println(ang_T1);
    if (rawEncAngleTop < T1_180first && rawEncAngleTop >= 0) {rawEncAngleTop = rawEncAngleTop + 360;}
    if (rawEncAngleTop > T1_90 && rawEncAngleTop <= T1_0) {thetaTop = map(rawEncAngleTop, T1_0, T1_90, 0, 90) + theta0;}
    else if (rawEncAngleTop > T1_180first && rawEncAngleTop <= T1_90) {thetaTop = map(rawEncAngleTop, T1_90, T1_180first, 90, 180) + theta0;}
    else if (rawEncAngleTop > T1_270 && rawEncAngleTop <= T1_180second) {thetaTop = map(rawEncAngleTop, T1_180second, T1_270, 180, 270) + theta0;}
    else if (rawEncAngleTop > T1_360 && rawEncAngleTop <= T1_270) {thetaTop = map(rawEncAngleTop, T1_270, T1_360, 270, 360) + theta0;}
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
    rightMotor(lowerR);                   
    leftMotor(lowerL); 

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

// TODO: implement PID feedback control on speed/position (no loops, just the feedback part) and call it from loop() under DRIVING state
void driveForward() { 
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

// TODO: generalize this so that it can drive both motors forward based on PID of speed/position that will be implemented in driveForward()
void driveMotor(int pinForward, int pinBackward, int lowerLimitPwm) {
  if (PID_control_O > 0) { // forward direction
    int pwmValue = PID_control_O;
    if (pwmValue > 255) pwmValue = 255;
    if (pwmValue < lowerLimitPwm) pwmValue = lowerLimitPwm;
    analogWrite(pinForward, pwmValue);
    analogWrite(pinBackward, 0);
  }
  else if (PID_control_O < 0) { // backward direction
    int pwmValue = abs(PID_control_O);
    if (pwmValue > 255) pwmValue = 255;
    if (pwmValue < lowerLimitPwm) pwmValue = lowerLimitPwm;
    analogWrite(pinForward, 0);
    analogWrite(pinBackward, pwmValue);
  }
  else { // stop motor if 0
    analogWrite(pinForward, 0);
    analogWrite(pinBackward, 0);
  }
}

