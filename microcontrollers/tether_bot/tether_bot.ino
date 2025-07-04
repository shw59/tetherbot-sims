/*
tether_bot.ino

This code is to be flashed onto the Raspberry Pi Pico 2 W onboard the tetherbot. Make sure to change the encoder and flex sensor
calibration values, desired delta angle, as well as select which tether(s) the robot uses to match the specific tetherbot before uploading. 
*/

#include <WiFi.h>
#include <Wire.h>
#include "AS5600.h"
#include <BasicLinearAlgebra.h>
#include "math.h"
using namespace BLA;


// define pins for motors and sensors 
// note: the encoder pins are already predefined in the AS5600 library based on the I2C address)
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

    // read raw encoder angles (in degrees)
    int rawEncAngle = encoder.readAngle() * AS5600_RAW_TO_RADIANS * 180/M_PI;

    // TODO: maybe make the calibration values into an array and loop through it or something to check the ranges
    // wrap raw encoder angles to ensure they are continuous over the 0/360 degree boundary for calculation purposes
    if (rawEncAngle < enc90 && rawEncAngle >= 0) {
      rawEncAngle = rawEncAngle + 360;
    }

    // calibrate raw encoder angles to angles suitable for tether angle calculations
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

// initialize tethers with their respective flex sensor pins, encoders, and calibration values
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

// PID control parameters
float pidControlOut; // motor PWM output from PID control

// PID over heading parameters
unsigned long prevTimeHeading = ULONG_MAX;
float prevErrorHeading;

float delta; // the current difference between the two tethers' angle thetas if both tethers are used

float desiredHeading; // the next-step desired heading of the robot in degrees with respect to one of its tethers



// TODO: most of the following variables could probably become local variables, but we can sort that out once we implement everything

float vectorAngleMag; // magnitude of the calculated next-step angle vector
float vectorAngleDir; // direction of the calculated next-step angle vector


// HEADING VECTORS (PID corrected)
float desHeadingMagnitude; // magnitude of robot's next-step heading vector
float desHeadingDirection; // direction of the robot's next-step heading vector


// RESULTANT VECTORS
float resultantMagnitude; // magnitude of the next-step resultant vector
float resultantDirection; // direction of the next-step resultant vector


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
      updateHeadingPID(desiredHeading, tetherBottom.theta);

      // apply control
      int lowerLimitPWM = 145;
      driveMotors(RIGHT_MOTOR_FORWARD, RIGHT_MOTOR_BACKWARD, lowerLimitPWM);
      driveMotors(LEFT_MOTOR_FORWARD, LEFT_MOTOR_BACKWARD, lowerLimitPWM);

      // if robot is facing desired direction, stop spinning and start driving forward
      if (!pidControlOut) {
        currState = DRIVING;
      }
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

void computeDelta() {
    // compute the difference between the two tethers' angles
    delta = tetherTop.theta - tetherBottom.theta;
    
    // handle angle wrap-around
    if (delta > 180) delta -= 360;
    if (delta < -180) delta += 360;
}

// all headings and angles are in degrees
void updateHeadingPID(float desHeading, float currHeading) {
  const float Kp = 1;
  const float Kd = 1;
  const float errTolerance = 5;
  unsigned long now = millis();
  float dt = (now - prevTimeHeading) / 1000.0; // convert to seconds
  if (dt <= 0) dt = 0.001; // prevent division by zero

  // error calculation (angle wrap-around handling)
  float errorHeading = desiredHeading - currentHeading;
  if (errorHeading > 180) errorHeading -= 360;
  if (errorHeading < -180) errorHeading += 360;

  // if the error is within tolerance, consider the desired heading to be reached and stop moving
  if (fabs(errorHeading) <= errTolerance) {
    prevTimeHeading = ULONG_MAX;
    prevErrorHeading = 0;
    pidControlOut = 0; // stop the motors
    return;
  }

  if (prevTimeHeading == ULLONG_MAX) { // if this is the first PID update call in a sequence
    float derivative = 0;
  } else {
    float derivative = (errorHeading - prevErrorHeading) / dt;
  }

  // PID control output
  pidControlOut = Kp * errorHeading + Kd * derivative;

  // limit control output
  if (pidControlOut > 255) pidControlOut = 255;
  if (pidControlOut < -255) pidControlOut = -255;

  prevTimeHeading = now;
  prevErrorHeading = errorHeading;
}

// TODO: implement PID feedback control on speed/position (no loops, just the feedback part) and call it from loop() under DRIVING state
void updateSpeedPID() { 
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

// TODO: generalize/fix this so that it can drive both motors forward based on PID of speed/position that will be implemented in driveForward()
void driveMotors(int pinForward, int pinBackward, int lowerLimitPwm) {
  if (pidControlOut > 0) { // forward direction
    int pwmValue = pidControlOut;
    if (pwmValue > 255) pwmValue = 255;
    if (pwmValue < lowerLimitPwm) pwmValue = lowerLimitPwm;
    analogWrite(pinForward, pwmValue);
    analogWrite(pinBackward, 0);
  }
  else if (pidControlOut < 0) { // backward direction
    int pwmValue = abs(pidControlOut);
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

