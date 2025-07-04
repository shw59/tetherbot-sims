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

const int MIN_PWM = 145; // minimum PWM value that can be written to the motors

// PID over heading parameters
unsigned long prevTimeHeading = ULONG_MAX;
float prevErrorHeading;
int pidHeadingOut;

float delta; // the current difference between the two tethers' angle thetas if both tethers are used

float desiredHeading; // the next-step desired heading of the robot in degrees
float desiredMagnitude; // desired next-step speed/amount to travel

// Goal Angle and Upper Stop Angle
float goalFlexAngle = 90; // the desired angle of the flex sensor to maintan desired tether strain
float lowerAngle = 45; // should we still keep these
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
      // update PID-corrected PWM output
      updateHeadingPID(desiredHeading, tetherBottom.theta);

      // spin robot accordingly
      driveMotors(-pidHeadingOUt, pidHeadingOut);

      // robot is facing desired direction, move on to driving forward state
      if (pidHeadingOut == 0.0) {
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
  float delta = tetherBottom.theta - tetherTop.theta;

  if (delta < 0)
    delta = 360 + delta;
}

// all headings and angles are in degrees
float updateHeadingPID(float desHeading, float currHeading) {
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
    pidHeadingOut = 0.0;
    return;
  }

  if (prevTimeHeading == ULLONG_MAX) { // if this is the first PID update call in a sequence
    float derivative = 0;
  } else {
    float derivative = (errorHeading - prevErrorHeading) / dt;
  }

  // PID control output
  pidHeadingOut = Kp * errorHeading + Kd * derivative;

  // ensure output is within valid PWM value limits
  clampOutputPID(pidHeadingOut, MIN_PWM);

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

void clampOutputPID(int& pidOut, int minPwm) {
  // note: changing pidOut will directly change the original variable entered in (pass by reference)
  if (abs(pidOut) < minPwm) {
    // set to minimum output if pid output is lower than minimum
    pidOut = minPwm;
  } else if (abs(pidOut) > 255) {
    // limit output to between -255 and 255
    int sign = (pidOut > 0) - (pidOut < 0);
    pidOut = sign * 255;
}
}

void driveMotors(int pwmLeft, int pwmRight) {
  // if left motor pwm is negative, go backwards by that pwm value
  if (pwmLeft < 0) {
    analogWrite(LEFT_MOTOR_BACKWARD, abs(pwmLeft));
  } else {
    analogWrite(LEFT_MOTOR_FORWARD, pwmLeft);
  }

  // if right motor pwm is negative, go backwards by that pwm value
  if (pwmRight < 0) {
    analogWrite(RIGHT_MOTOR_BACKWARD, abs(pwmRight));
  } else {
    analogWrite(RIGHT_MOTOR_FORWARD, pwmRight);
  }
}

