/*
  tether_bot.ino

  This code is to be flashed onto the Raspberry Pi Pico 2 W onboard the tetherbot. Make sure to change the encoder and flex sensor
  calibration values, desired delta angle, as well as select which tether(s) the robot uses to match the specific tetherbot before uploading. 

  Notes:
  - All angles are in degrees except inside the vector calculation functions unless specified otherwise
  - Tether m specified in simulation is always represented by the bottom tether on the hardware robot
  - If the tether bot is using both tethers, the bottom one (tether m) will always be the reference point for headings (both desired and current),
    use whichever tether is active otherwise (for end robots)
*/

#include "tether_bot_profiles.h"
#include <WiFi.h>
#include <Wire.h>
#include "AS5600.h"
#include <climits>
#include <BasicLinearAlgebra.h>
#include "math.h"
using namespace BLA;


// define pins for motors and sensors
// note: the encoder pins are already predefined in the Wire library based on the I2C address
# define RIGHT_MOTOR_FORWARD 0
# define RIGHT_MOTOR_BACKWARD 1
# define LEFT_MOTOR_FORWARD 2
# define LEFT_MOTOR_BACKWARD 3

# define TOP_FLEX_SENSOR 4
# define BOTTOM_FLEX_SENSOR 5

// initialize objects for the tether encoders
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

  // calibrated and calculation-usable angles from flex sensor and encoder
  float flexAngle;
  float theta;

  // initialize tether with flex sensor and encoder calibration values
  Tether(const int pinFlexSensorVal, AS5600 encoderObj, int straightCalib, int bentCalib) 
  : pinFlexSensor(pinFlexSensorVal), encoder(encoderObj), straight(straightCalib), bent(bentCalib) {}

  void readTetherSensors() {
    // read raw flex sensor values
    int rawFlexAngle = analogRead(pinFlexSensor);

    // calibrate raw flex sensor values to angles suitable for strain calculations
    flexAngle = map(rawFlexAngle, straight, bent, 0, 90);

    // read encoders and offset the top encoder to be independent of the bottom encoder's position
    float encAngleBottom = bottomEncoder.readAngle() * AS5600_RAW_TO_DEGREES;
    float encAngleTop = topEncoder.readAngle() * AS5600_RAW_TO_DEGREES;

    // check which encoder this specific tether is using and calibrate accordingly
    if (&encoder == &bottomEncoder) {
      theta = encAngleBottom;
    } else {
      // offset top encoder such that it is not dependent on the bottom encoder's position anymore
      theta = fmod(encAngleTop + encAngleBottom, 360);

      if (abs(theta) < 0.01) {
        theta = 360;
      }
    }
  }

};

// initialize current state of the robot
TetherBotState currState = IDLE;

// TODO: make both tethers' calibration values specific to robot (make choosable)
// also maybe recalibrate so that 0 degrees is heading of the robot to match simulation

// initialize tethers with their respective flex sensor pins, encoders, and calibration values
Tether tetherBottom(
  BOTTOM_FLEX_SENSOR,
  bottomEncoder,
  420, // straight
  300 // bent
);

Tether tetherTop(
  TOP_FLEX_SENSOR,
  topEncoder,
  410, // straight
  300 // bent
);

const int MIN_PWM = 145; // minimum PWM value that can be written to the motors

// PID over heading parameters
unsigned long prevTimeHeading = ULONG_MAX;
float prevErrorHeading;
int pidHeadingOut;

// PID over position parameters
unsigned long prevTimePosition = ULONG_MAX;
float prevErrorPosition;
int pidPositionOut;

float delta; // the current difference between the two tethers' angle thetas if both tethers are used

float desiredTheta; // the next-step desired heading of the robot in degrees (relative to theta of one of the tethers)
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

  bottomEncoder.begin();
  bottomEncoder.setDirection(AS5600_COUNTERCLOCK_WISE); // set direction pin
  bottomEncoder.setOffset(TETHER_M_ENC_OFFSET); // set encoder calibration offset
  Serial.print("Bottom Encoder: ");
  Serial.println(bottomEncoder.isConnected() ? "connected" : "not connected");
  delay(1000);

  topEncoder.begin();
  topEncoder.setDirection(AS5600_COUNTERCLOCK_WISE); // set direction pin
  topEncoder.setOffset(TETHER_P_ENC_OFFSET); // set encoder calibration offset
  Serial.print("Top Encoder: ");
  Serial.println(topEncoder.isConnected() ? "connected" : "not connected");
  delay(1000);
}

void loop() {
  readSensors(); // read all sensors and update sensor data variables

  switch (currState) {
    case IDLE:
      Serial.println("IDLE");
      delay(2000);
      // TODO: compute and set the next step vector + direction, then set current state to SPINNING or DRIVING depending on next step
      // compute and set resultant vector for next-step movement
      computeNextStep();

      // set robot to begin spinning to face desired heading
      currState = SPINNING;

      break;
    case SPINNING:
      Serial.println("SPINNING");
      delay(2000);
      // update PID-corrected PWM output
      updateHeadingPID();

      // spin robot accordingly
      driveMotors(-pidHeadingOut, pidHeadingOut);

      // robot is facing desired direction, move on to driving forward state
      if (!pidHeadingOut) {
        currState = DRIVING;
      }

      break;
    case DRIVING:
      Serial.println("DRIVING");
      delay(2000);
      // TODO: do PID control feedback on speed/position here and set next state to IDLE once robot has reached the desired goals in terms of tether angle/strain
      // update PID-corrected PWM output
      updatePositionPID();
      
      // drive robot forward accordingly
      driveMotors(pidPositionOut, pidPositionOut);

      // if robot has reached desired position, move on to idle state
      if (!pidPositionOut) {
        currState = IDLE;
      }

      break;
    }

}

void readSensors() {
  Serial.println("Reading sensors");
  delay(2000);
  // read flex sensors and use encoder sensor info to update data accordingly
  tetherBottom.readTetherSensors();
  tetherTop.readTetherSensors();

  // TODO: if we add close-range sensors or other sensors in the future, read them here as well

}

// TODO: implement the vector computation stuff
void computeVectorStrain() {
  return;
}

void computeVectorAngle() {
  return;
}

void computeNextStep() {
  Serial.println("Computing next step");
  delay(2000);
  // TODO: make it choosable whether we are computing with one or both tethers
  return;
}

void computeDelta() {
  delta = tetherBottom.theta - tetherTop.theta;

  if (delta < 0) {
    delta += 360;
  }
}

// all headings and angles are in degrees
void updateHeadingPID() {
  Serial.println("Updating PID heading");
  delay(2000);
  const float Kp = 1;
  const float Kd = 1;
  const float errorTolerance = 5;
  unsigned long now = millis();
  float dt = (now - prevTimeHeading) / 1000.0; // convert to seconds
  if (dt <= 0) dt = 0.001; // prevent division by zero

  // error calculation (angle wrap-around handling)
  float errorHeading = desiredTheta - tetherBottom.theta; 
  if (errorHeading > 180) errorHeading -= 360;
  if (errorHeading < -180) errorHeading += 360;

  // if the error is within tolerance, consider the desired heading to be reached and stop moving
  if (fabs(errorHeading) <= errorTolerance) {
    prevTimeHeading = ULONG_MAX;
    prevErrorHeading = 0;
    pidHeadingOut = 0.0;
    return;
  }

  float derivative = (errorHeading - prevErrorHeading) / dt;

  if (prevTimeHeading == ULONG_MAX) { // if this is the first PID update call in a sequence, then no derivative term
    float derivative = 0;
  }

  // PID control output
  pidHeadingOut = Kp * errorHeading + Kd * derivative;

  // ensure output is within valid PWM value limits
  clampOutputPID(pidHeadingOut, MIN_PWM);

  prevTimeHeading = now;
  prevErrorHeading = errorHeading;
}

void updatePositionPID() { 
  Serial.println("Updating PID position");
  delay(2000);
  const float Kp = 1;
  const float Kd = 1;
  const float errorTolerance = 0.1;
  unsigned long now = millis();
  float dt = (now - prevTimePosition) / 1000.0; // convert to seconds
  if (dt <= 0) dt = 0.001; // prevent division by zero

  // TODO: implement PID feedback control on speed/position maybe using wheel encoders? (no loops, just the feedback part) and call it from loop() under DRIVING state

  clampOutputPID(pidPositionOut, MIN_PWM);



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
    Serial.println("left motor backwards");
    delay(2000);
    analogWrite(LEFT_MOTOR_BACKWARD, abs(pwmLeft));
  } else {
    Serial.println("left motor forwards");
    delay(2000);
    analogWrite(LEFT_MOTOR_FORWARD, pwmLeft);
  }

  // if right motor pwm is negative, go backwards by that pwm value
  if (pwmRight < 0) {
    Serial.println("right motor backwards");
    delay(2000);
    analogWrite(RIGHT_MOTOR_BACKWARD, abs(pwmRight));
  } else {
    Serial.println("right motor forwards");
    delay(2000);
    analogWrite(RIGHT_MOTOR_FORWARD, pwmRight);
  }
}

