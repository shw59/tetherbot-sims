/*
  tether_bot.ino

  This code is to be flashed onto the Raspberry Pi Pico 2 W onboard the tetherbot. Make sure to change the encoder and flex sensor
  calibration values, desired delta angle, as well as select which tether(s) the robot uses to match the specific tetherbot before uploading. 

  Notes:
  - All angles are in radians unless otherwise specified
  - Tether m specified in simulation is always represented by the bottom tether on the hardware robot
  - If the tether bot is using both tethers, the bottom one (tether m) will always be the reference point for headings (both desired and current),
    use whichever tether is active otherwise (for end robots)
*/

#include "tether_bot_profiles.h"
#include <Wire.h>
#include "AS5600.h"
#include "utils.h"
#include <climits>
#include <BasicLinearAlgebra.h>
#include "math.h"
using namespace BLA;

// TODO: hook the pins up to the pico eventually instead of the artemis
// define pins for motors and sensors
// note: the encoder pins are already predefined in the Wire library based on the I2C address
# define RIGHT_MOTOR_FORWARD A16
# define RIGHT_MOTOR_BACKWARD A15
# define LEFT_MOTOR_FORWARD 13
# define LEFT_MOTOR_BACKWARD A14

# define TOP_FLEX_SENSOR A3
# define BOTTOM_FLEX_SENSOR A2

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
  int flexStraight; // raw flex sensor value when flex sensor is straight at 0 degrees
  int flexBent; // raw flex sensor value when flex sensor is bent at 90 degrees

  // calibrated and calculation-usable angles from flex sensor and encoder
  float flexAngle;
  float theta;

  // initialize tether with flex sensor and encoder calibration values
  Tether(const int pinFlexSensorVal, AS5600 encoderObj, int straightCalib, int bentCalib) 
  : pinFlexSensor(pinFlexSensorVal), encoder(encoderObj), flexStraight(straightCalib), flexBent(bentCalib) {}

  void readTetherSensors() {
    // read raw flex sensor values
    int rawFlexAngle = analogRead(pinFlexSensor);

    // calibrate raw flex sensor values to angles suitable for strain calculations (radians)
    flexAngle = map(rawFlexAngle, flexStraight, flexBent, 0, M_PI / 2);

    // read encoders and offset the top encoder to be independent of the bottom encoder's position
    float encAngleBottom = bottomEncoder.readAngle() * AS5600_RAW_TO_RADIANS;
    float encAngleTop = topEncoder.readAngle() * AS5600_RAW_TO_RADIANS;

    // check which encoder this specific tether is using and calibrate accordingly
    if (&encoder == &bottomEncoder) {
      theta = encAngleBottom;
    } else {
      // offset top encoder such that it is not dependent on the bottom encoder's position anymore
      theta = mod(encAngleTop + encAngleBottom, 2 * M_PI);

      if (abs(toDegrees(theta)) < 0.01) {
        theta = 2 * M_PI;
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
  TETHER_M_FLEX_STRAIGHT,
  TETHER_M_FLEX_BENT
);

Tether tetherTop(
  TOP_FLEX_SENSOR,
  topEncoder,
  TETHER_P_FLEX_STRAIGHT,
  TETHER_P_FLEX_BENT
);

const float ANGLE_WEIGHT = 1;
const float STRAIN_WEIGHT = 1;
const float GRADIENT_WEIGHT = 1;
const float REPULSION_WEIGHT = 1;

const int GOAL_FLEX_ANGLE = toRadians(90); // goal angle of the flex sensor to maintain strain
const int GOAL_DELTA = toRadians(DESIRED_DELTA);

const float ERR_ANGLE_HEADING = 10; // error tolerance for tether angles and headings
const float ERR_ANGLE_STRAIN = 15; // error tolerance for angle of the flex sensors

const int MIN_PWM = 145; // minimum PWM value that can be written to the motors

// PID over heading parameters
unsigned long prevTimeHeading = ULONG_MAX;
float prevErrorHeading;
int pidHeadingOut;

// PID over position parameters
unsigned long prevTimePosition = ULONG_MAX;
float prevErrorPosition;
int pidPositionOut;

float delta; // the current difference between the two tethers' angle thetas if both tethers are used (radians)

// intialize next-step vectors used for calculations
Matrix<2,1> vectorStrainBottom;
Matrix<2,1> vectorStrainTop;
Matrix<2,1> vectorAngle;

// resultant next-step vector robot will travel 
float desiredHeading; // the next-step desired heading of the robot in radians
float desiredMagnitude;


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
      driveMotors(-pidHeadingOut, pidHeadingOut); // positive pwm means CCW, negative means CW

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
  // read flex sensors and encoders and update sensor data variables
  tetherBottom.readTetherSensors();
  tetherTop.readTetherSensors();

  // update delta
  delta = mod(tetherBottom.theta - tetherTop.theta, 2 * M_PI);

  // TODO: if we add close-range sensors or other sensors in the future, read them here as well

}

void computeVectorStrain(Tether tether) {
  // initialize next-step strain-correction vector
  Matrix<2,1> vectorStrain;

  // calculate flex angle error (representing strain error)
  float strainError = tether.flexAngle - GOAL_FLEX_ANGLE;

  // set to 0 if error is small enough
  if (fabs(toDegrees(strainError)) > ERR_ANGLE_STRAIN) {
    float strainMagnitude = sign(strainError) * (strainError * strainError);

    // assemble the strain vector
    vectorStrain = {strainMagnitude * cos(tether.theta), strainMagnitude * sin(tether.theta)};
  } else {
    vectorStrain = {0, 0};
  }

  // assign to the corresponding global variable
  if (&tether == &tetherBottom) {
    vectorStrainBottom = vectorStrain;
  } else {
    vectorStrainTop = vectorStrain;
  } 
}

void computeVectorAngle() {
  // calculate error in delta
  float deltaError = GOAL_DELTA - delta;

  // keep angle vector at 0 if error it is close enough to the goal angle
  if (fabs(toDegrees(deltaError)) > ERR_ANGLE_HEADING) {
    float angleMagnitude = sign(deltaError) * sqrt(fabs(deltaError) / (2 * M_PI));

    // get unit vectors for the tether headings
    Matrix<2,1> tetherVectorBottom = {cos(tetherBottom.theta), sin(tetherBottom.theta)};
    Matrix<2,1> tetherVectorTop = {cos(tetherTop.theta), sin(tetherTop.theta)};

    // add tether unit vectors
    Matrix<2,1> angleHeadingUnitVector = normalizeVector(tetherVectorBottom + tetherVectorTop);

    // if delta is roughly 180 degrees, make the next vector heading the perpendicular vector between them following the direction the robot is currently facing
    if (delta <= (180 + ERR_ANGLE_HEADING) && delta >= (180 - ERR_ANGLE_HEADING)) {
      // take the larger theta and rotate it counter clockwise by 90 degrees to get the new vector direction
      float perpHeading;
      if (tetherTop.theta > tetherBottom.theta) {
        perpHeading = mod(tetherTop.theta + (M_PI / 2), 2 * M_PI);
      } else {
        perpHeading = mod(tetherBottom.theta + (M_PI / 2), 2 * M_PI);
      }

      angleHeadingUnitVector = {cos(perpHeading), sin(perpHeading)};
    }

    // assemble the angle vector
    vectorAngle = angleMagnitude * angleHeadingUnitVector;
  } else {
    vectorAngle = {0, 0};
  }
}

void computeNextStep() {
  Serial.println("Computing next step");
  delay(2000);
  // calculate strain and angle vectors
  if (TETHER_M && TETHER_P) {
    computeVectorAngle();
    computeVectorStrain(tetherBottom);
    computeVectorStrain(tetherTop);
  } else if (TETHER_M) {
    computeVectorStrain(tetherBottom);
  } else {
    computeVectorStrain(tetherTop);
  }

  Matrix<2,1> resultant = STRAIN_WEIGHT * (vectorStrainBottom + vectorStrainTop) + ANGLE_WEIGHT * vectorAngle;

  desiredHeading = vectorDirection(resultant);
  desiredMagnitude = vectorMagnitude(resultant);

  // TODO: might need to limit the magnitude based on how big it gets

}

void updateHeadingPID() {
  Serial.println("Updating PID heading");
  delay(2000);
  const float Kp = 1;
  const float Kd = 1;
  unsigned long now = millis();
  float dt = (now - prevTimeHeading) / 1000.0; // convert to seconds
  if (dt <= 0) dt = 0.001; // prevent division by zero

  // uses bottom tether as reference theta by default unless bottom tether is not being used
  float tetherTheta = tetherBottom.theta;
  if (!TETHER_M) {
    tetherTheta = tetherTop.theta;
  }

  // convert the desired heading to a desired theta value relative to either the bottom or top tether
  float desiredTheta = tetherTheta - desiredHeading;
  if (desiredTheta < 0) {
    desiredTheta += 2 * M_PI;
  }

  // heading error calculation using smallest signed-angle difference
  float errorHeading = mod(tetherTheta - desiredTheta + M_PI, 2 * M_PI) - M_PI;

  // if the error is within tolerance, consider the desired heading to be reached and stop moving
  if (fabs(toDegrees(errorHeading)) <= ERR_ANGLE_HEADING) {
    prevTimeHeading = ULONG_MAX;
    prevErrorHeading = 0;
    pidHeadingOut = 0.0;
    return;
  }

  float derivative = (errorHeading - prevErrorHeading) / dt;

  if (prevTimeHeading == ULONG_MAX) { // if this is the first PID update call in a sequence, then no derivative term
    derivative = 0;
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

  // TODO: implement PID feedback control on speed/position maybe using wheel encoders and call it from loop() under DRIVING state

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

