/*
  tether_bot.ino

  This code is to be flashed onto the Raspberry Pi Pico 2 W onboard the tetherbot. Make sure to change the encoder and flex sensor
  calibration values, desired delta angle, as well as select which tether(s) the robot uses to match the specific tetherbot before uploading. 

  Notes:
  - All angles are in degrees unless otherwise specified
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
  AS5600& encoder; // AS5600 magnetic encoder object

  // flex sensor calibration values
  int flexStraight; // raw flex sensor value when flex sensor is straight at 0 degrees
  int flexBent; // raw flex sensor value when flex sensor is bent at 90 degrees

  // calibrated and calculation-usable angles from flex sensor and encoder
  float flexAngle;
  float theta;

  // initialize tether with flex sensor and encoder calibration values
  Tether(const int pinFlexSensorVal, AS5600& encoderObj, int straightCalib, int bentCalib) 
  : pinFlexSensor(pinFlexSensorVal), encoder(encoderObj), flexStraight(straightCalib), flexBent(bentCalib) {}

  void readTetherSensors() {
    // read raw flex sensor values
    int rawFlexAngle;
    if (&encoder == &bottomEncoder) {
      rawFlexAngle = analogRead(BOTTOM_FLEX_SENSOR);
    } else {
      rawFlexAngle = analogRead(TOP_FLEX_SENSOR);
    }
    
    Serial.print("Raw Flex Reading: ");
    Serial.println(rawFlexAngle);

    // calibrate raw flex sensor values to angles suitable for strain calculations (degrees)
    flexAngle = map(rawFlexAngle, flexStraight, flexBent, 0, 90);

    Serial.print("Flex Angle: ");
    Serial.println(flexAngle);

    // read encoders and offset the top encoder to be independent of the bottom encoder's position
    float encAngleBottom = bottomEncoder.readAngle() * AS5600_RAW_TO_DEGREES;
    float encAngleTop = topEncoder.readAngle() * AS5600_RAW_TO_DEGREES;

    // check which encoder this specific tether is using and calibrate accordingly
    if (&encoder == &bottomEncoder) {
      theta = encAngleBottom;
    } else {
      // offset top encoder such that it is not dependent on the bottom encoder's position anymore
      theta = mod(encAngleTop + encAngleBottom, 360);

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
  TETHER_M_FLEX_STRAIGHT,
  TETHER_M_FLEX_BENT
);

Tether tetherTop(
  TOP_FLEX_SENSOR,
  topEncoder,
  TETHER_P_FLEX_STRAIGHT,
  TETHER_P_FLEX_BENT
);

constexpr float ANGLE_WEIGHT = 7;
constexpr float STRAIN_WEIGHT = 14;
constexpr float GRADIENT_WEIGHT = 1;
constexpr float REPULSION_WEIGHT = 1;

const float GOAL_FLEX_ANGLE = 57; // goal angle of the flex sensor to maintain strain

constexpr float ERR_ANGLE_HEADING = 10; // error tolerance for tether angles and headings
constexpr float ERR_ANGLE_STRAIN = 10; // error tolerance for angle of the flex sensors
constexpr float ERR_PID = 20; // PWM error threshold
constexpr float ERR_OVERALL = 2; // error tolerance for overall correction vector magnitude
 
constexpr int MIN_PWM = 145; // minimum PWM value that can be written to the motors

// PID over heading parameters
unsigned long prevTimeHeading = ULONG_MAX;
float prevErrorHeading;
float pidHeadingOut;
int pidHeadingPwm; // final pwm value based on the PID heading value

// PID over position parameters
unsigned long prevTimePosition = ULONG_MAX;
float prevErrorPosition;
int pidPositionOut;
int pidPositionPwm; // final pwm value based on the PID position value

float delta; // the current difference between the two tethers' angle thetas if both tethers are used (radians)

// intialize next-step vectors used for calculations
Matrix<2,1> vectorStrainBottom;
Matrix<2,1> vectorStrainTop;
Matrix<2,1> vectorAngle;

// initialize the reference theta variable for the heading
float thetaTetherRef;

// resultant next-step vector robot will travel 
float desiredHeading;
float desiredTheta; // the next-step desired theta value for one of the tethers (determined by desired heading)
float desiredMagnitude;

// kill switch (serial message)
bool isKilled = false;


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

  vectorStrainBottom = {0, 0};
  vectorStrainTop = {0, 0};
  vectorAngle = {0, 0};

  delay(30000);
}

void loop() {
  // serial message control ('p' for pause, 's' for start, 'r' for reset)
  if (Serial.available() > 0) {
    char cmd = Serial.read();

    switch (cmd) {
      case 'p':
        Serial.println("PAUSED");
        isKilled = true;
        break;
      case 's':
        Serial.println("RESUMED");
        isKilled = false;
        break;
      case 'r':
        Serial.println("RESET");
        NVIC_SystemReset();
    }
  }

  if (isKilled) {
    driveMotors(0, 0);
    return;
  }

  readSensors(); // read all sensors and update sensor data variables

  Serial.print("Delta: ");
  Serial.println(delta);

  switch (currState) {
    case IDLE:
      Serial.println("IDLE");
      // compute and set resultant vector for next-step movement
      computeNextStep();

      Serial.print("Angle Vector: ");
      Serial.println(vectorAngle);

      Serial.print("Strain Vector P: ");
      Serial.println(vectorStrainTop);

      Serial.print("Strain Vector m: ");
      Serial.println(vectorStrainBottom);

      // set robot to begin spinning to face desired heading, stop if reached all goals
      if (desiredHeading > ERR_OVERALL) {
        currState = SPINNING;
      }

      break;
    case SPINNING:
      Serial.println("SPINNING");
      // update PID-corrected PWM output
      updateHeadingPID();

      Serial.print("PID Heading PWM Raw: ");
      Serial.println(pidHeadingOut);

      // ensure output is within valid PWM value limits
      clampOutputPID(pidHeadingOut, MIN_PWM);

      // spin robot accordingly
      driveMotors(-pidHeadingPwm, pidHeadingPwm); // positive pwm means CCW, negative means CW

      // robot is facing desired direction, move on to driving forward state if there is a magnitude
      if (!pidHeadingOut) {
        if (desiredMagnitude > 0.0) {
          currState = DRIVING;
        } else {
          currState = IDLE;
        }
      }

      break;
    case DRIVING:
      Serial.println("DRIVING");
      // TODO: do PID control feedback on speed/position here and set next state to IDLE once robot has reached the desired goals in terms of tether angle/strain
      // update PID-corrected PWM output
      // updatePositionPID();
      
      // drive robot forward accordingly
      // driveMotors(pidPositionOut, pidPositionOut);=
      driveMotors(150, 150);
      delay(500);
      driveMotors(0, 0);

      // if robot has reached desired position, move on to idle state
      currState = IDLE;
      // if (!pidPositionOut) {
      //   currState = IDLE;
      // }

      break;
    }

}

void readSensors() {
  // read flex sensors and encoders and update sensor data variables
  tetherBottom.readTetherSensors();
  tetherTop.readTetherSensors();

  // update delta
  delta = smallestSignedAngleDiff(tetherTop.theta, tetherBottom.theta);

  // set the bottom tether theta as reference theta for heading by default unless bottom tether is not being used
  thetaTetherRef = tetherBottom.theta;
  if (!TETHER_M) {
    thetaTetherRef = tetherTop.theta;
  }

  // TODO: if we add close-range sensors or other sensors in the future, read them here as well

}

void computeVectorStrain(Tether& tether) {
  // initialize next-step strain-correction vector
  Matrix<2,1> vectorStrain;

  // calculate flex angle error (representing strain error)
  float strainError = GOAL_FLEX_ANGLE - tether.flexAngle;

  Serial.print("Current Flex Angle: ");
  Serial.println(tether.flexAngle);

  Serial.print("Goal Flex Angle: ");
  Serial.println(GOAL_FLEX_ANGLE);

  Serial.print("Strain Error: ");
  Serial.println(strainError);

  // set to 0 if error is small enough
  if (fabs(strainError) > ERR_ANGLE_STRAIN) {
    float strainMagnitude = sign(strainError) * (strainError * strainError);

    // assemble the strain vector
    vectorStrain = {strainMagnitude * cos(toRadians(tether.theta)), strainMagnitude * sin(toRadians(tether.theta))};
  } else {
    vectorStrain = {0, 0};
  }

  // assign to the corresponding global variable
  if (&tether == &tetherBottom) {
    vectorStrainBottom = 0.01f * vectorStrain;
  } else {
    vectorStrainTop = 0.01f * vectorStrain;
  } 
}

void computeVectorAngle() {
  // calculate error in delta
  float deltaError = GOAL_DELTA - delta;

  Serial.print("Delta Error: ");
  Serial.println(deltaError);

  // keep angle vector at 0 if error it is close enough to the goal angle
  if (fabs(deltaError) > ERR_ANGLE_HEADING) {
    float angleMagnitude = sign(deltaError) * sqrt(fabs(deltaError) / 360);

    // get unit vectors for the tether headings
    Matrix<2,1> tetherVectorBottom = {cos(toRadians(tetherBottom.theta)), sin(toRadians(tetherBottom.theta))};
    Matrix<2,1> tetherVectorTop = {cos(toRadians(tetherTop.theta)), sin(toRadians(tetherTop.theta))};

    // add tether unit vectors
    Matrix<2,1> angleHeadingUnitVector = normalizeVector(tetherVectorBottom) + normalizeVector(tetherVectorTop);

    // if delta is roughly 180 degrees, make the next vector heading the perpendicular vector between them following the direction the robot is currently facing
    if (round(fabs(delta)) == 180.0) {
      // take the larger theta and rotate it counter clockwise by 90 degrees to get the new vector direction
      float perpHeading;
      if (tetherTop.theta > tetherBottom.theta) {
        perpHeading = mod(tetherTop.theta + 90, 360);
      } else {
        perpHeading = mod(tetherBottom.theta + 90, 360);
      }

      angleHeadingUnitVector = {cos(toRadians(perpHeading)), sin(toRadians(perpHeading))};
    }

    Serial.print("Tether Direction Unit Vector: ");
    Serial.println(angleHeadingUnitVector);

    // assemble the angle vector
    vectorAngle = 1000.0f * angleMagnitude * angleHeadingUnitVector;
  } else {
    vectorAngle = {0, 0};
  }
}

void computeNextStep() {
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

  Serial.print("Resultant Vector:" );
  Serial.println(resultant);

  desiredHeading = vectorDirection(resultant);
  desiredMagnitude = vectorMagnitude(resultant);

  // convert the desired heading to a desired theta value relative to either the bottom or top tether
  desiredTheta = thetaTetherRef - desiredHeading;
  if (desiredTheta < 0) {
    desiredTheta += 360;
  }

  Serial.print("Desired Theta: ");
  Serial.println(desiredTheta);
  Serial.print("Desired Heading: ");
  Serial.println(desiredHeading);

  // TODO: might need to limit the magnitude based on how big it gets

}

void updateHeadingPID() {
  const float Kp = 0.5;
  const float Kd = 0.1;
  unsigned long now = millis();
  float dt = (now - prevTimeHeading) / 1000.0; // convert to seconds
  if (dt <= 0) dt = 0.001; // prevent division by zero

  // heading error calculation using smallest signed-angle difference (in degrees for more reasonable pwm values)
  float errorHeading = smallestSignedAngleDiff(desiredTheta, thetaTetherRef);
  int pwmScaledErrHeading = sign(errorHeading) * map(fabs(errorHeading), 0, 180, 0, 255);

  Serial.print("Theta of Ref Tether: ");
  Serial.println(thetaTetherRef);
  Serial.print("Heading Error: ");
  Serial.println(errorHeading);

  float derivative = (errorHeading - prevErrorHeading) / dt;

  if (prevTimeHeading == ULONG_MAX) { // if this is the first PID update call in a sequence, then no derivative term
    derivative = 0;
  }

  // PID control output
  pidHeadingOut = Kp * pwmScaledErrHeading + Kd * derivative;

  // if the error is within tolerance or the PID raw value is small, consider the desired heading to be reached and stop moving
  if (fabs(errorHeading) <= ERR_ANGLE_HEADING || fabs(pidHeadingOut) < ERR_PID) {
    prevTimeHeading = ULONG_MAX;
    prevErrorHeading = 0;
    pidHeadingOut = 0;
    return;
  }

  Serial.print("Raw PID output: ");
  Serial.println(pidHeadingOut);

  prevTimeHeading = now;
  prevErrorHeading = errorHeading;
}

void updatePositionPID() { 
  return;
  // TODO: implement PID feedback control on speed/position maybe using wheel encoders and call it from loop() under DRIVING state

}

void clampOutputPID(int pidOut, int minPwm) {
  // note: changing pidOut will directly change the original variable entered in (pass by reference)
  if (pidOut == 0) {
    pidHeadingPwm = 0;
  } else if (abs(pidOut) < minPwm) {
    // set to minimum output if pid output is lower than minimum
    pidHeadingPwm = minPwm;
  } else if (abs(pidOut) > 255) {
    // limit output to between -255 and 255
    pidHeadingPwm = sign(pidOut) * 255;
  } else {
    pidHeadingPwm = pidOut;
  }
}

void driveMotors(int pwmLeft, int pwmRight) {
  // if left motor pwm is negative, go backwards by that pwm value
  if (pwmLeft < 0) {
    analogWrite(LEFT_MOTOR_BACKWARD, abs(pwmLeft));
    analogWrite(LEFT_MOTOR_FORWARD, 0);
  } else {
    analogWrite(LEFT_MOTOR_FORWARD, pwmLeft);
    analogWrite(LEFT_MOTOR_BACKWARD, 0);
  }

  // if right motor pwm is negative, go backwards by that pwm value
  if (pwmRight < 0) {
    analogWrite(RIGHT_MOTOR_BACKWARD, abs(pwmRight));
    analogWrite(RIGHT_MOTOR_FORWARD, 0);
  } else {
    analogWrite(RIGHT_MOTOR_FORWARD, pwmRight);
    analogWrite(RIGHT_MOTOR_BACKWARD, 0);
  }
}

