/*
drive_test.ino

This file is a quick test to make sure the motors work as expected to drive the tether bot.
*/

#include <Wire.h>

# define RIGHT_MOTOR_FORWARD A16
# define RIGHT_MOTOR_BACKWARD A15
# define LEFT_MOTOR_FORWARD 13
# define LEFT_MOTOR_BACKWARD A14


void setup() {
  Serial.begin(115200);

  pinMode(RIGHT_MOTOR_FORWARD, OUTPUT);
  pinMode(RIGHT_MOTOR_BACKWARD, OUTPUT);
  pinMode(LEFT_MOTOR_FORWARD, OUTPUT);
  pinMode(RIGHT_MOTOR_BACKWARD, OUTPUT);
}

void loop() {
  driveMotors(150, 150); 
  Serial.println("Driving forward");

  delay(2000);

  driveMotors(0, 0);
  Serial.println("Stop");

  delay(2000);

  driveMotors(-150, 150); // counter-clockwise on-axis turn
  Serial.println("CCW turn");

  delay(2000);

  driveMotors(0, 0);
  Serial.println("Stop");

  delay(2000);

  driveMotors(150, 150);
  Serial.println("Driving forward");

  delay(2000);

  driveMotors(0, 0);
  Serial.println("Stop");

  delay(2000);

  driveMotors(150, -150); // clockwise on-axis turn
  Serial.println("CW turn");

  delay(2000);

  driveMotors(0, 0);
  Serial.println("Stop");

  delay(2000);
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