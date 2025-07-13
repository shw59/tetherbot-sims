/*
  tether_encoder_test.ino

  This is a test to ensure the tether encoders are correctly calibrated to measure theta and delta values.
*/

#include <Wire.h>
#include "AS5600.h"
#include <BasicLinearAlgebra.h>
#include "math.h"
using namespace BLA;

#define ENC_OFFSET_BOTTOM 190
#define ENC_OFFSET_TOP 192

AS5600 bottomEncoder(&Wire);
AS5600 topEncoder(&Wire1);

float encAngleBottom;
float encAngleTop;

float thetaBottom;
float thetaTop;

float delta;

void setup() {
  Serial.begin(115200);
  
  Wire.begin();
  Wire1.begin();

  bottomEncoder.begin();
  bottomEncoder.setDirection(AS5600_COUNTERCLOCK_WISE); // set direction pin
  bottomEncoder.setOffset(ENC_OFFSET_BOTTOM); // set calibration offset
  Serial.println(bottomEncoder.getAddress(),HEX);
  Serial.print("Bottom Encoder: ");
  Serial.println(bottomEncoder.isConnected() ? "connected" : "not connected");
  delay(1000);

  topEncoder.begin();
  topEncoder.setDirection(AS5600_COUNTERCLOCK_WISE); // set direction pin
  topEncoder.setOffset(ENC_OFFSET_TOP); // set calibration offset
  Serial.println(topEncoder.getAddress(),HEX);
  Serial.print("Top Encoder: ");
  Serial.println(topEncoder.isConnected() ? "connected" : "not connected");
  delay(1000);
}

void loop() {
  encAngleBottom = bottomEncoder.readAngle() * AS5600_RAW_TO_DEGREES;
  encAngleTop = topEncoder.readAngle() * AS5600_RAW_TO_DEGREES;

  thetaBottom = encAngleBottom;
  thetaTop = fmod((encAngleTop + thetaBottom), 360.0);

  if (abs(thetaTop) < 0.01) {
    thetaTop = 360;
  }

  delta = thetaBottom - thetaTop;

  Serial.print("Top Encoder Angle: ");
  Serial.println(encAngleTop);

  Serial.print("Top Theta: ");
  Serial.println(thetaTop);

  Serial.print("Bottom Encoder Angle: ");
  Serial.println(encAngleBottom);

  Serial.print("Bottom Theta: ");
  Serial.println(thetaBottom);

  Serial.print("Delta: ");
  Serial.println(delta);

  Serial.println();

  delay(2000);
}
