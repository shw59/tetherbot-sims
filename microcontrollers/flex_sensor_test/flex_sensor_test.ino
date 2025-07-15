/*
  flex_sensor_test.ino

  This is a test file to ensure that the flex sensors are calibrated properly.
*/

#include <Wire.h>
#include <BasicLinearAlgebra.h>
#include "math.h"
using namespace BLA;

#define BOTTOM_FLEX_SENSOR A2
#define TOP_FLEX_SENSOR A3

#define FLEX_VALUE_BOTTOM_STRAIGHT 455
#define FLEX_VALUE_BOTTOM_BENT 260

#define FLEX_VALUE_TOP_STRAIGHT 456
#define FLEX_VALUE_TOP_BENT 280

int flexAngleBottom;
int flexAngleTop;


void setup() {
  Serial.begin(115200);
  pinMode(A2, INPUT);
  pinMode(A3, INPUT);
}

void loop() {
  int rawFlexValueBottom = analogRead(BOTTOM_FLEX_SENSOR);
  int rawFlexValueTop = analogRead(TOP_FLEX_SENSOR);

  flexAngleBottom = map(rawFlexValueBottom, FLEX_VALUE_BOTTOM_STRAIGHT, FLEX_VALUE_BOTTOM_BENT, 0, 90);
  flexAngleTop = map(rawFlexValueTop, FLEX_VALUE_TOP_STRAIGHT, FLEX_VALUE_TOP_BENT, 0, 90);

  Serial.print("Raw Flex Value Top: ");
  Serial.println(rawFlexValueTop);

  Serial.print("Flex Angle Top: ");
  Serial.println(flexAngleTop);

  Serial.print("Raw Flex Value Bottom: ");
  Serial.println(rawFlexValueBottom);

  Serial.print("Flex Angle Bottom: ");
  Serial.println(flexAngleBottom);

  Serial.println();

  delay(2000);
}