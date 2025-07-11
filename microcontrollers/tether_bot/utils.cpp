/*
  utils.cpp

  This file contains some useful helper function implementations for calculations in tether_bot.ino
*/

#include "Arduino.h"
#include "utils.h"
#include <math.h>
#include <BasicLinearAlgebra.h>
using namespace BLA;

float toRadians(float degAngle) {
  return degAngle * M_PI / 180;
}

float toDegrees(float radAngle) {
  return radAngle * 180 / M_PI;
}

float mod(float x, float y) {
  // handle negative numbers in mod the same way python does
  float r = fmod(x, y);
  return (r < 0) ? r + y : r;
}

float smallestSignedAngleDiff(float startAng, float goalAng) {
  // return the smallest signed angle difference in degrees where + is CCW and - is CW
  return mod((goalAng - startAng) + 180, 360) - 180;
}

float sign(float x) {
  // return -1 if negative, 0 if 0, and +1 is positive
  return (x > 0) - (x < 0);
}

float vectorMagnitude(Matrix<2,1> vector) {
  return sqrt(vector(0)*vector(0) + vector(1)*vector(1));
}

Matrix<2,1> normalizeVector(Matrix<2,1> vector) {
  // normalize the 2-dimensional vector to a unit vector
  float magnitude = vectorMagnitude(vector);
  return (1 / magnitude) * vector;
}

float vectorDirection(Matrix<2,1> vector) {
  // calculate and return the angle direction in degrees (0-360 scale) of a vector
  return mod(toDegrees(atan2(vector(1), vector(0))), 360);
}