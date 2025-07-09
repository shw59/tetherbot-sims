/*
  utils.h

  This header file contains function definitions for useful helper functions that can be used in tether_bot.ino for calculations.
*/

#ifndef UTILS_H
#define UTILS_H

#include <BasicLinearAlgebra.h>
using namespace BLA;

float toRadians(float degAngle);

float toDegrees(float radAngle);

float mod(float x, float y);

int sign(float x);

float vectorMagnitude(Matrix<2,1> vector);

Matrix<2,1> normalizeVector(Matrix<2,1> vector);

float vectorDirection(Matrix<2,1> vector);

#endif
