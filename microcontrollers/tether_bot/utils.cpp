/*
  utils.cpp

  This file contains some useful helper function implementations for calculations in tether_bot.ino
*/

#include "Arduino.h"
#include "utils.h"

float mod(float x, float y) {
  // handle negative numbers in mod the same way python does
  float r = fmod(x, y);
  return (r < 0) ? r + y : r;
}
