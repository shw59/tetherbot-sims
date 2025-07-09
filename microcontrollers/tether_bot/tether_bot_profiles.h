/*
tether_bot_profiles.h

This defines the attributes specific to each tether bot in the demo and can be used interchangably in tether_bot.ino depending
on which tether bot you are flashing code to.
*/

#ifndef TETHER_BOT_PROFILES_H
#define TETHER_BOT_PROFILES_H

// uncomment the tether bot being used
#define ROBOT_0
// #define ROBOT_1
// #define ROBOT_2
// #define ROBOT_3
// #define ROBOT_4
// #define ROBOT_5

#ifdef ROBOT_0
  #define TETHER_M 0
  #define TETHER_P 1

  #define TETHER_M_ENC_OFFSET 190
  #define TETHER_P_ENC_OFFSET 192

  #define TETHER_M_FLEX_STRAIGHT 450
  #define TETHER_M_FLEX_BENT 270

  #define TETHER_P_FLEX_STRAIGHT 310
  #define TETHER_P_FLEX_BENT 100

  #define DESIRED_DELTA 0
#endif

#ifdef ROBOT_1
  #define TETHER_M 1
  #define TETHER_P 1

  #define TETHER_M_ENC_OFFSET 0
  #define TETHER_P_ENC_OFFSET 130

  #define TETHER_M_FLEX_STRAIGHT 360
  #define TETHER_M_FLEX_BENT 180

  #define TETHER_P_FLEX_STRAIGHT 440
  #define TETHER_P_FLEX_BENT 240

  #define DESIRED_DELTA 90
#endif

#ifdef ROBOT_2
  #define TETHER_M 1
  #define TETHER_P 1
#endif

#ifdef ROBOT_3
  #define TETHER_M 1
  #define TETHER_P 0

  #define TETHER_M_ENC_OFFSET 350
  #define TETHER_P_ENC_OFFSET 235

  #define TETHER_M_FLEX_STRAIGHT 470
  #define TETHER_M_FLEX_BENT 270

  #define TETHER_P_FLEX_STRAIGHT 440
  #define TETHER_P_FLEX_BENT 270

  #define DESIRED_DELTA 0
#endif

#ifdef ROBOT_4
  #define TETHER_M 1
  #define TETHER_P 1

  #define TETHER_M_ENC_OFFSET 100
  #define TETHER_P_ENC_OFFSET 150

  #define TETHER_M_FLEX_STRAIGHT 470
  #define TETHER_M_FLEX_BENT 270

  #define TETHER_P_FLEX_STRAIGHT 440
  #define TETHER_P_FLEX_BENT 270

  #define DESIRED_DELTA 90
#endif

#ifdef ROBOT_5
  #define TETHER_M 1
  #define TETHER_P 1
#endif

#ifdef ROBOT_6
  #define TETHER_M 1
  #define TETHER_P 0
#endif

#endif // TETHER_BOT_PROFILES_H