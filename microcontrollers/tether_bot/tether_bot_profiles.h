/*
tether_bot_profiles.h

This defines the attributes specific to each tether bot in the demo and can be used interchangably in tether_bot.ino depending
on which tether bot you are flashing code to.
*/

#ifndef TETHER_BOT_PROFILES_H
#define TETHER_BOT_PROFILES_H

// uncomment the tether bot being used
// #define ROBOT_0
// #define ROBOT_1
// #define ROBOT_2
// #define ROBOT_3
#define ROBOT_4
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

  #define GOAL_DELTA 0
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

  #define GOAL_DELTA 180
#endif

#ifdef ROBOT_2 // flex sensors don't work
  #define TETHER_M 1
  #define TETHER_P 1

  #define TETHER_M_ENC_OFFSET 173
  #define TETHER_P_ENC_OFFSET 240

  #define TETHER_M_FLEX_STRAIGHT
  #define TETHER_M_FLEX_BENT

  #define TETHER_P_FLEX_STRAIGHT
  #define TETHER_P_FLEX_BENT

  #define GOAL_DELTA 0
#endif

#ifdef ROBOT_3 // right wheel doesn't rotate
  #define TETHER_M 1
  #define TETHER_P 0

  #define TETHER_M_ENC_OFFSET 350
  #define TETHER_P_ENC_OFFSET 235

  #define TETHER_M_FLEX_STRAIGHT 470
  #define TETHER_M_FLEX_BENT 270

  #define TETHER_P_FLEX_STRAIGHT 440
  #define TETHER_P_FLEX_BENT 270

  #define GOAL_DELTA 0
#endif

#ifdef ROBOT_4
  #define TETHER_M 1
  #define TETHER_P 0

  #define TETHER_M_ENC_OFFSET 100
  #define TETHER_P_ENC_OFFSET 150

  #define TETHER_M_FLEX_STRAIGHT 470
  #define TETHER_M_FLEX_BENT 270

  #define TETHER_P_FLEX_STRAIGHT 440
  #define TETHER_P_FLEX_BENT 270

  #define GOAL_DELTA 0
#endif

#ifdef ROBOT_5
  #define TETHER_M 0
  #define TETHER_P 1
#endif

#endif // TETHER_BOT_PROFILES_H