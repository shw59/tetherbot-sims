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
  #define TETHER_M 1
  #define TETHER_P 1

  #define TETHER_M_ENC_OFFSET 290
  #define TETHER_P_ENC_OFFSET 190

  #define TETHER_M_FLEX_STRAIGHT 420
  #define TETHER_M_FLEX_BENT 270

  #define TETHER_P_FLEX_STRAIGHT 301
  #define TETHER_P_FLEX_BENT 102

  #define GOAL_DELTA 90
#endif

#ifdef ROBOT_1
  #define TETHER_M 0
  #define TETHER_P 1

  #define TETHER_M_ENC_OFFSET 5
  #define TETHER_P_ENC_OFFSET 125

  #define TETHER_M_FLEX_STRAIGHT 390
  #define TETHER_M_FLEX_BENT 169

  #define TETHER_P_FLEX_STRAIGHT 415
  #define TETHER_P_FLEX_BENT 195

  #define GOAL_DELTA 0
#endif

#ifdef ROBOT_2
  #define TETHER_M 1
  #define TETHER_P 1

  #define TETHER_M_ENC_OFFSET 250
  #define TETHER_P_ENC_OFFSET 245

  #define TETHER_M_FLEX_STRAIGHT 452
  #define TETHER_M_FLEX_BENT 278

  #define TETHER_P_FLEX_STRAIGHT 465
  #define TETHER_P_FLEX_BENT 270

  #define GOAL_DELTA 90
#endif

#ifdef ROBOT_3 // right wheel doesn't rotate
  #define TETHER_M 1
  #define TETHER_P 1

  #define TETHER_M_ENC_OFFSET 70
  #define TETHER_P_ENC_OFFSET 230

  #define TETHER_M_FLEX_STRAIGHT 455
  #define TETHER_M_FLEX_BENT 260

  #define TETHER_P_FLEX_STRAIGHT 456
  #define TETHER_P_FLEX_BENT 280

  #define GOAL_DELTA 90
#endif

#ifdef ROBOT_4
  #define TETHER_M 1
  #define TETHER_P 0

  #define TETHER_M_ENC_OFFSET 170
  #define TETHER_P_ENC_OFFSET 155

  #define TETHER_M_FLEX_STRAIGHT 470
  #define TETHER_M_FLEX_BENT 260

  #define TETHER_P_FLEX_STRAIGHT 425
  #define TETHER_P_FLEX_BENT 260

  #define GOAL_DELTA 0
#endif

#endif // TETHER_BOT_PROFILES_H