/*
tether_bot_profiles.h

This defines the attributes specific to each tether bot in the demo and can be used interchangably in tether_bot.ino depending
on which tether bot you are flashing code to.
*/

#ifndef TETHER_BOT_PROFILES_H
#define TETHER_BOT_PROFILES_H

// uncomment the tether bot being used
// #define ROBOT_1
#define ROBOT_2
//#define ROBOT_3

#ifdef ROBOT_1
  #define TETHER_M 0
  #define TETHER_P 1
#endif

#ifdef ROBOT_2
  #define TETHER_M 1
  #define TETHER_P 1
#endif

#ifdef ROBOT_3
  #define TETHER_M 1
  #define TETHER_P 1
#endif

#ifdef ROBOT_4
  #define TETHER_M 1
  #define TETHER_P 1
#endif

#ifdef ROBOT_5
  #define TETHER_M 1
  #define TETHER_P 1
#endif

#ifdef ROBOT_6
  #define TETHER_M 1
  #define TETHER_P 0
#endif

#endif