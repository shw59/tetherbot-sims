/*
tether_bot_profiles.h


This defines the attributes specific to each tether bot in the demo and can be used interchangably in tether_bot.ino depending
on which tether bot you are flashing code to.
*/

// CHANGED THE FLEX SENSOR CALLIBRATION RANGE FROM 0-->90 TO 45-->90


#ifndef TETHER_BOT_PROFILES_H
#define TETHER_BOT_PROFILES_H


// uncomment the tether bot being used
// #define ROBOT_0
// #define ROBOT_1
// #define ROBOT_2
#define ROBOT_3
// #define ROBOT_4
// #define ROBOT_5


#ifdef ROBOT_0
 #define FOLLOWS_GRADIENT 0

 #define TETHER_M 1
 #define TETHER_P 0

 #define TETHER_M_ENC_OFFSET 290
 #define TETHER_P_ENC_OFFSET 191

 #define TETHER_M_FLEX_STRAIGHT 350
 #define TETHER_M_FLEX_BENT 270

 #define TETHER_P_FLEX_STRAIGHT 155
 #define TETHER_P_FLEX_BENT 91

 #define GOAL_DELTA 0
#endif


#ifdef ROBOT_1
 #define FOLLOWS_GRADIENT 0

 #define TETHER_M 1
 #define TETHER_P 1

 #define TETHER_M_ENC_OFFSET 221.3
 #define TETHER_P_ENC_OFFSET 65.5

 #define TETHER_M_FLEX_STRAIGHT 230
 #define TETHER_M_FLEX_BENT 154

 #define TETHER_P_FLEX_STRAIGHT 255
 #define TETHER_P_FLEX_BENT 193

 #define GOAL_DELTA 180
#endif


#ifdef ROBOT_2
 #define FOLLOWS_GRADIENT 1

 #define TETHER_M 0
 #define TETHER_P 1

 #define TETHER_M_ENC_OFFSET 250
 #define TETHER_P_ENC_OFFSET 141

 #define TETHER_M_FLEX_STRAIGHT 330
 #define TETHER_M_FLEX_BENT 263

 #define TETHER_P_FLEX_STRAIGHT 376
 #define TETHER_P_FLEX_BENT 315

 #define GOAL_DELTA 0
#endif


#ifdef ROBOT_3
 #define FOLLOWS_GRADIENT 0

 #define TETHER_M 1
 #define TETHER_P 1

 #define TETHER_M_ENC_OFFSET 186.5
 #define TETHER_P_ENC_OFFSET 290

 #define TETHER_M_FLEX_STRAIGHT 345
 #define TETHER_M_FLEX_BENT 256

 #define TETHER_P_FLEX_STRAIGHT 347
 #define TETHER_P_FLEX_BENT 283

 #define GOAL_DELTA 180
#endif


#ifdef ROBOT_4
 #define FOLLOWS_GRADIENT 0

 #define TETHER_M 1
 #define TETHER_P 1

 #define TETHER_M_ENC_OFFSET 175.7
 #define TETHER_P_ENC_OFFSET 151.6

 #define TETHER_M_FLEX_STRAIGHT 360
 #define TETHER_M_FLEX_BENT 285

 #define TETHER_P_FLEX_STRAIGHT 315
 #define TETHER_P_FLEX_BENT 255

 #define GOAL_DELTA 180
#endif


#endif // TETHER_BOT_PROFILES_H