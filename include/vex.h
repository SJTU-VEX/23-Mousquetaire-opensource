/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       vex.h                                                     */
/*    Author:       Vex Robotics                                              */
/*    Created:      1 Feb 2019                                                */
/*    Description:  Default header for V5 projects                            */
/*                                                                            */
/*----------------------------------------------------------------------------*/
//
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "v5.h"
#include "v5_vcs.h"

#include "robot-config.h"

#define waitUntil(condition)                                                   \
  do {                                                                         \
    wait(5, msec);                                                             \
  } while (!(condition))

#define repeat(iterations)                                                     \
  for (int iterator = 0; iterator < iterations; iterator++)
  

/*---------------------------------FOR  DEBUG-------------------------------------*/
// 选择自动模式                                                                    //
#define AUTO                                                                      //
// #define FIFTEEN                                                                //
// #define COMPETITION_LEFT                                                       //
#define COMPETITION_RIGHT                                                         //
// #define COMPETITION_RIGHT_Oliver                                                         //
// #define SKILL                                                                  //
                                                                                  //
// 选择手动模式                                                                    //
// #define MANUAL                                                                 //
                                                                                  //
//是否开启自动防止四盘策略                                                          //
// #define FourBarProcess                                                         //
/*-------------------------------------------------------------------------------*/

// #define COMPETITION

// #define RED_ALLIANCE
#define BLUE_ALLIANCE

// #define debug