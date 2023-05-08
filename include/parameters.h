#ifndef PARAMETERS_H_
#define PARAMETERS_H_
#include "calc.h"
#include "vex.h"
#include <math.h>
#include <fstream>

const double PI = M_PI;
const float WheelRadius = 3.492;                // 主动轮半径 cm
const float TrackingWheelRadius = 3.492;        // 定位轮半径 cm

const double LEncoderAngle = acos(157.6291667 / 220);
const double REncoderAngle = acos(155.4553333 / 220);

const double IMUCoefficient = 3600 / 3594;

extern double rightCLCoefficient;
extern double leftCLCoefficient;
extern double leftCCCoefficient;
extern double rightCCCoefficient;

// const double targetPosX = 15;
// const double targetPosY = 297;

const double targetPosX = 283;
const double targetPosY = 20;

extern bool CollectFlag;
extern bool TestCaliberFlag;
extern bool isL;

extern bool fourBarFlag;

extern bool testFlyWheelFlag;

const float RefreshTime = 10;                   // 刷新时间 ms
const float positionRefreshTime = 10;           // 定位刷新时间 ms

const int CENTER_FOV = 150;
// centre at 158

// #define VisionSensor__SIG_TAR VisionSensor__SIG_BLUE

#ifdef RED_ALLIANCE
const float color_range[2] = {340, 50};
#define VisionSensor__SIG_TAR VisionSensor__SIG_RED
#endif

#ifdef BLUE_ALLIANCE
const float color_range[2] = {190, 280};
#define VisionSensor__SIG_TAR VisionSensor__SIG_BLUE
#endif

#endif