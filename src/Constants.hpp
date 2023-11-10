
#ifndef P28_CONSTANTS_HPP_
#define P28_CONSTANTS_HPP_

#include "CompileFlags.hpp"

/*
    All physical constants of the robot in SI units where applicable
*/

namespace p28 {

// Physical dimensions of the robot 
#ifdef AQUAMAN
constexpr float kRobotWidth = 0.19;              // m
#else
constexpr float kRobotWidth = 0.1850;              // m (distance between the motor wheels)
#endif
constexpr float kRobotWidth_2 = kRobotWidth/2;  // m
constexpr float kWheelRadius = 0.0383;           // m
constexpr int kTicksPerRotation = 3200;
constexpr float kCircumference = PI * kRobotWidth;
constexpr float kColorSensorToCenter = 0.047; // distance forward of the color sensor (m)
constexpr float kLineSensorToCenter = 0.07; // distance forward of the line sensor (m)
constexpr int kIRSensor_apartDist = 213; // mm
constexpr float kIRSensorBack_centerOffset = 84; // mm
constexpr int kCup_openAngle = 15;
constexpr int kCup_closeAngle = 180;
constexpr int kArm_openAngle = 20; // #define
constexpr int kArm_closeAngle = 150; // #define
constexpr int kClaw_openAngle = 0; // #define
constexpr int kClaw_closeAngle = 30; // #define 


// Physical movement constraints of the robot
constexpr float kMaxVel = 0.4;                  // m / s
constexpr float kAccel = 0.4; // m/s^2
constexpr float kMaxAngularVelocity = 0.8; // rad/s
constexpr float KMinVel = 0.03;
constexpr float kEndSegmentVel = 0.08;
constexpr float kFollowLineBaseVelocity = 0.1;


// Control constants 
constexpr float kMotorHarwareStateMixFactor = 0.85; // Used for exponential moving average between 2 hardware states
constexpr unsigned int kControlLoopDelay = 50; // ms
constexpr float kPathFollower_distEpsilon2 = 0.0001; // square of precision (1cm)
constexpr float kPathFollower_headingEpsilon2 = 0.00005; // square of precision


// Code constants and constructs
using time_t = long unsigned int;
constexpr float kInfinity = 100000;
constexpr unsigned int kMaxCheckPointForPath = 20;
enum class COLOR { RED, GREEN, BLUE, YELLOW, BLACK, WHITE };

 

// Mission specific constants
constexpr int kCupRelease_pathIndex = 3; // After which checkpoint of the maneuver should the cup be dropped?

constexpr int kCupDetectDist = 350; // mm
constexpr int kKnockCupDelay = 1500; // ms

// Bin motors
constexpr float kcolor_selected_angle_open = 180; // nombre a determiner

} // !p28

#endif