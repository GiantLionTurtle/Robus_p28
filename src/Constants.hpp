
#ifndef P28_CONSTANTS_HPP_
#define P28_CONSTANTS_HPP_

#include "CompileFlags.hpp"
#include <Utils/Geometry.hpp>

#include <Arduino.h>

/*
    All physical constants of the robot in SI units where applicable
*/

namespace p28 {

// Physical dimensions of the robot 
#ifdef AQUAMAN
constexpr float kRobotWidth = 0.192;              // m
#else
constexpr float kRobotWidth = 0.1850;              // m (distance between the motor wheels)
#endif
constexpr float kRobotWidth_2 = kRobotWidth/2;  // m
constexpr float kWheelRadius = 0.0382;           // m
constexpr int kTicksPerRotation = 3200;
constexpr float kCircumference = PI * kRobotWidth;
constexpr float kColorSensorToCenter = 0.047; // distance forward of the color sensor (m)
constexpr float kLineSensorToCenter = 0.07; // distance forward of the line sensor (m)
constexpr int kIRSensor_apartDist = 213; // mm
constexpr float kIRSensorBack_centerOffset = 84; // mm
constexpr int kArm_restAngle = 47; 
constexpr int kArm_downAngle = 30;
constexpr int kArm_upAngle = 180; 
constexpr int kClaw_openAngle = 82; 
constexpr int kClaw_closeAngle = 40; 
constexpr int kConveyor_stepsPerRevolution = 2038;
// Must be multiple of 3 cause it's a 4 pin stepper
constexpr int kConveyor_stepsUntilUp = 5190;//kConveyor_stepsPerRevolution*2.4; 


// Physical movement constraints of the robot
constexpr float kMaxVel = 0.4;                  // m / s
constexpr float kAccel = 0.4; // m/s^2
constexpr float kMaxAngularVelocity = 0.8; // rad/s
constexpr float KMinVel = 0.03;
constexpr float kEndSegmentVel = 0.08;
constexpr float kFollowLineBaseVel = 0.1;
constexpr float kFollowCamBaseVel = 0.08;
constexpr float kFollowCamVelCoef = 0.4;
constexpr float kTurnSpeed = 0.08;

// Control constants 
constexpr float kMotorHarwareStateMixFactor = 0.85; // Used for exponential moving average between 2 hardware states
constexpr unsigned int kControlLoopDelay = 10; // ms
constexpr float kPathFollower_distEpsilon2 = 0.0001; // square of precision (1cm)
constexpr float kPathFollower_headingEpsilon2 = 0.00005; // square of precision
constexpr float kConveyor_speed = 5; //in RPM #define

// Code constants and constructs
using time_t = long unsigned int;
constexpr float kInfinity = 100000;
constexpr unsigned int kMaxCheckPointForPath = 20;
enum class COLOR { RED, GREEN, BLUE, YELLOW, BLACK, WHITE };

// Bin motors
constexpr int kcolor_selected_angle_open = 65; // nombre a determiner
constexpr int kopen_trap_angle = 0;
constexpr int kclosed_trap_angle = 70; 
constexpr int kopen_bin_angle = 65;  // for the 3 bin servos
constexpr int kclosed_bin_angle = 10;

//Color code for the controller
constexpr int kRed = 0;
constexpr int KGreen = 1;
constexpr int kBlue = 2;
constexpr int kYellow = 3;
constexpr int kAllColors = 4;

constexpr int kDumpPointId = 57;

namespace Tracking {

constexpr float kAlignedEnough_magPx2 = 0.8;
constexpr int kBackOff_px = -7;
constexpr int kFarEngoughToGoForth_px = 30;
constexpr int kAlgignedEnoughToGoForth_px = 7;
constexpr float kClampOffset = 50.f;

constexpr unsigned char kCameraBrightness = 0;

const mt::Vec2 kClawPos(54, 167);
const mt::Line kClawLine { .origin=kClawPos, .dir=mt::Vec2(86, 17)-kClawPos};
const mt::i32Box kClawBox{.bottomLeft=mt::i32Vec2(35, 165), .topRight=mt::i32Vec2(75, 210)};
const int kCamYView = 220;

} // !Tracking


} // !p28

#endif