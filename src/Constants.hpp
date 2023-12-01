
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
constexpr float kRobotWidth = 0.19;              // m
constexpr float kRobotWidth_2 = kRobotWidth/2;  // m
constexpr float kWheelRadius = 0.0382;           // m
constexpr int kTicksPerRotation = 3200;
constexpr float kCircumference = PI * kRobotWidth;
constexpr float kLineSensorToCenter = 0.07; // distance forward of the line sensor (m)
constexpr float kRightWheelSquish = 0.00015; // The robot is heavier on the right! squish by 0.15mm

// Code constants and constructs
using time_t = long unsigned int;
constexpr float kInfinity = 100000;
constexpr unsigned int kMaxCheckPointForPath = 22;

//Color code for the controller
constexpr int kRed = 0;
constexpr int KGreen = 1;
constexpr int kBlue = 2;
constexpr int kYellow = 3;
constexpr int kAllColors = 4;


// Physical movement constraints of the robot
constexpr float kMaxVel = 0.4;                      // m / s
constexpr float kAccel = 0.2;                       // m/s^2
constexpr float kMaxAngularVelocity = 0.8;          // rad/s
constexpr float KMinVel = 0.03;                     // m/s
constexpr float kEndSegmentVel = 0.04;              // m/s
constexpr float kFollowLineBaseVel = 0.1;           // m/s
constexpr float kFollowLineCorrectCoeff = 0.003;    // m/s
constexpr float kFollowCamBaseVel = 0.08; 			// m/s
constexpr float kFollowCamVelCoef = 0.002; 			// m/s
constexpr float kTurnSpeed = 0.08; 					// m/s

// Control constants 
constexpr float kMotorHarwareStateMixFactor = 0.85; // Used for exponential moving average between 2 hardware states
constexpr unsigned int kControlLoopDelay = 1; // ms
constexpr float kPathFollower_distEpsilon2 = 0.0001; // square of precision (1cm)
constexpr float kPathFollower_headingEpsilon2 = 0.00005; // square of precision

// Arm-conveyor constants
constexpr int kArm_restAngle = 47; 
constexpr int kArm_downAngle = 30;
constexpr int kArm_upAngle = 180; 
constexpr int kClaw_openAngle = 82; 
constexpr int kClaw_closeAngle = 40; 
constexpr int kConveyor_stepsPerRevolution = 2038;
constexpr float kConveyor_speed = 5; //in RPM #define
constexpr int kConveyor_stepsUntilUp = 5051;//kConveyor_stepsPerRevolution*2.4; 

// Bin constants
constexpr int kBinTrap_openAngle = 0;
constexpr int kBinTrap_closedAngle = 85;
constexpr time_t kOpenTrapDelay_ms = 1200;

constexpr int kBinSelect_openAngle = 65;
constexpr int kBinSelect_closedAngle = 20;
constexpr int kBinCapacity = 20;


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
const mt::i32Vec2 kCamViewport{ 400, 220 };

constexpr short kObjectPermanence = 32;

} // !Tracking


} // !p28

#endif