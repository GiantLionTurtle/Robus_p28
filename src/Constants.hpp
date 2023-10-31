
#ifndef P28_CONSTANTS_HPP_
#define P28_CONSTANTS_HPP_

#include "CompileFlags.hpp"

/*
    All physical constants of the robot in SI units where applicable
*/

namespace p28 {

enum kMotorId : uint8_t { kLeftMotor = 0, kRightMotor = 1 };
enum kBumperId : uint8_t  { kLeftBumper = 0, kRightBumper = 1, kFrontBumper = 2, kRearBumper = 3 };

#ifdef AQUAMAN
constexpr float kRobotWidth = 0.19;              // m
#else
constexpr float kRobotWidth = 0.1850;              // m (distance between the motor wheels)
#endif
constexpr float kRobotWidth_2 = kRobotWidth/2;  // m
constexpr float kWheelRadius = 0.0383;           // m
constexpr int kTicksPerRotation = 3200;
constexpr float kCircumference = PI * kRobotWidth;
constexpr float kMaxVel = 0.8;                  // m / s
constexpr unsigned int kControlLoopDelay = 10; // ms
constexpr float kAccel = 0.4; // m/s^2
constexpr float kCatastrophicDecel = 1.0;
constexpr float kTurnAccel = 0.3;
constexpr unsigned int kDecelerationDelay = 50; // ms, time to stop the robot
constexpr float kMaxAngularVelocity = 0.8; // rad/s
constexpr float kForwardSpeed = 0.62;
constexpr float kMinSpeed = 0.03;
constexpr float kAccelDist = 0.2;
constexpr float kTurnOffset = 0.02;
constexpr int kIRSensor_apartDist = 84;
constexpr float kInfinity = 100000;

constexpr float kMotorHarwareStateMixFactor = 0.85;

using time_t = long unsigned int;

// Square of the precision of the path follower
// We want to land within a radius of 1cm
constexpr float kPathFollower_distEpsilon2 = 0.0001;
constexpr float kPathFollower_headingEpsilon2 = 0.00005;
constexpr unsigned int kMaxCheckPointForPath = 20;


// Cup holder constants
constexpr int kCup_servoId = 0;
constexpr int kCup_openAngle = 15;
constexpr int kCup_closeAngle = 180;
constexpr int kCupRelease_pathIndex = 3; // After which arc of the maneuver should the cup be dropped?

// Cup knocker arm constants
constexpr int kArm_servoId = 1;
constexpr int kArm_openAngle = 20;
constexpr int kArm_closeAngle = 150;
constexpr int kCupDetectDist = 200; // mm
constexpr int kKnockCupDelay = 1500; // ms

// position of the center of the arc of the turns
// one full turn mission constants
constexpr unsigned int kOneCWTurn_startTime = 20000;


enum class COLOR { RED, GREEN, BLUE, YELLOW, BLACK, WHITE };

} // !p28

#endif