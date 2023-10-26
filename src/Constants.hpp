
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
constexpr float kRobotWidth = 0.187;              // m
constexpr float kRobotWidth = 0.1865;              // m
#else
constexpr float kRobotWidth = 0.1850;              // m (distance between the motor wheels)
#endif
constexpr float kRobotWidth_2 = kRobotWidth/2;  // m
constexpr float kWheelRadius = 0.0383;           // m
constexpr int kTicksPerRotation = 3200;
constexpr float kCircumference = PI * kRobotWidth;
constexpr float kMaxVel = 1.9;                  // m / s
constexpr unsigned int kControlLoopDelay = 1; // ms
constexpr float kAccel = 0.8; // m/s^2
constexpr float kCatastrophicDecel = 2.0;
constexpr float kTurnAccel = 0.3;
constexpr unsigned int kDecelerationDelay = 50; // ms, time to stop the robot
constexpr float kMaxAngularVelocity = 5.0; // rad/s
constexpr float kForwardSpeed = 0.62;
constexpr float kMinVel = 0.04;
constexpr float kAccelDist = 0.2;
constexpr float kTurnOffset = 0.02;

constexpr float kInfinity = 100000;

using time_t = long unsigned int;

// Square of the precision of the path follower
constexpr float kPathFollower_epsilon2 = 0.004; // 2cm clearance

// Cup holder constants
constexpr int kCup_servoId = 0;
constexpr int kCup_openAngle = 15;
constexpr int kCup_closeAngle = 0;
constexpr int kCupRelease_pathIndex = 3; // After which arc of the maneuver should the cup be dropped?

// Cup knocker arm constants
constexpr int kArm_servoId = 1;
constexpr int kArm_openAngle = 90;
constexpr int kArm_closeAngle = 0;



enum class COLOR { RED, GREEN, BLUE, YELLOW, BLACK, WHITE };

} // !p28

#endif