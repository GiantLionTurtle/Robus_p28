
#ifndef P28_CONSTANTS_HPP_
#define P28_CONSTANTS_HPP_

/*
    All physical constants of the robot in SI units where applicable
*/

#define ROBOT_A

namespace p28 {

enum kMotorId : uint8_t { kLeftMotor = 0, kRightMotor = 1 };
enum kBumperId : uint8_t  { kLeftBumper = 0, kRightBumper = 1, kFrontBumper = 2, kRearBumper = 3 };

constexpr float kRobotWidth = 0.185;              // m
constexpr float kRobotWidth_2 = kRobotWidth/2;  // m
constexpr float kWheelRadius = 0.0381;           // m
constexpr int kTicksPerRotation = 3200;
constexpr float kMaxAngVelocity = 0.2;          // rad / s
constexpr float kMaxVel = 1.9;                  // m / s
constexpr unsigned int kControlLoopDelay = 1; // ms
constexpr float kAccell = 0.1; // m/s^2
constexpr float kdetectionDistance = 0.1; // m
constexpr unsigned int kDecelerationDelay = 100; // ms, time to stop the robot
constexpr float kTurnSpeed = 0.2;
constexpr float kForwardSpeed = 0.2;
constexpr float kDetectSpeed = 0.2;

using time_t = long unsigned int;

} // !p28

#endif