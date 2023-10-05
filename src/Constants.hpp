
#ifndef P28_CONSTANTS_HPP_
#define P28_CONSTANTS_HPP_

/*
    All physical constants of the robot in SI units where applicable
*/

#define ROBOT_A



enum kMotorId : uint8_t { kLeftMotor = 0, kRightMotor = 1 };
enum kBumperId : uint8_t  { kLeftBumper = 0, kRightBumper = 1, kFrontBumper = 2, kRearBumper = 3 };

constexpr float kRobotWidth = 0.183;              // m
constexpr float kRobotWidth_2 = kRobotWidth/2;  // m
constexpr float kWheelRadius = 0.0381;           // m
constexpr int kTicksPerRotation = 3200;
constexpr float kMaxAngVelocity = 0.2;          // rad / s
constexpr float kMaxVel = 1.9;                  // m / s
constexpr unsigned int kControlLoopDelay = 1; // ms
constexpr float kAccel = 0.2; // m/s^2
constexpr unsigned int kDecelerationDelay = 0; // ms, time to stop the robot
constexpr float kTurnSpeed = 0.08;
constexpr float kForwardSpeed = 0.5;
constexpr float kDetectSpeed = 0.5;
constexpr float kMinSpeed = 0.05;

using time_t = long unsigned int;



#endif