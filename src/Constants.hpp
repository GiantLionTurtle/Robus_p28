
#ifndef P28_CONSTANTS_HPP_
#define P28_CONSTANTS_HPP_

/*
    All physical constants of the robot in SI units where applicable
*/

#define ROBOT_A



enum kMotorId : uint8_t { kLeftMotor = 0, kRightMotor = 1 };
enum kBumperId : uint8_t  { kLeftBumper = 0, kRightBumper = 1, kFrontBumper = 2, kRearBumper = 3 };

constexpr double kRobotWidth = 0.189;              // m
constexpr double kRobotWidth_2 = kRobotWidth/2;  // m
constexpr double kWheelRadius = 0.0380;           // m
constexpr int kTicksPerRotation = 3200;
constexpr double kMaxVel = 1.9;                  // m / s
constexpr unsigned int kControlLoopDelay = 1; // ms
constexpr double kAccel = 1.1; // m/s^2
constexpr double kCatastrophicDecel = 1.8;
constexpr double kTurnAccel = 0.45;
constexpr unsigned int kDecelerationDelay = 100; // ms, time to stop the robot
constexpr double kTurnSpeed = 0.6;
constexpr double kForwardSpeed = 0.6;
constexpr double kDetectSpeed = 0.8;
constexpr double kMinSpeed = 0.08;
constexpr double kRealignSpeed = 0.01;
constexpr double kInertiaDist = 0.0;
constexpr double kAccelDist = 0.2;

using time_t = long unsigned int;



#endif