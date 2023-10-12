
#ifndef P28_CONSTANTS_HPP_
#define P28_CONSTANTS_HPP_

/*
    All physical constants of the robot in SI units where applicable
*/

#define AQUAMAN
#ifndef AQUAMAN
#define BATAMAN
#endif


enum kMotorId : uint8_t { kLeftMotor = 0, kRightMotor = 1 };
enum kBumperId : uint8_t  { kLeftBumper = 0, kRightBumper = 1, kFrontBumper = 2, kRearBumper = 3 };

#ifdef AQUAMAN
constexpr double kRobotWidth = 0.1875;              // m
#else
constexpr double kRobotWidth = 0.1850;              // m
#endif
constexpr double kRobotWidth_2 = kRobotWidth/2;  // m
constexpr double kWheelRadius = 0.0383;           // m
constexpr int kTicksPerRotation = 3200;
constexpr double kCircumference = PI * kRobotWidth;
constexpr double kMaxVel = 1.9;                  // m / s
constexpr unsigned int kControlLoopDelay = 1; // ms
constexpr double kAccel = 0.8; // m/s^2
constexpr double kCatastrophicDecel = 2.0;
constexpr double kTurnAccel = 0.3;
constexpr unsigned int kDecelerationDelay = 50; // ms, time to stop the robot
constexpr double kTurnSpeed = 0.3;
constexpr double kForwardSpeed = 0.62;
constexpr double kDetectSpeed = kForwardSpeed;
constexpr double kMinSpeed = 0.04;
constexpr double kRealignSpeed = 0.1;
constexpr double kInertiaForwardDist = 0.035;
constexpr double kInertiaDist = 0.035;
constexpr double kAccelDist = 0.2;

using time_t = long unsigned int;



#endif