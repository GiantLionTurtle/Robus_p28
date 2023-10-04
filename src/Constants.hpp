
#ifndef P28_CONSTANTS_HPP_
#define P28_CONSTANTS_HPP_

/*
    All physical constants of the robot in SI units where applicable
*/

#define ROBOT_A



enum kMotorId : uint8_t { kLeftMotor = 0, kRightMotor = 1 };
enum kBumperId : uint8_t  { kLeftBumper = 0, kRightBumper = 1, kFrontBumper = 2, kRearBumper = 3 };

const float kRobotWidth = 0.185;              // m
const float kRobotWidth_2 = kRobotWidth/2;  // m
const float kWheelRadius = 0.0381;           // m
const int kTicksPerRotation = 3200;
const float kMaxAngVelocity = 0.2;          // rad / s
const float kMaxVel = 1.9;                  // m / s
const unsigned int kControlLoopDelay = 1; // ms
const float kAccell = 0.1; // m/s^2
const float kdetectionDistance = 0.1; // m
const unsigned int kDecelerationDelay = 100; // ms, time to stop the robot
const float kTurnSpeed = 0.2;
const float kForwardSpeed = 0.2;
const float kDetectSpeed = 0.2;

using time_t = long unsigned int;



#endif