
#ifndef P28_CONSTANTS_HPP_
#define P28_CONSTANTS_HPP_

/*
    All physical constants of the robot in SI units where applicable
*/

namespace p28 {

enum kMotorId : uint8_t { kLeftMotor = 0, kRightMotor = 1 };
enum kBumperId : uint8_t  { kLeftBumper = 0, kRightBumper = 1, kFrontBumper = 2, kRearBumper = 3 };

constexpr float kRobotWidth = 0.2;              // m
constexpr float kRobotWidth_2 = kRobotWidth/2;  // m
constexpr float kWheelRadius = 0.025;           // m
constexpr int kTicksPerRotation = 100;
constexpr float kMaxAngVelocity = 0.2;          // rad / s
constexpr float kMaxVel = 0.4;                  // m / s

using time_t = long unsigned int;

} // !p28

#endif