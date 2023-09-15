
#ifndef P28_CONSTANTS_HPP_
#define P28_CONSTANTS_HPP_

namespace p28 {

enum kMotorId : uint8_t { kLeftMotor = 0, kRightMotor = 1 };
enum kBumperId : uint8_t  { kLeftBumper = 0, kRightBumper = 1, kFrontBumper = 2, kRearBumper = 3 };

// In meters
constexpr float kRobotWidth = 0.2;              // m
constexpr float kRobotWidth_2 = kRobotWidth/2;  // m
constexpr float kWheelRadius = 0.025;           // m
constexpr int kTicksPerRotation = 100;
constexpr float kPi = 3.1415;
constexpr float kMaxAngVelocity = 0.2;          // rad / s
constexpr float kMaxVel = 0.4;                  // m / s

using time_t = long long unsigned int;

} // !p28

#endif