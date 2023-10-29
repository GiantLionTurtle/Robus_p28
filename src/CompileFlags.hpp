
#ifndef P28_COMPILEFLAGS_HPP_
#define P28_COMPILEFLAGS_HPP_

// Chose target robot
//#define AQUAMAN
#ifndef AQUAMAN
#define BATAMAN
#endif

// Uncomment to enable additional prints
#define DEBUG_MODE

// Uncomment to force the drivebase to align itself with a 
// wall no matter the context
// #define FORCE_WALL_ALIGN

// Uncomment to allow the robot to adjust the drivebase position
// with game zones (color sensor)
#define ENABLE_ZONESWITCH_DRIVEBASE_ADJUSTMENTS

// Uncomment to enable race mode vs qualification mode
// #define RACE_MODE

#endif