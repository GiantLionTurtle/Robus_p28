
#ifndef P28_COMPILEFLAGS_HPP_
#define P28_COMPILEFLAGS_HPP_

// Chose target robot
#define AQUAMAN
#ifndef AQUAMAN
#define BATAMAN
#endif

// Uncomment to enable additional prints
#define DEBUG_MODE

// Uncomment to enable the test objective
#define ENABLE_TEST_OBJECTIVE

// Uncomment to force the drivebase to align itself with a 
// wall no matter the context
 #define FORCE_WALL_ALIGN

// Uncomment to allow the robot to adjust the drivebase position
// with game zones (color sensor)
#define ENABLE_ZONESWITCH_DRIVEBASE_ADJUSTMENTS

// Uncomment to use the left bumper to manualy switch the internal zone state
// FOR DEBUG & TESTS ONLY
// #define LEFT_BUMPER_FOR_ZONE_INCREMENT

// Uncomment to enable race mode vs qualification mode
// #define RACE_MODE

#endif