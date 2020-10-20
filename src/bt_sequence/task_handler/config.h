#ifndef CONFIG_H
#define CONFIG_H

#include <string>

// position of system origo
static const double ORIGO_X = 0.0;
static const double ORIGO_Y = 0.0;
static const double ORIGO_Z = 0.0;

// Radius of the brush wood
static const double RADIUS = 0.04;

// Height of the brush wood
static const double HEIGHT = 2.0;

// The predetermined cutting height
static const double CUT_HEIGHT = 0.8;

// cartesian approach travel distance required plus a tolerance.
static const double EE_LEN = 0.2;

// Cartesian travel distance tolerance
static const double CARTESIAN_LIMIT = 0.6;

// The name of the move group to be used
static const std::string PLANNING_GROUP = "manipulator";
// static const std::string PLANNING_GROUP = "panda_arm";

// The name of the motion planner to be used
static const std::string MOTION_PLANNER = "RRTstarkConfigDefault";

// The maximum time allowed for the planner
static const double PLANNING_TIME = 2.0;

#endif