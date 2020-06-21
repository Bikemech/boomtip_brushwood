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
static const double CUT_HEIGHT = 0.3;

// Lenght required from the end effector for a cartesian approach
static const double EE_LEN = 0.2;

// The amount of the cartesian path that need to be completed
// for the task to pass as completed.
static const double CARTESIAN_LIMIT = 0.6;

// The name of the move group to be used
static const std::string PLANNING_GROUP = "manipulator";

// The name of the motion planner to be used
static const std::string MOTION_PLANNER = "RRTstarkConfigDefault";

// The maximum time allowed for the planner
static const double PLANNING_TIME = 10.0;

#endif