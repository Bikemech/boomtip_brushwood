#ifndef OBSTACLE_H
#define OBSTACLE_H

#include "moveit_msgs/CollisionObject.h"
#include "moveit/move_group_interface/move_group_interface.h"

#include "config.h"


/* This class represents an obstacle that is only meant
to be avoided by the arm.
At this point the only available geometry is a cylinder.
*/

class Obstacle
{
public:
	// Position (x, y) relative to the planning_scenes origo \
	and a radius r
	Obstacle(double x, double y, double r);
	~Obstacle();

	void setPlanningFrame(moveit::planning_interface::MoveGroupInterface& move_group);
	geometry_msgs::Pose getPose();
	moveit_msgs::CollisionObject getCollisionObject();
	std::string getID();

private:
	moveit_msgs::CollisionObject collision_object;
	geometry_msgs::Pose pose;

	// Global count of obstacle instances.
	static int count;

	// Local name derived from global count at instanciation.
	std::string ID;
};

#endif