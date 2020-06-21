#ifndef OBSTACLE_H
#define OBSTACLE_H

#include "moveit_msgs/CollisionObject.h"
#include "moveit/move_group_interface/move_group_interface.h"

#include "config.h"

class Obstacle
{
public:
	Obstacle(double x, double y, double r);
	~Obstacle();

	void setPlanningFrame(moveit::planning_interface::MoveGroupInterface& move_group);
	geometry_msgs::Pose getPose();
	moveit_msgs::CollisionObject getCollisionObject();
	std::string getID();

private:
	moveit_msgs::CollisionObject collision_object;
	geometry_msgs::Pose pose;
	static int count;
	std::string ID;

};

#endif