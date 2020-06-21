#ifndef TASK_H
#define TASK_H

#include "moveit_msgs/CollisionObject.h"
#include "moveit/move_group_interface/move_group_interface.h"

#include "config.h"

class Task
{
public:
	Task(double x, double y);
	~Task();

	void setPlanningFrame(moveit::planning_interface::MoveGroupInterface& move_group);
	geometry_msgs::Pose getPose();
	moveit_msgs::CollisionObject getCollisionObject();
	std::string getID();
	
	//
	bool attemptToComplete();
	bool getStatus();
	void completeTask();

private:
	// Counts the number of instances of this class.
	// Will be used to give all jobs a unique string id
	static int count;
	std::string ID;

	bool completed;

	// The collision object is used to let MoveIt know it cannot
	// plan for a motion inside this geometry
	// This object will be fully constructed during instanciation
	moveit_msgs::CollisionObject collision_object;

	// This objects holds the position that will be pushed onto the
	// collision_object.primitive_poses vector
	geometry_msgs::Pose pose;
};

#endif