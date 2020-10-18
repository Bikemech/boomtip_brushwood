#ifndef TASK_H
#define TASK_H

#include "moveit_msgs/CollisionObject.h"
#include "moveit/move_group_interface/move_group_interface.h"

#include "config.h"


/* This class represents a task with associated geometry 
 * The properties of the geometry has some parameters in 
 * in the config.h file and some propeties are hard coded.
 */

class Task
{
public:
	Task(double x, double y);
	~Task();

	void setPlanningFrame(moveit::planning_interface::MoveGroupInterface& move_group);
	geometry_msgs::Pose getPose();
	moveit_msgs::CollisionObject getCollisionObject();
	std::string getID();

	// Returns true if task is completed.
	bool getStatus();

	// Sets private bool completed to True
	void completeTask();

private:
	// Counts the number of instances of this class. \
	Will be used to give all jobs a unique string id
	static int count;
	std::string ID;

	// Set to true if the task is completed \
	and moves the geometry of the task down to clear \
	the path physically in the planning scene.
	bool completed;

	// The collision object is used to let MoveIt know it cannot \
	plan for a motion inside this geometry \
	This object will be fully constructed during instanciation
	moveit_msgs::CollisionObject collision_object;

	// This objects holds the position that will be pushed onto the \
	collision_object.primitive_poses vector
	geometry_msgs::Pose pose;
};

#endif