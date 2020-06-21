#ifndef TASK_MANAGER_H
#define TASK_MANAGER_H

#include <cmath>

#include "moveit_msgs/CollisionObject.h"
#include "moveit/move_group_interface/move_group_interface.h"
#include "moveit/planning_scene_interface/planning_scene_interface.h"

#include "task.h"
#include "obstacle.h"
#include "config.h"

class TaskManager
{
public:
	/**
	* Constructs with a move_group to make motionplanning messages more accesible to the class.
	* Constructs with a planning_scene to enable the class to manipulate the environment.
	* @param the move group for the manipulator
	* @param the interface to MoveIt's planning scene.
	*/
	TaskManager(
		moveit::planning_interface::MoveGroupInterface* move_group,
		moveit::planning_interface::PlanningSceneInterface* planning_scene
		);

	~TaskManager();

	/**
	Completed tasks will remain in the queue.
	* Will return a reference to the task_que;
	* @return a container of all tasks pushed onto the queue
	*/
	std::vector<Task>& getQueue();

	/**
	* pushes a Pre-instanciated task onto the to queue;
	* Sets the headerframe of the task to this instances headerframe
	* @param a Task object
	*/
	void pushTask(Task& task);

	/**
	* creates a Task object and pushes it onto the queue
	* Sets the headerframe of the task to this instances headerframe
	* @param the tasks x-coordinate
	* @param the tasks y-coordinate
	*/
	void pushTask(double x, double y);

	/**
	* creates a collisionobject to add to the scene
	* @param x coordinate
	* @param y coordinate
	* @param tree radius
	*/
	void pushObstacle(double x, double y, double r);

	/**
	*
	* @param x coordinate
	* @param y coordinate
	* @param stone x-span
	* @param stone y-span
	* @oaran stone height
	*/
	void pushStone(double x, double y, double w, double b, double h);

	/**
	* Attempts to plan and execute the task the pointer is currently pointing to
	* Then moves the pointer up one step regardless of success.
	* sets the any_completion bool to true if success.
	* Will use the private functions attempt_motion_plan
	* and attempt_approach.
	* @return true if there are more jobs in the que and false if its at the end.
	*/
	bool cycle();

	/**
	* Will run the cycle method, reset at the end of the queue
	* until all the tasks are completed or no more tasks can be completed.
	*/
	void autoCycle();

	/**
	* Manually skip ahead in the que
	*/
	void skip();

	/**
	* sets the index to the queue's first position and sets
	* the any_completion bool to false
	*/
	void resetCycle();

	/**
	* Will iterate through the task_queue and call
	* applyCollisionObject() for each item in queue with the current items
	* getCollisionObject() method.
	* This will make the collision geometry associated with each task available to
	* the planning frame.
	*/
	void spawnTasksToScene();

	void spawnObstaclesToScene();

	void taskListToConsole();

private:
	// This is the queue for all tasks.
	std::vector<Task> task_que;

	// This is a list of all obstacles
	std::vector<Obstacle> obstacle_que;

	// To track how many cycles it took to finish
	unsigned int cycle_count;

	// the index of the task_queue pointer
	std::vector<Task>::iterator index;

	// Sets to false at constructions and cycle reset.
	bool any_completions;

	moveit::planning_interface::MoveGroupInterface* move_group;
	moveit::planning_interface::PlanningSceneInterface* planning_scene;

	// system origo
	geometry_msgs::Pose origo;

	// keep the current pose stored here
	geometry_msgs::Pose current_pose;
	// target pose here
	geometry_msgs::Pose target_pose;

	// A plan data structure may contain preplanned trajectorys.
	// this is crucial for planning cartesian paths.
	moveit::planning_interface::MoveGroupInterface::Plan motion_plan;

private:
	/**
	* Attempts to compute a cartesian path to approach the target.
	* if false is returned cycle forward else to task should be called with
	* the completeTask() method and reapplyed to the planning_scene.
	* MoveIt error codes evaluates as bool.
	* @return true if successfull else false.
	*/
	moveit::planning_interface::MoveItErrorCode attempt_approach();

	/**
	* creates a pose with orientation and position
	* relative to the target task and the decided margin required
	* for the end effector to approach.
	* @return a pose to set as target for the move group
	*/
	virtual geometry_msgs::Pose get_target_position();

	/**
	* computes a cartesian motion for the boom tips approach
	*
	* @return a double such that 0 >= value >= 1 representing
	* the percentage of path successfully planned.
	*/
	double cartesian_grasp();

	/**
	* Computes the norm of the vector between origo and the target as projected
	* onto the xy-plane;
	* @return the euclidean distance between target and origo.
	*/
	double compute_norm();

	/**
	* Checks if there are any unfinished tasks in the list.
	* @return
	*/
	bool unfinished_tasks();
};


#endif