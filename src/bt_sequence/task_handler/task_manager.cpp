#include "task_manager.h"

TaskManager::TaskManager(
		moveit::planning_interface::MoveGroupInterface* move_group,
		moveit::planning_interface::PlanningSceneInterface* planning_scene
		)
{
	this->move_group = move_group;
	this->planning_scene = planning_scene;
	this->index = this->task_que.begin();
	this->any_completions = false;
	this->cycle_count = 0;
	
	this->origo.position.x = ORIGO_X;
	this->origo.position.y = ORIGO_Y;
	this->origo.position.z = ORIGO_Z;
	this->origo.orientation.w = 1.0;
}

TaskManager::~TaskManager(){}

void TaskManager::pushTask(Task& task)
{
	// set the proper planning frame here
	task.setPlanningFrame(*(this->move_group));
	this->task_que.push_back(task);
	this->resetCycle();
}

void TaskManager::pushTask(double x, double y)
{
	this->task_que.push_back(Task(x, y));

	// set the proper planning frame here
	(*(this->task_que.end() - 1)).setPlanningFrame(*(this->move_group));
	this->resetCycle();
}

void TaskManager::pushObstacle(double x, double y, double r)
{
	this->obstacle_que.push_back(Obstacle(x, y, r));

	// set the proper planning frame here
	(*(this->obstacle_que.end() - 1)).setPlanningFrame(*(this->move_group));
	this->resetCycle();
}

void TaskManager::spawnTasksToScene()
{
	for (auto i = this->task_que.begin(); i < this->task_que.end(); ++i)
	{
		this->planning_scene->applyCollisionObject((*i).getCollisionObject());
	}
}

void TaskManager::spawnObstaclesToScene()
{
	for (auto i = this->obstacle_que.begin(); i < this->obstacle_que.end(); ++i)
	{
		this->planning_scene->applyCollisionObject((*i).getCollisionObject());
	}
}

bool TaskManager::cycle()
{
	// This method is a bit messy because there are a lot of things going on here. \
	This should be refactored to multiple methods to make it easier to follow.

	// Return false if the queue is finished.
	if (this->index == this->task_que.end()) return false;


	// Check the status of the task next in queue.
	if ((*(this->index)).getStatus())
	{
		// skip if the task is completed already.
		std::cout << "skipped" << std::endl;
		this->skip();
		return true;
	}


	moveit::planning_interface::MoveItErrorCode execution_status;
	double cartesian_completion = 0;

	geometry_msgs::Pose target_pose = this->get_target_position();

	// Set pose target to move group
	if (this->move_group->setPoseTarget(target_pose))
		execution_status = this->move_group->move();

	// if motion was succesfull proceed to initiate cartesian motion.
	if (execution_status)
	{
		this->current_pose = this->target_pose;

		// compute cartesian motion plan
		cartesian_completion = this->cartesian_grasp();
		std::cout << cartesian_completion << std::endl;

	}

	// Check if enough of the path is possible to complete.
	if (cartesian_completion >= CARTESIAN_LIMIT)
		execution_status = this->attempt_approach();

	// if both steps are completed the task should also be completed.
	if (execution_status && cartesian_completion >= CARTESIAN_LIMIT)
	{
		// update task status and task geometry.
		(*(this->index)).completeTask();

		// update the planning scene.
		this->spawnTasksToScene();

		// Flag for a second iteration trough the queue.
		this->any_completions = true;
	}

	if (!(*(this->index)).getStatus())
	{
		std::cout << "Could not complete " << (*(this->index)).getID() << std::endl;
	}

	this->index++;
	return true;
}


void TaskManager::autoCycle()
{
	this->resetCycle();

	std::cout << "begin auto cycle" << std::endl;

	// cycle until completion or iteration without mutation occurs.
	while (this->cycle());

	if (this->any_completions && this->unfinished_tasks())
	{
		this->autoCycle();
	}
}


void TaskManager::skip()
{
	this->index++;
}

void TaskManager::resetCycle()
{
	this->any_completions = false;
	this->index = this->task_que.begin();
}

geometry_msgs::Pose TaskManager::get_target_position()
{
	geometry_msgs::Pose task_pose = (*(this->index)).getPose();

	double x = task_pose.position.x;
	double y = task_pose.position.y;

	double norm = sqrt(pow(x, 2) + pow(y, 2));

	// Use this to set the orientation of the end effector
	tf2::Quaternion q;
	if (x >= 0) q.setRPY(0, 0, atan(y/x));
	else q.setRPY(0, 0, atan(y/x) + M_PI);
	


	//	The following lines computes the position the end effector \
		should stop at before the cartesian approach.
	(this->target_pose.position).x = (x / norm) * (norm - EE_LEN);
	(this->target_pose.position).y = (y / norm) * (norm - EE_LEN);
	(this->target_pose).position.z = CUT_HEIGHT;

	(this->target_pose).orientation.x = q.x();
	(this->target_pose).orientation.y = q.y();
	(this->target_pose).orientation.z = q.z();
	(this->target_pose).orientation.w = q.w();


	return target_pose;
}

double TaskManager::cartesian_grasp()
{
	std::vector<geometry_msgs::Pose> way_points;
	way_points.push_back(this->current_pose);

	// this->target_pose = this->current_pose;

	geometry_msgs::Pose task_pose = (*(this->index)).getPose();

	this->target_pose.position.x = task_pose.position.x;
	this->target_pose.position.y = task_pose.position.y;
	this->target_pose.position.z = CUT_HEIGHT;
	way_points.push_back(this->target_pose);

	// store computed cartesian path at the local motion_plan object.
	return this->move_group->computeCartesianPath(way_points, 0.01, 0.0, this->motion_plan.trajectory_);
}

moveit::planning_interface::MoveItErrorCode TaskManager::attempt_approach()
{
	return this->move_group->execute(this->motion_plan);
}

double TaskManager::compute_norm()
{
	return sqrt(pow(this->target_pose.position.x, 2) +
				pow(this->target_pose.position.y, 2));
}

void TaskManager::taskListToConsole()
{
	for (auto i = this->task_que.begin(); i < this->task_que.end(); ++i)
	{
		std::cout << (*i).getID() << "\t" << ( ((*i).getStatus()) ? "Completed" : "Failed") << std::endl;
	}
}

bool TaskManager::unfinished_tasks()
{
	for (auto i = this->task_que.begin(); i < this->task_que.end(); ++i)
	{
		return !((*i).getStatus());
	}
	return false;
}