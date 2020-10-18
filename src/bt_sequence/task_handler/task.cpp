#include "task.h"

//	initializes the counter to 0. \
	Note it is a static variable.
int Task::count = 0;

Task::Task(double x, double y)
{
	// Couple class instance with a unique name string.
	this->ID = "task_" + std::to_string(this->count);
	this->count++;
	this->pose.position.x = x;
	this->pose.position.y = y;
	this->pose.position.z = HEIGHT/2 - 0.45;
	this->pose.orientation.w = 1.0;

    // Couple moveit object with the same name string.
	this->collision_object.id = this->ID;

	shape_msgs::SolidPrimitive primitive;
	primitive.type = primitive.CYLINDER;
	primitive.dimensions.resize(2);
	primitive.dimensions[0] = HEIGHT;
	primitive.dimensions[1] = RADIUS;

	// this->collision_object.operation = this->collision_object.ADD;
	this->collision_object.primitives.push_back(primitive);
	this->collision_object.primitive_poses.push_back(this->pose);

	this->completed = false;
}

Task::~Task(){}


void Task::setPlanningFrame(moveit::planning_interface::MoveGroupInterface& move_group)
{
	this->collision_object.header.frame_id = move_group.getPlanningFrame();
}


moveit_msgs::CollisionObject Task::getCollisionObject()
{
	return this->collision_object;
}

geometry_msgs::Pose Task::getPose()
{
	return this->pose;
}

void Task::completeTask()
{
	// Move the geometry down. This should  be\
	replaced with a parameter
	this->pose.position.z -= 1.4;

	this->collision_object.primitive_poses.clear();
	this->collision_object.primitive_poses.push_back(this->pose);
	this->completed = true;
	std::cout << "Finished task " << this->ID << std::endl;
}

bool Task::getStatus()
{
	return this->completed;
}

std::string Task::getID()
{
	return this->ID;
}