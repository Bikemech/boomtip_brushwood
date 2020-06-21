#include "obstacle.h"

int Obstacle::count = 0;

Obstacle::Obstacle(double x, double y, double r)
{
	this->ID = "obstacle_" + std::to_string(this->count);
	this->count++;
	this->pose.position.x = x;
	this->pose.position.y = y;
	this->pose.position.z = HEIGHT/2 - 0.45;
	this->pose.orientation.w = 1.0;

	this->collision_object.id = this->ID;

	shape_msgs::SolidPrimitive primitive;
	primitive.type = primitive.CYLINDER;
	primitive.dimensions.resize(2);
	primitive.dimensions[0] = HEIGHT;
	primitive.dimensions[1] = r;

	// this->collision_object.operation = this->collision_object.ADD;
	this->collision_object.primitives.push_back(primitive);
	this->collision_object.primitive_poses.push_back(this->pose);
}

Obstacle::~Obstacle(){}


void Obstacle::setPlanningFrame(moveit::planning_interface::MoveGroupInterface& move_group)
{
	this->collision_object.header.frame_id = move_group.getPlanningFrame();
}


moveit_msgs::CollisionObject Obstacle::getCollisionObject()
{
	return this->collision_object;
}

geometry_msgs::Pose Obstacle::getPose()
{
	return this->pose;
}