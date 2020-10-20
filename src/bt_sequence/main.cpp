#include "moveit/move_group_interface/move_group_interface.h"
#include "moveit/planning_scene_interface/planning_scene_interface.h"

#include "task_handler/task.h"
#include "task_handler/task_manager.h"

// The configuration for the tasks and the motion planner
#include "task_handler/config.h"

int main(int argc, char** argv)
{
	ros::init(argc, argv, "maintest");
	/* Using ros::Spin() will cause the method
	 * moveit::planning_interface::MoveGroupInterface()
	 * to block indefinetly.
	 * More than one thread will make programming acrobatics beyond my expertise neccessary.
	 */

	ros::AsyncSpinner spinner(1);
	spinner.start();

	// PLANNING_GROUP and MOTION_PLANNER are constants from config.h
	moveit::planning_interface::PlanningSceneInterface scene;
	moveit::planning_interface::MoveGroupInterface group(PLANNING_GROUP);

	group.setPlannerId(MOTION_PLANNER);
	group.setPlanningTime(PLANNING_TIME);

	// // // // // // // // // // // // // // // //
	// // // // // // // // // // // // // // // //


	/*
	 * Joint state works on UR_10 and UR_5
	 * Will cause the program to throw an exception on other models.
	 * Comment out group.setJointValueTarget and group.move
	 * here and at the EOF to avoid crash when using other models.
	 */

	std::map<std::string, double> joint_target_state{
		{"shoulder_pan_joint", 0.0},
		{"shoulder_lift_joint", -M_PI*2/3},
		{"elbow_joint", M_PI*3/4},
		{"wrist_1_joint", 0.0},
		{"wrist_2_joint", M_PI/2},
		{"wrist_3_joint", 0.0}
	};

	group.setJointValueTarget(joint_target_state);
	group.move();

	/* // // // // // // // // // // // // // // // //
	//			Add collision objects.				//
	// // // // // // // // // // // // // // // // */

	std::vector<moveit_msgs::CollisionObject> collision_objects;

	moveit_msgs::CollisionObject ground;
	ground.header.frame_id = group.getPlanningFrame();
	ground.id = "Ground_primitive";

	shape_msgs::SolidPrimitive ground_primitive;
	ground_primitive.type = ground_primitive.CYLINDER;
	ground_primitive.dimensions.resize(2);

	ground_primitive.dimensions[0] = 0.1;
	ground_primitive.dimensions[1] = 2.0;

	geometry_msgs::Pose ground_pose;

	ground_pose.orientation.w = 1;
	ground_pose.position.x = 0.0;
	ground_pose.position.y = 0.0;
	ground_pose.position.z = -0.5;

	ground.primitives.push_back(ground_primitive);
	ground.primitive_poses.push_back(ground_pose);

	// // // // // // // // // // // // // // // //

	moveit_msgs::CollisionObject vehicle;
	vehicle.header.frame_id = group.getPlanningFrame();
	vehicle.id = "Vehicle_primitive";

	shape_msgs::SolidPrimitive vehicle_primitive;
	vehicle_primitive.type = vehicle_primitive.BOX;
	vehicle_primitive.dimensions.resize(3);

	vehicle_primitive.dimensions[0] = 1;
	vehicle_primitive.dimensions[1] = 1;
	vehicle_primitive.dimensions[2] = 1;

	geometry_msgs::Pose vehicle_pose;

	vehicle_pose.orientation.w = 1.0;

	vehicle_pose.position.x = -1.0;
	vehicle_pose.position.y = 0.0;
	vehicle_pose.position.z = 0.0;

	vehicle.primitives.push_back(vehicle_primitive);
	vehicle.primitive_poses.push_back(vehicle_pose);

	// // // // // // // // // // // // // // // //

	moveit_msgs::CollisionObject mount;
	mount.header.frame_id = group.getPlanningFrame();
	mount.id = "arm_mount";

	shape_msgs::SolidPrimitive mount_primitive;
	mount_primitive.type = mount_primitive.CYLINDER;
	mount_primitive.dimensions.resize(2);

	mount_primitive.dimensions[0] = 0.4;
	mount_primitive.dimensions[1] = 0.1;

	geometry_msgs::Pose mount_pose;

	mount_pose.orientation.w = 1.0;
	mount_pose.position.x = 0.0;
	mount_pose.position.y = 0.0;
	mount_pose.position.z = -0.25;

	mount.primitives.push_back(mount_primitive);
	mount.primitive_poses.push_back(mount_pose);


	// // // // // // // // // // // // // // // //

	collision_objects.push_back(ground);
	collision_objects.push_back(vehicle);
	collision_objects.push_back(mount);

	scene.addCollisionObjects(collision_objects);

	// // // // // // // // // // // // // // // //
	// // // // // // // // // // // // // // // //


	TaskManager tm(&group, &scene);

	tm.pushTask(1.15, -0.0);
	tm.pushTask(0.8, -0.05);
	tm.pushTask(0.9, 0.2);
	tm.pushTask(0.0, -1.0);
	tm.pushTask(1.15, 0.5);

	tm.pushObstacle(0.7, 1.0, 0.25);
	tm.pushObstacle(0.8, 0.5, 0.1);
	tm.pushObstacle(0.8, -0.5, 0.2);

	
	tm.spawnTasksToScene();
	tm.spawnObstaclesToScene();

	// // // // // // // // // // // // // // // //
	// // // // // // // // // // // // // // // //

	if (argc > 1)
	{
		std::cout << "Quit after setup" << std::endl;
		return 0;
	}


	tm.autoCycle();

	std::cout << std::endl;

	tm.taskListToConsole();

	group.setJointValueTarget(joint_target_state);
	group.move();

	return 0;
}