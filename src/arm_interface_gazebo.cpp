#include "youbot_object_grasp/arm_interface_gazebo.h"
#include <std_msgs/Float64.h>
#include <iostream>

////////////////////////////////////////////////////////////////////////////////
//  Construction / Destruction
////////////////////////////////////////////////////////////////////////////////

cArmInterfaceGazebo::cArmInterfaceGazebo(const tf::Transform& g_arm0_to_base_link, ros::NodeHandle& nh)
	: cArmInterface(g_arm0_to_base_link)
{
	mArmJoint1Pub = nh.advertise<std_msgs::Float64>( "/youbot/arm_joint_1_position_controller/command", 1 );
	mArmJoint2Pub = nh.advertise<std_msgs::Float64>( "/youbot/arm_joint_2_position_controller/command", 1 );
	mArmJoint3Pub = nh.advertise<std_msgs::Float64>( "/youbot/arm_joint_3_position_controller/command", 1 );
	mArmJoint4Pub = nh.advertise<std_msgs::Float64>( "/youbot/arm_joint_4_position_controller/command", 1 );
	mArmJoint5Pub = nh.advertise<std_msgs::Float64>( "/youbot/arm_joint_5_position_controller/command", 1 );

	mLeftGripperPub = nh.advertise<std_msgs::Float64>( "/youbot/gripper_finger_joint_l_position_controller/command", 1 );
	mRightGripperPub = nh.advertise<std_msgs::Float64>( "/youbot/gripper_finger_joint_r_position_controller/command", 1 );
}

////////////////////////////////////////////////////////////////////////////////
//  Interface Functions
////////////////////////////////////////////////////////////////////////////////

void cArmInterfaceGazebo::PublishJointValues(const std::vector<double>& values)
{
	std::cerr << "Publishing joint angles:" << std::endl;
	std_msgs::Float64 v0;
	std::cerr << "\tJoint 1:  " << values[0] << std::endl;
	v0.data = values[0];
	mArmJoint1Pub.publish(v0);

	std_msgs::Float64 v1;
	std::cerr << "\tJoint 2:  " << values[1] << std::endl;
	v1.data = values[1];
	mArmJoint2Pub.publish(v1);

	std_msgs::Float64 v2;
	std::cerr << "\tJoint 3:  " << values[2] << std::endl;
	v2.data = values[2];
	mArmJoint3Pub.publish(v2);

	std_msgs::Float64 v3;
	std::cerr << "\tJoint 4:  " << values[3] << std::endl;
	v3.data = values[3];
	mArmJoint4Pub.publish(v3);

	std_msgs::Float64 v4;
	std::cerr << "\tJoint 5:  " << values[4] << std::endl;
	v4.data = values[4];
	mArmJoint5Pub.publish(v4);
	std::cerr << std::endl;
}

////////////////////////////////////////////////////////////////////////////////
//  Helper Functions
////////////////////////////////////////////////////////////////////////////////

void cArmInterfaceGazebo::PublishGripperValues(double width)
{
	std::cerr << "Publishing joint angles:" << std::endl;
	std_msgs::Float64 v0;
	std::cerr << "\tLeft Gripper Finger:  " << width << std::endl;
	v0.data = width;
	mLeftGripperPub.publish(v0);

	std_msgs::Float64 v1;
	std::cerr << "\tRight Gripper Finger:  " << width << std::endl;
	v1.data = width;
	mRightGripperPub.publish(v1);
	std::cerr << std::endl;
}


