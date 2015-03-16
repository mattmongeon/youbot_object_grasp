#include "youbot_object_grasp/arm_interface_youbot.h"
#include <brics_actuator/JointPositions.h>
#include <brics_actuator/JointValue.h>
#include <sstream>

////////////////////////////////////////////////////////////////////////////////
//  Construction / Destruction
////////////////////////////////////////////////////////////////////////////////

cArmInterfaceYoubot::cArmInterfaceYoubot(ros::NodeHandle& nh)
	: cArmInterface()
{
	mArmJointsPub = nh.advertise<brics_actuator::JointPositions>("arm_1/arm_controller/position_command", 1);
	mGripperPub = nh.advertise<brics_actuator::JointPositions>("arm_1/gripper_controller/position_command", 1);
}

////////////////////////////////////////////////////////////////////////////////
//  Interface Functions
////////////////////////////////////////////////////////////////////////////////

void cArmInterfaceYoubot::PublishJointValues(const std::vector<double>& values)
{
	brics_actuator::JointPositions pos;
	for( std::size_t i = 0; i < values.size(); ++i )
	{
		brics_actuator::JointValue val;
		
		std::ostringstream jointName;
		jointName << "arm_joint_" << i;
		val.joint_uri = jointName.str();
		val.unit = "rad";
		val.value = values[i];

		pos.positions.push_back(val);
	}

	mArmJointsPub.publish(pos);
}

////////////////////////////////////////////////////////////////////////////////

void cArmInterfaceYoubot::PublishGripperValues(double width)
{
	brics_actuator::JointPositions pos;

	brics_actuator::JointValue finger;
	finger.joint_uri = "gripper_finger_joint_l";
	finger.unit = "m";
	finger.value = width;
	pos.positions.push_back(finger);

	finger.joint_uri = "gripper_finger_joint_r";
	finger.unit = "m";
	finger.value = width;
	pos.positions.push_back(finger);

	mGripperPub.publish(pos);
}


