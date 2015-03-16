#include "youbot_object_grasp/arm_interface.h"
#include <pluginlib/class_loader.h>
#include <geometry_msgs/Pose.h>
#include <tf/LinearMath/Quaternion.h>
#include <tf/LinearMath/Vector3.h>
#include <tf/LinearMath/Matrix3x3.h>
#include <iostream>

////////////////////////////////////////////////////////////////////////////////
//  Construction / Destruction
////////////////////////////////////////////////////////////////////////////////

cArmInterface::cArmInterface()
{
    pluginlib::ClassLoader<kinematics::KinematicsBase> loader("moveit_core", "kinematics::KinematicsBase");
    mpArmKinematics = loader.createInstance("youbot_arm_kinematics_moveit::KinematicsPlugin");
	mpArmKinematics->initialize("/robot_description", "arm_1", "arm_link_0", "arm_link_5", 0.1);
}

////////////////////////////////////////////////////////////////////////////////
//  Interface Functions
////////////////////////////////////////////////////////////////////////////////

bool cArmInterface::PositionArm(const tf::Transform& g, const std::vector<double>& seedVals)
{
    std::vector<double> solution;
    moveit_msgs::MoveItErrorCodes error_code;

	const tf::Vector3& position = g.getOrigin();
    geometry_msgs::Pose pose;
	pose.position.x = position.getX();
	pose.position.y = position.getY();
	pose.position.z = position.getZ();

	const tf::Matrix3x3& rot = g.getBasis();
	tf::Quaternion q;
	rot.getRotation( q );
	pose.orientation.w = q.getW();
	pose.orientation.x = q.getX();
	pose.orientation.y = q.getY();
	pose.orientation.z = q.getZ();

    if( mpArmKinematics->getPositionIK(pose, seedVals, solution, error_code) )
	{
		std::cerr << "Found a solution" << std::endl;
		std::cerr << "Size of solution:  " << solution.size() << std::endl;
		for( std::size_t i = 0; i < solution.size(); ++i )
		{
			std::cerr << "Joint " << i << ":  " << solution[i] << std::endl;
		}

		PublishJointValues(solution);
	}
	else
	{
		std::cerr << "NO SOLUTION" << std::endl;
	}
}

