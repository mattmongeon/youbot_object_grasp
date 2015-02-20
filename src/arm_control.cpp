#include <moveit/kinematics_base/kinematics_base.h>
#include <pluginlib/class_loader.h>
#include <ros/ros.h>
#include <geometry_msgs/Pose.h>

#include <iostream>


const std::string PLUGIN = "youbot_arm_kinematics_moveit::KinematicsPlugin";
typedef boost::shared_ptr<kinematics::KinematicsBase> KinematicsBasePtr;


void driveArm()
{
    pluginlib::ClassLoader<kinematics::KinematicsBase> loader("moveit_core", "kinematics::KinematicsBase");
    KinematicsBasePtr kinematics = loader.createInstance(PLUGIN);
    kinematics->initialize("/robot_description", "arm_1", "arm_link_0", "arm_link_5", 0.1);

    geometry_msgs::Pose pose;
    std::vector<double> seed(5, 0.0);
    std::vector<double> solution;
    moveit_msgs::MoveItErrorCodes error_code;

    pose.position.x = 0.057;
    pose.position.y = 0.0;
    pose.position.z = 0.535;

    if( kinematics->getPositionIK(pose, seed, solution, error_code) )
	{
		std::cerr << "Found a solution" << std::endl;
	}
	else
	{
		std::cerr << "NO SOLUTION" << std::endl;
	}
}


int main (int argc, char** argv)
{
	ros::init(argc, argv, "arm_control");
    ros::NodeHandle nh;
	driveArm();
	ros::spin();
	return 0;
}
