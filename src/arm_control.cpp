#include <moveit/kinematics_base/kinematics_base.h>
#include <pluginlib/class_loader.h>
#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <std_msgs/Float64.h>
#include <tf/LinearMath/Quaternion.h>
#include <tf/LinearMath/Vector3.h>

#include <iostream>


const std::string PLUGIN = "youbot_arm_kinematics_moveit::KinematicsPlugin";
typedef boost::shared_ptr<kinematics::KinematicsBase> KinematicsBasePtr;
ros::Publisher armJoint1;
ros::Publisher armJoint2;
ros::Publisher armJoint3;
ros::Publisher armJoint4;
ros::Publisher armJoint5;

// Set up some joint angle values for seeding the ik solver when we need to start
// grasping objects.
double seedGraspForward[] = { 2.95, 2.04, -1.75, 2.64, 2.9 };
double seedGraspRight90Deg[] = { 4.56, 2.04, -1.75, 2.64, 2.9 };
double seedGraspRight45Deg[] = { 3.76, 2.04, -1.75, 2.64, 2.9 };
double seedGraspLeft90Deg[] = { 1.37, 2.04, -1.75, 2.64, 2.9 };
double seedGraspLeft45Deg[] = { 2.16, 2.04, -1.75, 2.64, 2.9 };


void driveArm()
{
    pluginlib::ClassLoader<kinematics::KinematicsBase> loader("moveit_core", "kinematics::KinematicsBase");
    KinematicsBasePtr kinematics = loader.createInstance(PLUGIN);
	kinematics->initialize("/robot_description", "arm_1", "arm_link_0", "arm_link_5", 0.1);

    geometry_msgs::Pose pose;
    // std::vector<double> seed(5, 0.0);
	std::vector<double> seed;
	seed.push_back(1.52);
	seed.push_back(1.84);
	seed.push_back(-1.26);
	seed.push_back(2.4);
	seed.push_back(3.10);
    std::vector<double> solution;
    moveit_msgs::MoveItErrorCodes error_code;

	// Figure out rotation matix, then use matrix logarithm to find the axis of
	//     rotation and the magnitude.
	// pose.quaternion = Quaternion(const Vector3& axis, const tfScalar& angle);

	// Values from Jarvis's test file.  Requires URDF with virtual joints.
	pose.orientation.w = 0.601;
	pose.orientation.x = 0.591;
	pose.orientation.y = -0.372;
	pose.orientation.z = 0.388;

    pose.position.x = 0.181;
    pose.position.y = 0.778;
    pose.position.z = 0.108;

	// Candle position.
    // pose.position.x = 0.057;
    // pose.position.y = 0.0;
    // pose.position.z = 0.535;

    if( kinematics->getPositionIK(pose, seed, solution, error_code) )
	{
		std::cerr << "Found a solution" << std::endl;
		std::cerr << "Size of solution:  " << solution.size() << std::endl;
		for( std::size_t i = 0; i < solution.size(); ++i )
		{
			std::cerr << "Joint " << i << ":  " << solution[i] << std::endl;
		}

		std_msgs::Float64 v0;
		v0.data = solution[0];
		armJoint1.publish(v0);

		std_msgs::Float64 v1;
		v1.data = solution[1];
		armJoint2.publish(v1);

		std_msgs::Float64 v2;
		v2.data = solution[2];
		armJoint3.publish(v2);

		std_msgs::Float64 v3;
		v3.data = solution[3];
		armJoint4.publish(v3);

		std_msgs::Float64 v4;
		v4.data = solution[4];
		armJoint5.publish(v4);
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
	
	std::cerr << "Creating publishers." << std::endl;
	armJoint1 = nh.advertise<std_msgs::Float64>( "/youbot/arm_joint_1_position_controller/command", 1 );
	armJoint2 = nh.advertise<std_msgs::Float64>( "/youbot/arm_joint_2_position_controller/command", 1 );
	armJoint3 = nh.advertise<std_msgs::Float64>( "/youbot/arm_joint_3_position_controller/command", 1 );
	armJoint4 = nh.advertise<std_msgs::Float64>( "/youbot/arm_joint_4_position_controller/command", 1 );
	armJoint5 = nh.advertise<std_msgs::Float64>( "/youbot/arm_joint_5_position_controller/command", 1 );

	std::cerr << "Waiting 5 seconds to allow everything to start up." << std::endl;
	for( int i = 5; i > 0; --i )
	{
		std::cerr << i << std::endl;
		ros::Duration(1).sleep();
	}

	std::cerr << "Driving arm." << std::endl;
	driveArm();
	ros::spin();
	return 0;
}
