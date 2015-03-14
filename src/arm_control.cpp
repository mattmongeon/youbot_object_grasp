#include <moveit/kinematics_base/kinematics_base.h>
#include <pluginlib/class_loader.h>
#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib_msgs/GoalStatusArray.h>
#include <std_msgs/Float64.h>
#include <tf/LinearMath/Transform.h>
#include <tf/LinearMath/Quaternion.h>
#include <tf/LinearMath/Vector3.h>
#include <tf/LinearMath/Matrix3x3.h>

#include <iostream>
#include <vector>


// --- Constants --- //

const double PI = 3.14159265358979323846;


// --- ROS Stuff --- //

const std::string PLUGIN = "youbot_arm_kinematics_moveit::KinematicsPlugin";
typedef boost::shared_ptr<kinematics::KinematicsBase> KinematicsBasePtr;
ros::Publisher armJoint1;
ros::Publisher armJoint2;
ros::Publisher armJoint3;
ros::Publisher armJoint4;
ros::Publisher armJoint5;
ros::Publisher baseVelPub;

ros::Subscriber blockPoseSub;
ros::Subscriber odomSub;
ros::Subscriber moveBaseGoalStatusSub;


// --- Helper transformation matrices --- //

// These define some transformations between various links and parts of the robot.
tf::Transform g_A5ToAsus;
tf::Transform g_AsusToA5;
tf::Transform g_base_link_to_arm0;
tf::Transform g_arm0_to_base_link;


// --- Poses of arm_link_5 relative to arm_link_0 --- //

tf::Transform g_cameraSearch_05;
double seedCameraSearch[] = { 2.93215, 0.25865, -0.84097, 2.52836, 2.92343 };

// Set up some joint angle values for seeding the ik solver when we need to start
// grasping objects.
double seedGraspForward[] = { 2.95, 2.04427, -1.51891, 2.54343, 2.93883 };
double seedGraspRight90Deg[] = { 4.56, 2.04427, -1.51891, 2.54343, 2.93883 };
double seedGraspRight45Deg[] = { 3.76, 2.04427, -1.51891, 2.54343, 2.93883 };
double seedGraspLeft90Deg[] = { 1.37, 2.04427, -1.51891, 2.54343, 2.93883 };
double seedGraspLeft45Deg[] = { 2.16, 2.04427, -1.51891, 2.54343, 2.93883 };

tf::Transform g_startGraspForward_05;
tf::Transform g_startGraspRight90Deg_05;
tf::Transform g_startGraspRight45Deg_05;
tf::Transform g_startGraspLeft90Deg_05;
tf::Transform g_startGraspLeft45Deg_05;


// --- Starting Position --- //

tf::Transform g_StartingPose_w;


// --- Target Position --- //

double targetX_m = 0.0;
double targetY_m = 0.0;
bool targetSet = false;


// --- Nav Stuff --- //

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
move_base_msgs::MoveBaseGoal goal;

ros::Publisher moveBaseGoalPub;

tf::Transform currentOdom;


// --- Controller Values --- //

enum ProcessState
{
	Initializing,
	WaitingForBlock,
	NavigatingToBlock,
	MovingArmToSearchPose,
	AligningToBlock,
	GraspingBlock,
	PuttingArmInCarryPose,
	InitiatingTurningAround,
	TurningAround,
	InitiatingReturnToStart,
	ReturningToStart,
	InitiateTurnToOriginalYaw,
	TurningToOriginalYaw,
	Finished
};

ProcessState currentState = Initializing;

bool blockFound = false;

tf::Transform g_baseToBlock;

int navGoalStatus = 0;


void initialize(KinematicsBasePtr kinematics)
{
	tf::Matrix3x3 rot;
	tf::Vector3 t;
	
	// --- arm_link_5 to ASUS center --- //
	
	rot.setValue(0.0, -1.0, 0.0,
				 1.0, 0.0, 0.0,
				 0.0, 0.0, 1.0);
	t.setValue(0.088, 0.0, 0.22);
		
	g_A5ToAsus.setBasis(rot);
	g_A5ToAsus.setOrigin(t);


	// --- ASUS center to arm_link_5 --- //
		
	g_AsusToA5 = g_A5ToAsus.inverse();


	// --- base_link to arm_link_0 --- //

	rot.setValue( 1.0, 0.0, 0.0,
				  0.0, 1.0, 0.0,
				  0.0, 0.0, 1.0 );
	t.setValue( 0.143, 0.0, 0.046 );
		
	g_base_link_to_arm0.setBasis(rot);
	g_base_link_to_arm0.setOrigin(t);


	// --- arm_link_0 to base_link --- //
	
	g_arm0_to_base_link = g_base_link_to_arm0.inverse();

	
	// --- Define the Camera Search pose --- //

	// This goes from base_link to arm_link_5
	rot.setValue( 0.0, 0.0, 1.0,
				  0.0, 1.0, 0.0,
				  -1.0, 0.0, 0.0 );
	t.setValue( 0.310676, 0.00250788, 0.351227 );
	
	g_cameraSearch_05.setBasis(rot);
	g_cameraSearch_05.setOrigin(t);

	// Now we will use the arm_link_0 -> base_link transformation matrix
	// to get the camera search transformation to be from arm_link_0 to 5.
	g_cameraSearch_05 = g_arm0_to_base_link * g_cameraSearch_05;


	// --- Define Grasping Poses --- //

	// Forward

	tf::Quaternion q( -0.016, 0.975, 0.0, 0.222 );
	t.setValue( 0.355, 0.006, 0.059 );

	g_startGraspForward_05.setRotation(q);
	g_startGraspForward_05.setOrigin(t);
	
	// Right 90 degrees.

	q.setValue( 0.692, 0.687, -0.16, 0.154 );
	t.setValue( 0.017, -0.331, 0.059 );

	g_startGraspRight90Deg_05.setRotation(q);
	g_startGraspRight90Deg_05.setOrigin(t);

	// Right 45 degrees.
	
	q.setValue( 0.37, 0.902, -0.087, 0.204 );
	t.setValue( 0.256, -0.236, 0.059 );

	g_startGraspRight45Deg_05.setRotation(q);
	g_startGraspRight45Deg_05.setOrigin(t);

	// Left 45 degrees.

	q.setValue( -0.39, 0.894, 0.086, 0.205 );
	t.setValue( 0.253, 0.239, 0.059 );

	g_startGraspLeft45Deg_05.setRotation(q);
	g_startGraspLeft45Deg_05.setOrigin(t);

	// Left 90 degrees.
	
	q.setValue( 0.704, -0.675, -0.158, -0.156 );
	t.setValue( 0.015, 0.331, 0.059 );

	g_startGraspLeft90Deg_05.setRotation(q);
	g_startGraspLeft90Deg_05.setOrigin(t);
}


void positionArm_fk(const std::vector<double>& angles)
{
	std::cerr << "Publishing joint angles:" << std::endl;
	std_msgs::Float64 v0;
	std::cerr << "\tJoint 1:  " << angles[0] << std::endl;
	v0.data = angles[0];
	armJoint1.publish(v0);

	std_msgs::Float64 v1;
	std::cerr << "\tJoint 2:  " << angles[1] << std::endl;
	v1.data = angles[1];
	armJoint2.publish(v1);

	std_msgs::Float64 v2;
	std::cerr << "\tJoint 3:  " << angles[2] << std::endl;
	v2.data = angles[2];
	armJoint3.publish(v2);

	std_msgs::Float64 v3;
	std::cerr << "\tJoint 4:  " << angles[3] << std::endl;
	v3.data = angles[3];
	armJoint4.publish(v3);

	std_msgs::Float64 v4;
	std::cerr << "\tJoint 5:  " << angles[4] << std::endl;
	v4.data = angles[4];
	armJoint5.publish(v4);
	std::cerr << std::endl;
}


void positionArm_ik(KinematicsBasePtr kinematics, const tf::Transform& g, double* seedVals)
{
	std::vector<double> seed;
	seed.push_back(seedVals[0]);
	seed.push_back(seedVals[1]);
	seed.push_back(seedVals[2]);
	seed.push_back(seedVals[3]);
	seed.push_back(seedVals[4]);
    std::vector<double> solution;
    moveit_msgs::MoveItErrorCodes error_code;

	// Candle position.
    // pose.position.x = 0.057;
    // pose.position.y = 0.0;
    // pose.position.z = 0.535;

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

    if( kinematics->getPositionIK(pose, seed, solution, error_code) )
	{
		std::cerr << "Found a solution" << std::endl;
		std::cerr << "Size of solution:  " << solution.size() << std::endl;
		for( std::size_t i = 0; i < solution.size(); ++i )
		{
			std::cerr << "Joint " << i << ":  " << solution[i] << std::endl;
		}

		positionArm_fk( solution );
	}
	else
	{
		std::cerr << "NO SOLUTION" << std::endl;
	}
}


// Helper function that takes in a geometry_msgs::Pose object and
// returns a new tf::Transform.
tf::Transform getTransformFromPose(const geometry_msgs::Pose& pose)
{
	tf::Quaternion q( pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w );
	tf::Vector3 t( pose.position.x, pose.position.y, pose.position.z );
	
	return tf::Transform( q, t );
}


tf::Transform getBaseToBlockTransform(const geometry_msgs::Pose& pose_ASUStoBlock)
{
	// Start off as base_link -> arm_link_0
	tf::Transform g_baseToBlock = g_base_link_to_arm0;

	// Now take it from base_link -> arm_link_5
	g_baseToBlock *= g_cameraSearch_05;

	// Now take it from base_link -> ASUS frame
	g_baseToBlock *= g_A5ToAsus;

	// Now take it the rest of the way from base_link -> block.  The transform returned
	// from the function is ASUS -> block.
	g_baseToBlock *= getTransformFromPose(pose_ASUStoBlock);

	return g_baseToBlock;
}


void block_callback(const geometry_msgs::Pose& pose)
{
	if( blockFound )
		return;
	
	std::cerr << "Received block pose!" << std::endl;
	std::cerr << "Quaternion:" << std::endl;
	std::cerr << "\tw: " << pose.orientation.w << std::endl;
	std::cerr << "\tx: " << pose.orientation.x << std::endl;
	std::cerr << "\ty: " << pose.orientation.y << std::endl;
	std::cerr << "\tz: " << pose.orientation.z << std::endl;
	std::cerr << std::endl;
	std::cerr << "Position:" << std::endl;
	std::cerr << "\tx: " << pose.position.x << std::endl;
	std::cerr << "\ty: " << pose.position.y << std::endl;
	std::cerr << "\tz: " << pose.position.z << std::endl;
	std::cerr << std::endl;

	// Build a transformation matrix
	g_baseToBlock = getBaseToBlockTransform(pose);

	tf::Matrix3x3 rot = g_baseToBlock.getBasis();
	std::cerr << "Block pose relative to base frame" << std::endl;
	tf::Vector3 row = rot.getRow(0);
	std::cerr << "    | " << row.getX() << " " << row.getY() << " " << row.getZ() << " | " << std::endl;
	row = rot.getRow(1);
	std::cerr << "R = | " << row.getX() << " " << row.getY() << " " << row.getZ() << " | " << std::endl;
	row = rot.getRow(2);
	std::cerr << "    | " << row.getX() << " " << row.getY() << " " << row.getZ() << " | " << std::endl;
	std::cerr << std::endl;
	tf::Vector3 t = g_baseToBlock.getOrigin();
	std::cerr << "t = < " << t.getX() << ", " << t.getY() << ", " << t.getZ() << " >" << std::endl;

	blockFound = true;
}

void moveToAbsolutePosition( const tf::Transform& g )
{
	tf::Vector3 t = g.getOrigin();
	tf::Quaternion q = g.getRotation();

	std::cerr << "Preparing to publish move goal." << std::endl;
	geometry_msgs::PoseStamped goalPose;
	goalPose.header.stamp = ros::Time::now();
	goalPose.header.frame_id = "map";

	goalPose.pose.position.x = t.getX();
	goalPose.pose.position.y = t.getY();

	/*
	goalPose.pose.orientation.x = q.getX();
	goalPose.pose.orientation.y = q.getY();
	goalPose.pose.orientation.z = q.getZ();
	*/
	goalPose.pose.orientation.w = q.getW();

	std::cerr << "Publishing to /move_base_goal/simple" << std::endl;
	moveBaseGoalPub.publish(goalPose);
}

void moveRelativeToBaseLink( const tf::Transform& g )
{
	tf::Transform goal = currentOdom * g;
	tf::Vector3 t = goal.getOrigin();
	tf::Quaternion q = goal.getRotation();

	std::cerr << "Preparing to publish move goal." << std::endl;
	geometry_msgs::PoseStamped goalPose;
	goalPose.header.stamp = ros::Time::now();
	goalPose.header.frame_id = "map";

	goalPose.pose.position.x = t.getX();
	goalPose.pose.position.y = t.getY();

	/*
	goalPose.pose.orientation.x = q.getX();
	goalPose.pose.orientation.y = q.getY();
	goalPose.pose.orientation.z = q.getZ();
	*/
	goalPose.pose.orientation.w = q.getW();

	std::cerr << "Publishing to /move_base_goal/simple" << std::endl;
	moveBaseGoalPub.publish(goalPose);
}

void odom_callback(const nav_msgs::Odometry& odom)
{
	tf::Vector3 t( odom.pose.pose.position.x,
				   odom.pose.pose.position.y,
				   odom.pose.pose.position.z );
	tf::Quaternion q( odom.pose.pose.orientation.x,
					  odom.pose.pose.orientation.y,
					  odom.pose.pose.orientation.z,
					  odom.pose.pose.orientation.w );
	currentOdom.setOrigin( t );
	currentOdom.setRotation( q );
}

void move_base_status_callback( const actionlib_msgs::GoalStatusArray& status )
{
	if( !status.status_list.empty() )
	{
		int index = status.status_list.size() - 1;
		navGoalStatus = status.status_list[index].status;
	}
}

int main( int argc, char** argv )
{
	ros::init(argc, argv, "arm_control");
    ros::NodeHandle nh;

	
	std::cerr << "Creating publishers." << std::endl;
	armJoint1 = nh.advertise<std_msgs::Float64>( "/youbot/arm_joint_1_position_controller/command", 1 );
	armJoint2 = nh.advertise<std_msgs::Float64>( "/youbot/arm_joint_2_position_controller/command", 1 );
	armJoint3 = nh.advertise<std_msgs::Float64>( "/youbot/arm_joint_3_position_controller/command", 1 );
	armJoint4 = nh.advertise<std_msgs::Float64>( "/youbot/arm_joint_4_position_controller/command", 1 );
	armJoint5 = nh.advertise<std_msgs::Float64>( "/youbot/arm_joint_5_position_controller/command", 1 );

	baseVelPub = nh.advertise<geometry_msgs::Twist>( "/youbot/cmd_vel", 1 );

	moveBaseGoalPub = nh.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 5, true);
	
	
	std::cerr << "Creating subscribers." << std::endl;
    blockPoseSub = nh.subscribe( "/block_pose", 1, block_callback );
	odomSub = nh.subscribe( "/youbot/odom", 1, odom_callback );
	moveBaseGoalStatusSub = nh.subscribe( "/move_base/status", 1, move_base_status_callback );

	
	std::cerr << "Waiting 5 seconds to allow everything to start up." << std::endl;
	for( int i = 5; i > 0; --i )
	{
		std::cerr << i << std::endl;
		ros::Duration(1).sleep();
	}


	std::cerr << "Creating kinematics object." << std::endl;
    pluginlib::ClassLoader<kinematics::KinematicsBase> loader("moveit_core", "kinematics::KinematicsBase");
    KinematicsBasePtr kinematics = loader.createInstance(PLUGIN);
	kinematics->initialize("/robot_description", "arm_1", "arm_link_0", "arm_link_5", 0.1);
	
	
	std::cerr << "Initializing matrices and other things." << std::endl;
	initialize(kinematics);
	
	std::cerr << "Driving arm to camera position." << std::endl;
	positionArm_ik( kinematics, g_cameraSearch_05, seedCameraSearch );

	currentState = WaitingForBlock;
	while(ros::ok())
	{
		switch(currentState)
		{
		case WaitingForBlock:
			if( blockFound )
			{
				// First let's save our starting position so we can return to it.
				g_StartingPose_w = currentOdom;
				
				// Drive the base next to the block.
				tf::Transform goal = g_baseToBlock;
				goal.getOrigin().setY( goal.getOrigin().getY() - 0.5 );

				moveRelativeToBaseLink(goal);
				currentState = NavigatingToBlock;
				std::cerr << "Exiting the WaitingForBlock state" << std::endl;
				std::cerr << std::endl;
				std::cerr << "Entering NavigatingToBlock state" << std::endl;
			}
			break;

			
		case NavigatingToBlock:
			if( navGoalStatus == 3 )  // state SUCCEEDED
			{
				currentState = MovingArmToSearchPose;
				navGoalStatus = 0;
				std::cerr << "Reached navigation goal." << std::endl;
				std::cerr << "Exiting NavigatingToBlock state" << std::endl;
			}
			break;

			
		case MovingArmToSearchPose:
		{
			std::cerr << std::endl;
			std::cerr << "Entering MovingArmToSearchPose state" << std::endl;
			std::cerr << "Moving arm to search pose." << std::endl;
			positionArm_ik( kinematics, g_startGraspLeft90Deg_05, seedGraspLeft90Deg );
			currentState = AligningToBlock;
			std::cerr << "Exiting MovingArmToSearchPose state" << std::endl;
			break;
		}

		
		case AligningToBlock:
		{
			std::cerr << std::endl;
			std::cerr << "Entering AligningToBlock state" << std::endl;
			currentState = GraspingBlock;
			std::cerr << "Exiting AligningToBlock state" << std::endl;
			break;
		}

		
		case GraspingBlock:
		{
			std::cerr << std::endl;
			std::cerr << "Entering GraspingBlock state" << std::endl;
			std::cerr << "Waiting 5 seconds to allow arm to finish moving." << std::endl;
			ros::Duration(5).sleep();

			tf::Transform translate;
			translate.setIdentity();
			translate.getOrigin().setZ( translate.getOrigin().getZ() + 0.05 );

			std::cerr << "Translating along search pose." << std::endl;
			positionArm_ik( kinematics, g_startGraspLeft90Deg_05 * translate, seedGraspLeft90Deg );
			ros::Duration(3.0).sleep();  // Allow the arm to translate so we see it.
			currentState = PuttingArmInCarryPose;
			std::cerr << "Exiting GraspingBlock state" << std::endl;
			break;
		}

		
		case PuttingArmInCarryPose:
		{
			std::cerr << std::endl;
			std::cerr << "Entered PuttingArmInCarryPose state" << std::endl;
			std::cerr << "Driving arm to carry pose." << std::endl;
			positionArm_ik( kinematics, g_cameraSearch_05, seedCameraSearch );
			std::cerr << "Waiting 3 seconds to allow arm to reach pose" << std::endl;
			ros::Duration(3.0).sleep();  // Allow the arm to reach the pose.
			currentState = InitiatingTurningAround;
			navGoalStatus = 0;
			std::cerr << "Exiting PuttingArmInCarryPose state" << std::endl;
			break;
		}


		case InitiatingTurningAround:
		{
			std::cerr << std::endl;
			std::cerr << "Entering InitiatingTurningAround state" << std::endl;
			std::cerr << "navGoalStatus:  " << navGoalStatus << std::endl;
			std::cerr << "Publishing goal and waiting for status to change" << std::endl;

			tf::Matrix3x3 r;
			r.setRPY(0, 0, PI);
			
			tf::Transform g_yawed;
			g_yawed.setBasis(r);

			moveRelativeToBaseLink(g_yawed);

			while( navGoalStatus == 3 )
			{
				ros::spinOnce();
			}
			std::cerr << "Move goal accepted" << std::endl;

			std::cerr << "Exiting InitiatingTurningAround state" << std::endl;
			std::cerr << std::endl;
			std::cerr << "Entering TurningAround state" << std::endl;
			currentState = TurningAround;

			break;
		}

		
		case TurningAround:
		{
			if( navGoalStatus == 3 )  // state SUCCEEDED
			{
				currentState = InitiatingReturnToStart;
				std::cerr << "Reached yaw goal." << std::endl;
				std::cerr << "Exiting TurningAround state" << std::endl;
			}
			break;
		}


		case InitiatingReturnToStart:
		{
			/*
			std::cerr << std::endl;
			std::cerr << "Entering InitiatingReturnToStart state" << std::endl;
			std::cerr << "navGoalStatus:  " << navGoalStatus << std::endl;
			std::cerr << "Publishing goal and waiting for status to change" << std::endl;

			moveToAbsolutePosition(g_StartingPose_w);

			while( navGoalStatus == 3 )
			{
				ros::spinOnce();
			}

			currentState = ReturningToStart;
			std::cerr << "Exiting InitiatingReturnToStart" << std::endl;
			std::cerr << std::endl;
			std::cerr << "Entering ReturningToStart state" << std::endl;
			*/
			break;
		}
		
		case ReturningToStart:
		{
			if( navGoalStatus == 3 )
			{
				// When we have reached the goal, transition to the next state.
				currentState = TurningToOriginalYaw;
				std::cerr << "Exiting the ReturningToStart state" << std::endl;
				std::cerr << std::endl;
				std::cerr << "Entering final state:  TurningToOriginalYaw" << std::endl;
			}
			break;
		}


		case InitiateTurnToOriginalYaw:
		{
			break;
		}

		
		case TurningToOriginalYaw:
			break;

			
		default:
			break;
		}

		// Exit our loop if we are finished.
		if( currentState == Finished )
		{
			std::cerr << "Finished executing!  Exiting." << std::endl;
			break;
		}

		ros::spinOnce();
	}
	
	return 0;
}
