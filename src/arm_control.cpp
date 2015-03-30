#include <moveit/kinematics_base/kinematics_base.h>
#include <pluginlib/class_loader.h>
#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>
#include <nav_msgs/Odometry.h>
#include <actionlib_msgs/GoalStatusArray.h>
#include <std_msgs/Float64.h>
#include <tf/LinearMath/Transform.h>
#include <tf/LinearMath/Quaternion.h>
#include <tf/LinearMath/Vector3.h>
#include <tf/LinearMath/Matrix3x3.h>
#include <tf/transform_listener.h>

#include "youbot_object_grasp/arm_interface_gazebo.h"
#include "youbot_object_grasp/arm_interface_youbot.h"

#include <iostream>
#include <vector>


// --- Constants --- //

const double PI = 3.14159265358979323846;


// --- ROS Stuff --- //

bool usingGazebo = false;

cArmInterface* pArmInterface;

ros::Publisher baseVelPub;

ros::Subscriber blockPoseSub;
ros::Subscriber odomSub;
ros::Subscriber moveBaseGoalStatusSub;
ros::Subscriber floorNormalSub;

tf::TransformListener* listener;

std::string odomFrameName;
std::string globalFrameName;


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


// --- Nav Stuff --- //

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
	InitiatingReturnToStart,
	ReturningToStart,
	Finished
};

ProcessState currentState = Initializing;

bool cameraCalibrated = false;
bool blockFound = false;

tf::Transform g_baseToBlock;

int navGoalStatus = 0;


void initialize()
{
	tf::Matrix3x3 rot;
	tf::Vector3 t;
	
	// --- arm_link_5 to ASUS center --- //

	// Initialize the transformation for arm_lin_5 -> ASUS.  The rotation will
	// later be automatically calibrated.
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

	if( !cameraCalibrated )
	{
		std::cout << "Camera not calibrated yet.  Throwing away block pose." << std::endl;
		std::cout << std::endl;
		return;
	}

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
	goalPose.header.frame_id = globalFrameName;

	goalPose.pose.position.x = t.getX();
	goalPose.pose.position.y = t.getY();

	// Ensure we have no rotation except possibly about the z axis.
	tf::Matrix3x3 rot( q );
	tfScalar r, p, y;
	rot.getRPY( r, p, y );
	q.setRPY( 0, 0, y );

	goalPose.pose.orientation.x = q.getX();
	goalPose.pose.orientation.y = q.getY();
	goalPose.pose.orientation.z = q.getZ();
	goalPose.pose.orientation.w = q.getW();

	std::cerr << "Publishing to /move_base_goal/simple" << std::endl;
	moveBaseGoalPub.publish(goalPose);
}

void moveRelativeToBaseLink( const tf::Transform& g )
{
	// At some point we might be using the map frame as our global frame.
	// At other times we are simply using odom.  We will assume we are using
	// the odom frame and initialize this transformation matrix to be an
	// identity matrix.  If we are using something other than odom, we can
	// use TF to get the transform.
	tf::StampedTransform g_mapToOdom;
	tf::Vector3 t( 0, 0, 0 );
	g_mapToOdom.setOrigin( t );
	g_mapToOdom.setBasis( tf::Matrix3x3::getIdentity() );
	
	if( globalFrameName != "odom" )
	{
		try
		{
			listener->lookupTransform(globalFrameName, odomFrameName, ros::Time(0), g_mapToOdom);
		}
		catch(tf::TransformException e)
		{
			std::cerr << std::endl;
			std::cerr << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!" << std::endl;
			std::cerr << "Error getting transform through tf:" << std::endl;
			std::cerr << e.what() << std::endl;
			std::cerr << std::endl;
			std::cerr << "Setting to identity matrix" << std::endl;
			std::cerr << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!" << std::endl;
			std::cerr << std::endl;
		}
	}

	moveToAbsolutePosition( g_mapToOdom * currentOdom * g );
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

void floor_normal_callback( const geometry_msgs::Vector3& norm )
{
	// --- First Transform Everything To Base Frame --- //

	tf::Vector3 floorNormal(norm.x, norm.y, norm.z);

	// Start off as base_link -> arm_link_0
	tf::Transform g_baseToASUS = g_base_link_to_arm0;

	// Now take it from base_link -> arm_link_5
	g_baseToASUS *= g_cameraSearch_05;

	// Now take it from base_link -> ASUS frame
	g_baseToASUS *= g_A5ToAsus;

	// Now get the vector in terms of the base frame.
	floorNormal = g_baseToASUS*floorNormal;


	// --- Get Rotation Error --- //
	
	// Ultimately we want to find a quaternion such that we can transform the
	// floor normal so that it aligns with the base z axis.
	tf::Vector3 baseZAxis(0, 0, 1);
	tf::Vector3 cross = baseZAxis.cross(floorNormal);

	// This will give us the rotation from the base's normal vector to the floor's normal
	// from the camera's perspective.  We want to get 
	tf::Quaternion q( cross.getX(),
					  cross.getY(),
					  cross.getZ(),
					  sqrt(floorNormal.length2()*baseZAxis.length2()) + baseZAxis.dot(floorNormal) );


	// --- Now Apply It As A Correction --- //

	tf::Transform g_Correction;
	g_Correction.setIdentity();
	g_Correction.setRotation(q);

	// We need the inverse of this matrix so that we undo the rotation error from the floor normal to
	// the base's coordinate frame.
	g_A5ToAsus *= g_Correction.inverse();

	
	cameraCalibrated = true;
}

int main( int argc, char** argv )
{
	ros::init(argc, argv, "arm_control");
    ros::NodeHandle nh("~");

	// --- Parameters --- //
	
	ros::param::get("/using_gazebo", usingGazebo);
	nh.param("odom_frame_id", odomFrameName, std::string("/odom"));
	nh.param("global_frame_id", globalFrameName, std::string("odom"));


	// --- Arm Interface object --- //
	
	std::cerr << "Creating arm interface." << std::endl;
	if( usingGazebo )
	{
		std::cerr << "\tUsing Gazebo interface" << std::endl;
		pArmInterface = new cArmInterfaceGazebo(nh);
	}
	else
	{
		std::cerr << "\tUsing youBot interface" << std::endl;
		pArmInterface = new cArmInterfaceYoubot(nh);
	}


	// --- Base Movement Publishers --- //
	
	baseVelPub = nh.advertise<geometry_msgs::Twist>( "/cmd_vel", 1 );
	moveBaseGoalPub = nh.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 5, true);


	// --- TF --- //
	
	listener = new tf::TransformListener();
	

	// --- Subscribers --- //
	
	std::cerr << "Creating subscribers." << std::endl;
    blockPoseSub = nh.subscribe( "/block_pose", 1, block_callback );
	odomSub = nh.subscribe( "/odom", 1, odom_callback );
	moveBaseGoalStatusSub = nh.subscribe( "/move_base/status", 1, move_base_status_callback );
	floorNormalSub = nh.subscribe( "/floor_normal", 1, floor_normal_callback );


	// --- Initialization --- //

	std::cerr << "Initializing matrices and other things." << std::endl;
	initialize();
	
	std::cerr << "Waiting 5 seconds to allow everything to start up." << std::endl;
	for( int i = 5; i > 0; --i )
	{
		std::cerr << i << std::endl;
		ros::Duration(1).sleep();
	}


	// --- Begin --- //
	
	std::cerr << "Driving arm to camera position." << std::endl;
	std::vector<double> seedVals;
	seedVals.push_back(seedCameraSearch[0]);
	seedVals.push_back(seedCameraSearch[1]);
	seedVals.push_back(seedCameraSearch[2]);
	seedVals.push_back(seedCameraSearch[3]);
	seedVals.push_back(seedCameraSearch[4]);
	pArmInterface->PositionArm( g_cameraSearch_05, seedVals );
	ros::Duration(3).sleep();  // Wait for the arm to get to the position.

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
				
				// Drive the base next to the block.  Make sure we don't rotate
				// based on the rotation matrix of the block itself.
				tf::Transform goal = g_baseToBlock;
				goal.getOrigin().setX( goal.getOrigin().getX() - 0.25 );
				if( goal.getOrigin().getY() < 0.0 )
				  goal.getOrigin().setY( goal.getOrigin().getY() + 0.25 );
				else
				  goal.getOrigin().setY( goal.getOrigin().getY() - 0.25 );

				goal.setBasis( tf::Matrix3x3::getIdentity() );

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
			seedVals.clear();
			for( std::size_t i = 0; i < 5; ++i )
			{
				seedVals.push_back(seedGraspLeft90Deg[i]);
			}
			pArmInterface->PositionArm( g_startGraspLeft90Deg_05, seedVals );
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
			seedVals.clear();
			for( std::size_t i = 0; i < 5; ++i )
			{
				seedVals.push_back(seedGraspLeft90Deg[i]);
			}
			pArmInterface->PositionArm( g_startGraspLeft90Deg_05 * translate, seedVals );
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
			seedVals.clear();
			for( std::size_t i = 0; i < 5; ++i )
			{
				seedVals.push_back(seedCameraSearch[i]);
			}
			pArmInterface->PositionArm( g_cameraSearch_05, seedVals );
			std::cerr << "Waiting 3 seconds to allow arm to reach pose" << std::endl;
			ros::Duration(3.0).sleep();  // Allow the arm to reach the pose.
			currentState = InitiatingReturnToStart;
			navGoalStatus = 0;
			std::cerr << "Exiting PuttingArmInCarryPose state" << std::endl;
			break;
		}


		case InitiatingReturnToStart:
		{
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

			break;
		}
		
		case ReturningToStart:
		{
			if( navGoalStatus == 3 )
			{
				// When we have reached the goal, transition to the next state.
				currentState = Finished;
				std::cerr << "Exiting the ReturningToStart state" << std::endl;
				std::cerr << std::endl;
				std::cerr << "Finished!" << std::endl;
			}
			break;
		}


		default:
			break;
		}

		ros::spinOnce();
	}

	delete listener;
	listener = 0;
	
	return 0;
}
