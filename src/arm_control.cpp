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
tf::Transform g_AsusCorrection;


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
tf::Transform* pGraspingTransform;
std::vector<double> graspingSeedVals;

tf::Transform g_baseToBlock;

int navGoalStatus = 0;


void initialize()
{
	tf::Matrix3x3 rot;
	tf::Vector3 t;
	
	// --- arm_link_5 to ASUS center --- //

	// Initialize the transformation for arm_link_5 -> ASUS.  The rotation will
	// later be automatically calibrated.
	rot.setValue(0.0, -1.0, 0.0,
				 1.0, 0.0, 0.0,
				 0.0, 0.0, 1.0);
	t.setValue(0.088, 0.0, 0.022);
		
	g_A5ToAsus.setBasis(rot);
	g_A5ToAsus.setOrigin(t);


	// --- Define the Camera Search pose --- //

	// This goes from base_link to arm_link_5.  It was found using rviz.
	rot.setValue( 0.0, 0.0, 1.0,
				  0.0, 1.0, 0.0,
				  -1.0, 0.0, 0.0 );
	t.setValue( 0.310676, 0.00250788, 0.351227 );
	
	g_cameraSearch_05.setBasis(rot);
	g_cameraSearch_05.setOrigin(t);

	// Now we will use the arm_link_0 -> base_link transformation matrix
	// to get the camera search transformation to be from arm_link_0 to 5.
	tf::StampedTransform g_arm0_to_base_link;
	listener->lookupTransform("arm_link_0", "base_link", ros::Time(0), g_arm0_to_base_link);
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
	tf::StampedTransform g_baseToA5;
	listener->lookupTransform("base_link", "arm_link_5", ros::Time(0), g_baseToA5);
	
	// Start off as base_link -> arm_link_5
	tf::Transform g = g_baseToA5;

	// Now take it from base_link -> ASUS frame
	g *= g_A5ToAsus;

	// Apply the calibration factor.
	//g *= g_AsusCorrection;

	// Now take it the rest of the way from base_link -> block.  The transform returned
	// from the function is ASUS -> block.
	g *= getTransformFromPose(pose_ASUStoBlock);
	
	return g;
}


void block_callback(const geometry_msgs::Pose& pose)
{
	if( blockFound )
		return;
	
	std::cout << "Received block pose!" << std::endl;
	std::cout << "Quaternion:" << std::endl;
	std::cout << "\tw: " << pose.orientation.w << std::endl;
	std::cout << "\tx: " << pose.orientation.x << std::endl;
	std::cout << "\ty: " << pose.orientation.y << std::endl;
	std::cout << "\tz: " << pose.orientation.z << std::endl;
	std::cout << std::endl;
	std::cout << "Position:" << std::endl;
	std::cout << "\tx: " << pose.position.x << std::endl;
	std::cout << "\ty: " << pose.position.y << std::endl;
	std::cout << "\tz: " << pose.position.z << std::endl;
	std::cout << std::endl;

	if( !cameraCalibrated )
	{
		std::cout << "Camera not calibrated yet.  Throwing away block pose." << std::endl;
		std::cout << std::endl;
		return;
	}

	// Build a transformation matrix
	g_baseToBlock = getBaseToBlockTransform(pose);

	tf::Matrix3x3 rot = g_baseToBlock.getBasis();
	std::cout << "Block pose relative to base frame" << std::endl;
	tf::Vector3 row = rot.getRow(0);
	std::cout << "    | " << row.getX() << " " << row.getY() << " " << row.getZ() << " | " << std::endl;
	row = rot.getRow(1);
	std::cout << "R = | " << row.getX() << " " << row.getY() << " " << row.getZ() << " | " << std::endl;
	row = rot.getRow(2);
	std::cout << "    | " << row.getX() << " " << row.getY() << " " << row.getZ() << " | " << std::endl;
	std::cout << std::endl;
	tf::Vector3 t = g_baseToBlock.getOrigin();
	std::cout << "t = < " << t.getX() << ", " << t.getY() << ", " << t.getZ() << " >" << std::endl;

	std::cout << std::endl;

	std::cout << "Arm5 pose relative to base_link" << std::endl;
	tf::StampedTransform g_baseToA5;
	listener->lookupTransform("base_link", "arm_link_5", ros::Time(0), g_baseToA5);
	rot = g_baseToA5.getBasis();
	row = rot.getRow(0);
	std::cout << "    | " << row.getX() << " " << row.getY() << " " << row.getZ() << " | " << std::endl;
	row = rot.getRow(1);
	std::cout << "R = | " << row.getX() << " " << row.getY() << " " << row.getZ() << " | " << std::endl;
	row = rot.getRow(2);
	std::cout << "    | " << row.getX() << " " << row.getY() << " " << row.getZ() << " | " << std::endl;
	std::cout << std::endl;
	t = g_baseToA5.getOrigin();
	std::cout << "t = < " << t.getX() << ", " << t.getY() << ", " << t.getZ() << " >" << std::endl << std::endl;

	std::cout << "ASUS pose relative to arm_link_5" << std::endl;
	rot = g_A5ToAsus.getBasis();
	row = rot.getRow(0);
	std::cout << "    | " << row.getX() << " " << row.getY() << " " << row.getZ() << " | " << std::endl;
	row = rot.getRow(1);
	std::cout << "R = | " << row.getX() << " " << row.getY() << " " << row.getZ() << " | " << std::endl;
	row = rot.getRow(2);
	std::cout << "    | " << row.getX() << " " << row.getY() << " " << row.getZ() << " | " << std::endl;
	std::cout << std::endl;
	t = g_A5ToAsus.getOrigin();
	std::cout << "t = < " << t.getX() << ", " << t.getY() << ", " << t.getZ() << " >" << std::endl << std::endl;

	std::cout << "ASUS pose relative to base_link" << std::endl;
	tf::Transform g = g_baseToA5 * g_A5ToAsus;
	rot = g.getBasis();
	row = rot.getRow(0);
	std::cout << "    | " << row.getX() << " " << row.getY() << " " << row.getZ() << " | " << std::endl;
	row = rot.getRow(1);
	std::cout << "R = | " << row.getX() << " " << row.getY() << " " << row.getZ() << " | " << std::endl;
	row = rot.getRow(2);
	std::cout << "    | " << row.getX() << " " << row.getY() << " " << row.getZ() << " | " << std::endl;
	std::cout << std::endl;
	t = g.getOrigin();
	std::cout << "t = < " << t.getX() << ", " << t.getY() << ", " << t.getZ() << " >" << std::endl << std::endl;
	
	blockFound = true;
}

void moveToAbsolutePosition( const tf::Transform& g )
{
	tf::Vector3 t = g.getOrigin();
	tf::Quaternion q = g.getRotation();

	std::cout << "Preparing to publish move goal." << std::endl;
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

	std::cout << "t = < " << t.getX() << ", " << t.getY() << ", z >" << std::endl;
	std::cout << "Publishing to /move_base_goal/simple" << std::endl;
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
			std::cout << std::endl;
			std::cout << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!" << std::endl;
			std::cout << "Error getting transform through tf:" << std::endl;
			std::cout << e.what() << std::endl;
			std::cout << std::endl;
			std::cout << "Setting to identity matrix" << std::endl;
			std::cout << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!" << std::endl;
			std::cout << std::endl;
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
	if( cameraCalibrated )
		return;

	// --- First Transform Everything To Base Frame --- //

	tf::Vector3 floorNormal(norm.x, norm.y, norm.z);

	std::cout << "Floor normal" << std::endl;
	std::cout << "    | " << floorNormal.getX() << " |" << std::endl;
	std::cout << "    | " << floorNormal.getY() << " |" << std::endl;
	std::cout << "    | " << floorNormal.getZ() << " |" << std::endl;
	std::cout << std::endl;

	
	// --- Get Rotation Error --- //
	
	// Ultimately we want to find a quaternion such that we can transform the
	// floor normal so that it aligns with the base z axis.  To do that we
	// will find a quaternion that aligns the normal to the y-axis of the camera.
	tf::Vector3 cameraYAxis(0, 1, 0);
	if( floorNormal.getY() < 0.0 )
		cameraYAxis.setY(-1.0);

	tfScalar angle = cameraYAxis.angle(floorNormal);
	tf::Vector3 rotAxis = cameraYAxis.cross(floorNormal);
	tf::Quaternion q(rotAxis, angle);
	
	// --- Now Apply It As A Correction --- //

	g_AsusCorrection.setIdentity();
	g_AsusCorrection.setRotation(q);
	g_AsusCorrection = g_AsusCorrection.inverse();

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
	
	std::cout << "Creating arm interface." << std::endl;
	if( usingGazebo )
	{
		std::cout << "\tUsing Gazebo interface" << std::endl;
		pArmInterface = new cArmInterfaceGazebo(nh);
	}
	else
	{
		std::cout << "\tUsing youBot interface" << std::endl;
		pArmInterface = new cArmInterfaceYoubot(nh);
	}


	// --- Base Movement Publishers --- //
	
	baseVelPub = nh.advertise<geometry_msgs::Twist>( "/cmd_vel", 1 );
	moveBaseGoalPub = nh.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 5, true);


	// --- TF --- //
	
	listener = new tf::TransformListener();
	std::cout << "Wait for 2 seconds to allow tfs to buffer" << std::endl;
	ros::Duration(2).sleep();
	

	// --- Subscribers --- //
	
	std::cout << "Creating subscribers." << std::endl;
    blockPoseSub = nh.subscribe( "/block_pose", 1, block_callback );
	odomSub = nh.subscribe( "/odom", 1, odom_callback );
	moveBaseGoalStatusSub = nh.subscribe( "/move_base/status", 1, move_base_status_callback );
	floorNormalSub = nh.subscribe( "/floor_normal", 1, floor_normal_callback );


	// --- Initialization --- //

	std::cout << "Initializing matrices and other things." << std::endl;
	initialize();

	if( usingGazebo )
	{
		// This is because sometimes when using Gazebo all of the extra stuff
		// takes a while to start up.
		std::cout << "Waiting 5 seconds to allow everything to start up." << std::endl;
		for( int i = 5; i > 0; --i )
		{
			std::cout << i << std::endl;
			ros::Duration(1).sleep();
		}
	}


	// --- Begin --- //
	
	std::cout << "Driving arm to camera position." << std::endl;
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
				goal.getOrigin().setX( goal.getOrigin().getX() );
				if( goal.getOrigin().getY() < 0.0 )
				{
					pGraspingTransform = &g_startGraspRight90Deg_05;
					graspingSeedVals.clear();
					for( std::size_t i = 0; i < 5; ++i )
					{
						graspingSeedVals.push_back(seedGraspRight90Deg[i]);
					}
					goal.getOrigin().setY( goal.getOrigin().getY() + 0.5 );
				}
				else
				{
					pGraspingTransform = &g_startGraspLeft90Deg_05;
					graspingSeedVals.clear();
					for( std::size_t i = 0; i < 5; ++i )
					{
						graspingSeedVals.push_back(seedGraspLeft90Deg[i]);
					}
					goal.getOrigin().setY( goal.getOrigin().getY() - 0.5 );
				}

				goal.setBasis( tf::Matrix3x3::getIdentity() );

				moveRelativeToBaseLink(goal);
				currentState = NavigatingToBlock;
				std::cout << "Exiting the WaitingForBlock state" << std::endl;
				std::cout << std::endl;
				std::cout << "Entering NavigatingToBlock state" << std::endl;
			}
			break;

			
		case NavigatingToBlock:
			if( navGoalStatus == 3 )  // state SUCCEEDED
			{
				currentState = MovingArmToSearchPose;
				navGoalStatus = 0;
				std::cout << "Reached navigation goal." << std::endl;
				std::cout << "Exiting NavigatingToBlock state" << std::endl;
			}
			break;

			
		case MovingArmToSearchPose:
		{
			std::cout << std::endl;
			std::cout << "Entering MovingArmToSearchPose state" << std::endl;
			std::cout << "Moving arm to search pose." << std::endl;
			pArmInterface->PositionArm( *pGraspingTransform, graspingSeedVals );
			currentState = AligningToBlock;
			std::cout << "Exiting MovingArmToSearchPose state" << std::endl;
			break;
		}

		
		case AligningToBlock:
		{
			std::cout << std::endl;
			std::cout << "Entering AligningToBlock state" << std::endl;
			currentState = GraspingBlock;
			std::cout << "Exiting AligningToBlock state" << std::endl;
			break;
		}

		
		case GraspingBlock:
		{
			std::cout << std::endl;
			std::cout << "Entering GraspingBlock state" << std::endl;
			std::cout << "Waiting 5 seconds to allow arm to finish moving." << std::endl;
			ros::Duration(5).sleep();

			tf::Transform translate;
			translate.setIdentity();
			translate.getOrigin().setZ( translate.getOrigin().getZ() + 0.05 );

			std::cout << "Translating along search pose." << std::endl;
			pArmInterface->PositionArm( (*pGraspingTransform) * translate, graspingSeedVals );
			ros::Duration(3.0).sleep();  // Allow the arm to translate so we see it.
			currentState = PuttingArmInCarryPose;
			std::cout << "Exiting GraspingBlock state" << std::endl;
			break;
		}

		
		case PuttingArmInCarryPose:
		{
			std::cout << std::endl;
			std::cout << "Entered PuttingArmInCarryPose state" << std::endl;
			std::cout << "Driving arm to carry pose." << std::endl;
			seedVals.clear();
			for( std::size_t i = 0; i < 5; ++i )
			{
				seedVals.push_back(seedCameraSearch[i]);
			}
			pArmInterface->PositionArm( g_cameraSearch_05, seedVals );
			std::cout << "Waiting 3 seconds to allow arm to reach pose" << std::endl;
			ros::Duration(3.0).sleep();  // Allow the arm to reach the pose.
			currentState = InitiatingReturnToStart;
			navGoalStatus = 0;
			std::cout << "Exiting PuttingArmInCarryPose state" << std::endl;
			break;
		}


		case InitiatingReturnToStart:
		{
			std::cout << std::endl;
			std::cout << "Entering InitiatingReturnToStart state" << std::endl;
			std::cout << "navGoalStatus:  " << navGoalStatus << std::endl;
			std::cout << "Publishing goal and waiting for status to change" << std::endl;

			moveToAbsolutePosition(g_StartingPose_w);

			while( navGoalStatus == 3 )
			{
				ros::spinOnce();
			}

			currentState = ReturningToStart;
			std::cout << "Exiting InitiatingReturnToStart" << std::endl;
			std::cout << std::endl;
			std::cout << "Entering ReturningToStart state" << std::endl;

			break;
		}
		
		case ReturningToStart:
		{
			if( navGoalStatus == 3 )
			{
				// When we have reached the goal, transition to the next state.
				currentState = Finished;
				std::cout << "Exiting the ReturningToStart state" << std::endl;
				std::cout << std::endl;
				std::cout << "Finished!" << std::endl;
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
