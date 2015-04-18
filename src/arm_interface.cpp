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

cArmInterface::cArmInterface(const tf::Transform& g_arm0_to_base_link)
	:	mGripperWidthAtGrasp( 0.0025 ),
		mGripperWidthOpen( 0.0099 )
{
    pluginlib::ClassLoader<kinematics::KinematicsBase> loader("moveit_core", "kinematics::KinematicsBase");
    mpArmKinematics = loader.createInstance("youbot_arm_kinematics_moveit::KinematicsPlugin");
	mpArmKinematics->initialize("/robot_description", "arm_1", "arm_link_0", "arm_link_5", 0.1);

	
	// --- Define the Camera Search pose --- //

	tf::Matrix3x3 rot;
	tf::Vector3 t;	
	
	// This goes from base_link to arm_link_5.  It was found using rviz.
	rot.setValue( 0.0, 0.0, 1.0,
				  0.0, 1.0, 0.0,
				  -1.0, 0.0, 0.0 );
	t.setValue( 0.310676, 0.00250788, 0.351227 );
	
	mG_CameraSearch_05.setBasis(rot);
	mG_CameraSearch_05.setOrigin(t);

	// Now we will use the arm_link_0 -> base_link transformation matrix
	// to get the camera search transformation to be from arm_link_0 to 5.
	mG_CameraSearch_05 = g_arm0_to_base_link * mG_CameraSearch_05;

	mCameraSearchPoseSeedVals.push_back( 2.93215 );
	mCameraSearchPoseSeedVals.push_back( 0.25865 );
	mCameraSearchPoseSeedVals.push_back( -0.84097 );
	mCameraSearchPoseSeedVals.push_back( 2.52836 );
	mCameraSearchPoseSeedVals.push_back( 2.92343 );


	// --- Define Arm Right Poses --- //

	mArmRight90DegSeedVals.push_back( 4.56 );
	mArmRight90DegSeedVals.push_back( 2.04427 );
	mArmRight90DegSeedVals.push_back( -1.51891);
	mArmRight90DegSeedVals.push_back( 2.54343 );
	mArmRight90DegSeedVals.push_back( 2.93883 );

	// Right home
	
	tf::Quaternion q( 0.692, 0.687, -0.16, 0.154 );
	t.setValue( 0.017, -0.331, 0.059 );
	
	mG_RightHomePose_05.setRotation(q);
	mG_RightHomePose_05.setOrigin(t);

	// Alignment pose
	
	tf::Transform translate;
	translate.setIdentity();
	
	translate.getOrigin().setZ( -0.1 );
	mG_RightAlignPose_05 = mG_RightHomePose_05 * translate;

	// Grasp pose

	translate.getOrigin().setZ( 0.094 );
	mG_RightGraspPose_05 = mG_RightHomePose_05 * translate;
	

	// --- Define Arm Left Poses --- //

	mArmLeft90DegSeedVals.push_back( 1.37 );
	mArmLeft90DegSeedVals.push_back( 2.04427 );
	mArmLeft90DegSeedVals.push_back( -1.51891 );
	mArmLeft90DegSeedVals.push_back( 2.54343 );
	mArmLeft90DegSeedVals.push_back( 2.93883 );
	
	// Left home
	
	q.setValue( 0.704, -0.675, -0.158, -0.156 );
	t.setValue( 0.015, 0.331, 0.059 );

	mG_LeftHomePose_05.setRotation(q);
	mG_LeftHomePose_05.setOrigin(t);

	// Alignment Pose

	translate.getOrigin().setZ( -0.1 );
	mG_LeftAlignPose_05 = mG_LeftHomePose_05 * translate;

	// Grasp pose

	translate.getOrigin().setZ( 0.094 );
	mG_LeftGraspPose_05 = mG_LeftHomePose_05 * translate;
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
			std::cerr << "Joint " << i+1 << ":  " << solution[i] << std::endl;
		}

		PublishJointValues(solution);
	}
	else
	{
		std::cerr << "NO SOLUTION" << std::endl;
	}
}

////////////////////////////////////////////////////////////////////////////////

void cArmInterface::OpenGrippers()
{
	PublishGripperValues( mGripperWidthOpen );
}

////////////////////////////////////////////////////////////////////////////////

void cArmInterface::CloseGrippers()
{
	PublishGripperValues( mGripperWidthAtGrasp );
}

////////////////////////////////////////////////////////////////////////////////
//  Pre-Defined Pose Functions
////////////////////////////////////////////////////////////////////////////////

void cArmInterface::GoToCameraSearchPose()
{
	PositionArm( mG_CameraSearch_05, mCameraSearchPoseSeedVals );
}

////////////////////////////////////////////////////////////////////////////////

void cArmInterface::GoToRightHomePose()
{
	PositionArm( mG_RightHomePose_05, mArmRight90DegSeedVals );
}

////////////////////////////////////////////////////////////////////////////////

void cArmInterface::GoToRightAlignPose()
{
	PositionArm( mG_RightAlignPose_05, mArmRight90DegSeedVals );
}

////////////////////////////////////////////////////////////////////////////////

void cArmInterface::GoToRightGraspPose()
{
	PositionArm( mG_RightGraspPose_05, mArmRight90DegSeedVals );
}

////////////////////////////////////////////////////////////////////////////////

void cArmInterface::GoToLeftHomePose()
{
	PositionArm( mG_LeftHomePose_05, mArmLeft90DegSeedVals );
}

////////////////////////////////////////////////////////////////////////////////

void cArmInterface::GoToLeftAlignPose()
{
	PositionArm( mG_LeftAlignPose_05, mArmLeft90DegSeedVals );
}

////////////////////////////////////////////////////////////////////////////////

void cArmInterface::GoToLeftGraspPose()
{
	PositionArm( mG_LeftGraspPose_05, mArmLeft90DegSeedVals );
}
