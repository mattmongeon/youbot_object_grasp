#include "youbot_object_grasp/base_control.h"
#include <iostream>


////////////////////////////////////////////////////////////////////////////////
//  Construction / Destruction
////////////////////////////////////////////////////////////////////////////////

cBaseControl::cBaseControl(ros::NodeHandle& nh)
{
	mCmdVelPub = nh.advertise<geometry_msgs::Twist>( "/cmd_vel", 1 );
	mMoveBaseGoalPub = nh.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 5, true);
	mMoveBaseGoalActive = false;
	mNavGoalStatus = 0;

	mOdomSub = nh.subscribe( "/odom", 1, &cBaseControl::OdomCallback, this );
	mMoveBaseGoalStatusSub = nh.subscribe( "/move_base/status", 1, &cBaseControl::MoveBaseStatusCallback, this );

	nh.param("odom_frame_id", mOdomFrameName, std::string("/odom"));
	nh.param("global_frame_id", mGlobalFrameName, std::string("odom"));

	mpListener = new tf::TransformListener();
	ros::Duration(1).sleep();

	mCurrentOdom.setIdentity();
}

////////////////////////////////////////////////////////////////////////////////

cBaseControl::~cBaseControl()
{
	delete mpListener;
	mpListener = 0;
}

////////////////////////////////////////////////////////////////////////////////
//  Interface Functions
////////////////////////////////////////////////////////////////////////////////

void cBaseControl::CommandBaseVelocity( double xVel, double yVel, double thetaVel )
{
	geometry_msgs::Twist twist;
	twist.linear.x = xVel;
	twist.linear.y = yVel;
	twist.angular.z = thetaVel;

	CommandBaseVelocity( twist );
}

////////////////////////////////////////////////////////////////////////////////

void cBaseControl::CommandBaseVelocity(const geometry_msgs::Twist& twist)
{
	if( mMoveBaseGoalActive )
		return;
	
	mCmdVelPub.publish( twist );
}

////////////////////////////////////////////////////////////////////////////////

void cBaseControl::MoveRelativeToArmLink0( double x, double y, double theta )
{
	tf::Transform g;
	g.setOrigin( tf::Vector3(x, y, 0)  );
	g.getBasis().setRPY(0, 0, theta);

	MoveRelativeToArmLink0( g );
}

////////////////////////////////////////////////////////////////////////////////

void cBaseControl::MoveRelativeToArmLink0( const tf::Transform& g )
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
	
	if( mGlobalFrameName != "odom" )
	{
		try
		{
			mpListener->lookupTransform(mGlobalFrameName, mOdomFrameName, ros::Time(0), g_mapToOdom);
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

	// The currentOdom transformation keeps track of where the base is relative to the
	// odom frame, so we need to add another transformation for base_link -> arm_link_0.
	MoveToWorldPosition( g_mapToOdom * mCurrentOdom * g );
}

////////////////////////////////////////////////////////////////////////////////

void cBaseControl::MoveToWorldPosition( double x, double y, double theta )
{
	tf::Transform g;
	g.setOrigin( tf::Vector3(x, y, 0)  );
	g.getBasis().setRPY(0, 0, theta);

	MoveToWorldPosition(g);
}

////////////////////////////////////////////////////////////////////////////////

void cBaseControl::MoveToWorldPosition( const tf::Transform& g )
{
	if( mMoveBaseGoalActive )
		return;
		
	tf::Vector3 t = g.getOrigin();
	tf::Quaternion q = g.getRotation();

	geometry_msgs::PoseStamped goalPose;
	goalPose.header.stamp = ros::Time::now();
	goalPose.header.frame_id = mGlobalFrameName;

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

	std::cout << "Publishing to /move_base_goal/simple:  t = < " << t.getX() << ", " << t.getY() << ", z >" << std::endl;
	mMoveBaseGoalPub.publish(goalPose);

	// Wait for the goal to be received and be put in action.
	std::cout << mNavGoalStatus << std::endl;
	while( mNavGoalStatus != 1 )
	{
		ros::spinOnce();
	}
	
	mMoveBaseGoalActive = true;	
}

////////////////////////////////////////////////////////////////////////////////

bool cBaseControl::MovingToMoveBaseGoal() const
{
	return mMoveBaseGoalActive;
}

////////////////////////////////////////////////////////////////////////////////

const tf::Transform& cBaseControl::GetCurrentWorldPosition() const
{
	return mCurrentOdom;
}

////////////////////////////////////////////////////////////////////////////////
//  Subscriber Functions
////////////////////////////////////////////////////////////////////////////////

void cBaseControl::OdomCallback(const nav_msgs::Odometry& odom)
{
	tf::Vector3 t( odom.pose.pose.position.x,
				   odom.pose.pose.position.y,
				   odom.pose.pose.position.z );
	tf::Quaternion q( odom.pose.pose.orientation.x,
					  odom.pose.pose.orientation.y,
					  odom.pose.pose.orientation.z,
					  odom.pose.pose.orientation.w );
	mCurrentOdom.setOrigin( t );
	mCurrentOdom.setRotation( q );	
}

////////////////////////////////////////////////////////////////////////////////

void cBaseControl::MoveBaseStatusCallback( const actionlib_msgs::GoalStatusArray& status )
{
	if( !status.status_list.empty() )
	{
		int index = status.status_list.size() - 1;
		mNavGoalStatus = status.status_list[index].status;

		if( mMoveBaseGoalActive )
		{
			mMoveBaseGoalActive = (mNavGoalStatus != 3);
		}
	}
}
