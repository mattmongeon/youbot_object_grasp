#ifndef INCLUDED_BASE_CONTROL_H
#define INCLUDED_BASE_CONTROL_H

#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <actionlib_msgs/GoalStatusArray.h>
#include <tf/LinearMath/Transform.h>
#include <tf/transform_listener.h>
#include <string>


class cBaseControl
{
public:

	//--------------------------------------------------------------------------//
	//----------------------  CONSTRUCTION / DESTRUCTION  ----------------------//
	//--------------------------------------------------------------------------//

	cBaseControl(ros::NodeHandle& nh);

	~cBaseControl();


	//--------------------------------------------------------------------------//
	//-------------------------  INTERFACE FUNCTIONS  --------------------------//
	//--------------------------------------------------------------------------//

	void CommandBaseVelocity( double xVel, double yVel, double thetaVel );
	void CommandBaseVelocity(const geometry_msgs::Twist& twist);

	void MoveRelativeToArmLink0( double x, double y, double theta );
	void MoveRelativeToArmLink0( const tf::Transform& g );

	void MoveToWorldPosition( double x, double y, double theta );
	void MoveToWorldPosition( const tf::Transform& g );
	
	bool MovingToMoveBaseGoal() const;

	const tf::Transform& GetCurrentWorldPosition() const;
	

private:

	//--------------------------------------------------------------------------//
	//-------------------------  SUBSCRIBER FUNCTIONS  -------------------------//
	//--------------------------------------------------------------------------//

	void OdomCallback(const nav_msgs::Odometry& odom);
	void MoveBaseStatusCallback( const actionlib_msgs::GoalStatusArray& status );


	//--------------------------------------------------------------------------//
	//-----------------------------  DATA MEMBERS  -----------------------------//
	//--------------------------------------------------------------------------//
	
	ros::Publisher mCmdVelPub;
	ros::Publisher mMoveBaseGoalPub;

	ros::Subscriber mOdomSub;
	ros::Subscriber mMoveBaseGoalStatusSub;

	bool mMoveBaseGoalActive;
	int mNavGoalStatus;

	tf::TransformListener* mpListener;

	std::string mOdomFrameName;
	std::string mGlobalFrameName;

	tf::Transform mCurrentOdom;
};


#endif  // INCLUDED_BASE_CONTROL_H

