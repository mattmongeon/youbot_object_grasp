#ifndef INCLUDED_BLOCK_INFO_H
#define INCLUDED_BLOCK_INFO_H


#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Point.h>
#include <tf/LinearMath/Transform.h>
#include <tf/LinearMath/Vector3.h>
#include <tf/transform_listener.h>


class cBlockInfo
{
public:

	//--------------------------------------------------------------------------//
	//----------------------  CONSTRUCTION / DESTRUCTION  ----------------------//
	//--------------------------------------------------------------------------//
	
	cBlockInfo(ros::NodeHandle& nh);

	~cBlockInfo();


	//--------------------------------------------------------------------------//
	//-------------------------  INTERFACE FUNCTIONS  --------------------------//
	//--------------------------------------------------------------------------//

	bool BlockFound() const;

	tf::Transform GetTransformA5ToBlock() const;

	const tf::Vector3& GetBlockAlignmentPosition() const;
	

private:

	//--------------------------------------------------------------------------//
	//-------------------------  SUBSCRIBER FUNCTIONS  -------------------------//
	//--------------------------------------------------------------------------//

	void BlockCallback(const geometry_msgs::Pose& pose_ASUStoBlock);
	void FloorNormalCallback( const geometry_msgs::Vector3& norm );
	void BlockAlignLocationCallback( const geometry_msgs::Point& loc );


	//--------------------------------------------------------------------------//
	//---------------------------  HELPER FUNCTIONS  ---------------------------//
	//--------------------------------------------------------------------------//

	tf::Transform GetTransformFromPose(const geometry_msgs::Pose& pose);

	
	//--------------------------------------------------------------------------//
	//-----------------------------  DATA MEMBERS  -----------------------------//
	//--------------------------------------------------------------------------//
	
	ros::Subscriber mBlockPoseSub;
	ros::Subscriber mFloorNormalSub;
	ros::Subscriber mRgbBlockLocSub;

	tf::Transform mG_A5ToAsus;
	tf::Transform mG_AsusCorrection;
	tf::Transform mG_AsusToBlock;

	tf::Vector3 mBlockAlignmentLocation;

	bool mCameraCalibrated;

	bool mBlockFound;

	tf::TransformListener* mpListener;
};

#endif  // INCLUDED_BLOCK_INFO_H

