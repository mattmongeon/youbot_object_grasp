#ifndef INCLUDED_ARM_INTERFACE_GAZEBO_H
#define INCLUDED_ARM_INTERFACE_GAZEBO_H

#include "arm_interface.h"
#include <ros/ros.h>

class cArmInterfaceGazebo : public cArmInterface
{
public:

	//--------------------------------------------------------------------------//
	//-----------------------------  CONSTRUCTION  -----------------------------//
	//--------------------------------------------------------------------------//

	cArmInterfaceGazebo(const tf::Transform& g_arm0_to_base_link, ros::NodeHandle& nh);
	

	//--------------------------------------------------------------------------//
	//--------------------------  INTERFACE FUNCTIONS  -------------------------//
	//--------------------------------------------------------------------------//
	
    // Publishes joint values to the robot.
	//
	// Params:
	// values - a vector of joint values to be published.  The values are expected
	//          to be in the order from arm_joint_0 to arm_joint_5.
	virtual void PublishJointValues(const std::vector<double>& values);


protected:

	//--------------------------------------------------------------------------//
	//---------------------------  HELPER FUNCTIONS  ---------------------------//
	//--------------------------------------------------------------------------//
	
	// Publishes gripper position values to the robot.
	//
	// Params:
	// width - the width of each finger.
	virtual void PublishGripperValues(double width);

	
private:

	ros::Publisher mArmJoint1Pub;
	ros::Publisher mArmJoint2Pub;
	ros::Publisher mArmJoint3Pub;
	ros::Publisher mArmJoint4Pub;
	ros::Publisher mArmJoint5Pub;

	ros::Publisher mLeftGripperPub;
	ros::Publisher mRightGripperPub;
};

#endif

