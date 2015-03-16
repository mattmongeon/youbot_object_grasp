#ifndef INCLUDED_ARM_INTERFACE_YOUBOT_H
#define INCLUDED_ARM_INTERFACE_YOUBOT_H

#include "arm_interface.h"
#include <ros/ros.h>

class cArmInterfaceYoubot : public cArmInterface
{
public:

	//--------------------------------------------------------------------------//
	//-----------------------------  CONSTRUCTION  -----------------------------//
	//--------------------------------------------------------------------------//

	cArmInterfaceYoubot(ros::NodeHandle& nh);
	

	//--------------------------------------------------------------------------//
	//--------------------------  INTERFACE FUNCTIONS  -------------------------//
	//--------------------------------------------------------------------------//
	
    // Publishes joint values to the robot.
	//
	// Params:
	// values - a vector of joint values to be published.  The values are expected
	//          to be in the order from arm_joint_0 to arm_joint_5.
	virtual void PublishJointValues(const std::vector<double>& values);

	// Publishes gripper position values to the robot.
	//
	// Params:
	// width - the width of each finger.
	virtual void PublishGripperValues(double width);


private:

	ros::Publisher mArmJointsPub;
	ros::Publisher mGripperPub;
	
};


#endif

