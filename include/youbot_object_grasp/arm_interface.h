#ifndef INCLUDED_ARM_INTERFACE_H
#define INCLUDED_ARM_INTERFACE_H

#include <moveit/kinematics_base/kinematics_base.h>
#include <tf/LinearMath/Transform.h>
#include <vector>

class cArmInterface
{
public:

	//--------------------------------------------------------------------------//
	//-----------------------------  CONSTRUCTION  -----------------------------//
	//--------------------------------------------------------------------------//

	cArmInterface();
	

	//--------------------------------------------------------------------------//
	//--------------------------  INTERFACE FUNCTIONS  -------------------------//
	//--------------------------------------------------------------------------//
	
	// Publishes joint values to the robot.
	//
	// Params:
	// values - a vector of joint values to be published.  The values are expected
	//          to be in the order from arm_joint_0 to arm_joint_5.
	virtual void PublishJointValues(const std::vector<double>& values) = 0;

	// Publishes gripper position values to the robot.
	//
	// Params:
	// width - the width of each finger.
	virtual void PublishGripperValues(double width) = 0;

	// Requests that the arm be moved to the parameter pose.  The pose should be
	// for arm_link_5 relative to arm_link_0.  If a solution is found the function
	// will publish the joint angles to the robot and return true.  If no valid
	// solution is found, no joint angles will be published and the function
	// will return false.
	//
	// Params:
	// g - the target pose transformation matrix.
	// seedvals - joint values for seeding the IK.
	//
	// Return - true if the IK was successful, false otherwise.
	virtual bool PositionArm(const tf::Transform& g, const std::vector<double>& seedVals);

protected:

	typedef boost::shared_ptr<kinematics::KinematicsBase> KinematicsBasePtr;
    KinematicsBasePtr mpArmKinematics;

private:

    // Not copyable!
    cArmInterface(const cArmInterface& copyMe);
    cArmInterface& operator=(const cArmInterface& rhs);
	
};


#endif

