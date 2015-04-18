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

	cArmInterface(const tf::Transform& g_arm0_to_base_link);


	//--------------------------------------------------------------------------//
	//--------------------------  INTERFACE FUNCTIONS  -------------------------//
	//--------------------------------------------------------------------------//
	
	// Publishes joint values to the robot.
	//
	// Params:
	// values - a vector of joint values to be published.  The values are expected
	//          to be in the order from arm_joint_0 to arm_joint_5.
	virtual void PublishJointValues(const std::vector<double>& values) = 0;
	
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

	void OpenGrippers();

	void CloseGrippers();


	//--------------------------------------------------------------------------//
	//----------------------  PRE-DEFINED POSE FUNCTIONS  ----------------------//
	//--------------------------------------------------------------------------//

	void GoToCameraSearchPose();

	void GoToRightHomePose();
	void GoToRightAlignPose();
	void GoToRightGraspPose();

	void GoToLeftHomePose();
	void GoToLeftAlignPose();
	void GoToLeftGraspPose();

	
protected:

	//--------------------------------------------------------------------------//
	//---------------------------  HELPER FUNCTIONS  ---------------------------//
	//--------------------------------------------------------------------------//
	
	// Publishes gripper position values to the robot.
	//
	// Params:
	// width - the width of each finger.
	virtual void PublishGripperValues(double width) = 0;

	
	typedef boost::shared_ptr<kinematics::KinematicsBase> KinematicsBasePtr;
    KinematicsBasePtr mpArmKinematics;

	
private:

	std::vector<double> mCameraSearchPoseSeedVals;
	tf::Transform mG_CameraSearch_05;

	std::vector<double> mArmRight90DegSeedVals;
	tf::Transform mG_RightHomePose_05;
	tf::Transform mG_RightAlignPose_05;
	tf::Transform mG_RightGraspPose_05;

	std::vector<double> mArmLeft90DegSeedVals;
	tf::Transform mG_LeftHomePose_05;
	tf::Transform mG_LeftAlignPose_05;
	tf::Transform mG_LeftGraspPose_05;

	const double mGripperWidthAtGrasp;
	const double mGripperWidthOpen;
	
	
    // Not copyable!
    cArmInterface(const cArmInterface& copyMe);
    cArmInterface& operator=(const cArmInterface& rhs);
	
};


#endif

