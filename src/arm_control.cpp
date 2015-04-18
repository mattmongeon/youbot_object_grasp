#include <moveit/kinematics_base/kinematics_base.h>
#include <pluginlib/class_loader.h>
#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Point.h>
#include <nav_msgs/Odometry.h>
#include <actionlib_msgs/GoalStatusArray.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>
#include <tf/LinearMath/Transform.h>
#include <tf/LinearMath/Quaternion.h>
#include <tf/LinearMath/Vector3.h>
#include <tf/LinearMath/Matrix3x3.h>
#include <tf/transform_listener.h>

#include "youbot_object_grasp/arm_interface_gazebo.h"
#include "youbot_object_grasp/arm_interface_youbot.h"
#include "youbot_object_grasp/base_control.h"
#include "youbot_object_grasp/block_info.h"

#include <iostream>
#include <vector>


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
	PrepareForDrop,
	DropBlock,
	ReturnToPickupPoint,
	MoveToFinish,
	Finished
};


int main( int argc, char** argv )
{
	ros::init(argc, argv, "arm_control");
    ros::NodeHandle nh("~");


	// --- Constants --- //

	const double adjustmentBaseSpeed = 0.0075;
	
	
	// --- Parameters --- //
	
	bool usingGazebo = false;
	ros::param::get("/using_gazebo", usingGazebo);


	// --- TF --- //
	
	tf::TransformListener* pListener = new tf::TransformListener();
	std::cout << "Wait for 2 seconds to allow tfs to buffer" << std::endl;
	ros::Duration(2).sleep();

	tf::StampedTransform g_arm0_to_base_link;
	pListener->lookupTransform("arm_link_0", "base_link", ros::Time(0), g_arm0_to_base_link);
	

	// --- Arm Interface object --- //
	
	cArmInterface* pArmInterface;
	std::cout << "Creating arm interface." << std::endl;
	if( usingGazebo )
	{
		std::cout << "\tUsing Gazebo interface" << std::endl;
		pArmInterface = new cArmInterfaceGazebo(g_arm0_to_base_link, nh);
	}
	else
	{
		std::cout << "\tUsing youBot interface" << std::endl;
		pArmInterface = new cArmInterfaceYoubot(g_arm0_to_base_link, nh);
	}


	// --- Base Controller --- //

	cBaseControl* pBaseController = new cBaseControl(nh);


	// --- Initialization --- //

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


	// --- Position Arm --- //

	std::cout << "Driving arm to camera position." << std::endl;
	pArmInterface->GoToCameraSearchPose();
	ros::Duration(3).sleep();  // Wait for the arm to get to the position.


	// --- Block Info --- //
	
	cBlockInfo* pBlockInfo = new cBlockInfo(nh);


	// --- Begin --- //

	bool graspingLeft = false;
	ProcessState currentState = WaitingForBlock;
	tf::Transform g_StartingPose_w;
	tf::Transform pickupGoal;
	int stopBaseCounter = 0;
	while(ros::ok())
	{
		switch(currentState)
		{
		case WaitingForBlock:
		{
			if( pBlockInfo->BlockFound() )
			{
				std::cout << "Found the block" << std::endl;
				// First let's save our starting position so we can return to it.
				g_StartingPose_w = pBaseController->GetCurrentWorldPosition();
				
				// Drive the youBot next to the block.  Make sure we don't rotate
				// based on the rotation matrix of the block itself.
				tf::StampedTransform g_arm0_to_arm5;
				pListener->lookupTransform("arm_link_0", "arm_link_5", ros::Time(0), g_arm0_to_arm5);
				pickupGoal = g_arm0_to_arm5 * pBlockInfo->GetTransformA5ToBlock();
				if( pickupGoal.getOrigin().getY() < 0.0 )
				{
					graspingLeft = false;
					pickupGoal.getOrigin().setY( pickupGoal.getOrigin().getY() + 0.3 );
				}
				else
				{
					graspingLeft = true;
					pickupGoal.getOrigin().setY( pickupGoal.getOrigin().getY() - 0.3 );
				}

				pickupGoal.setBasis( tf::Matrix3x3::getIdentity() );

				std::cout << "Publishing move goal" << std::endl;
				pBaseController->MoveRelativeToArmLink0( pickupGoal );
				currentState = NavigatingToBlock;
				std::cout << "Exiting the WaitingForBlock state" << std::endl;
				std::cout << std::endl;
				std::cout << "Entering NavigatingToBlock state" << std::endl;
			}
			break;
		}

			
		case NavigatingToBlock:
		{
			if( !pBaseController->MovingToMoveBaseGoal() )
			{
				currentState = MovingArmToSearchPose;
				std::cout << "Reached navigation goal." << std::endl;
				std::cout << "Exiting NavigatingToBlock state" << std::endl;
			}
			break;
		}

			
		case MovingArmToSearchPose:
		{
			std::cout << std::endl;
			std::cout << "Entering MovingArmToSearchPose state" << std::endl;
			std::cout << "Moving arm to search pose." << std::endl;

			if( graspingLeft )
			{
				pArmInterface->GoToLeftAlignPose();
			}
			else
			{
				pArmInterface->GoToRightAlignPose();
			}
			
			ros::Duration(4).sleep();  // Wait for the arm to get to the position.

			currentState = AligningToBlock;
			std::cout << "Exiting MovingArmToSearchPose state" << std::endl;
			break;
		}

		
		case AligningToBlock:
		{
			// Target is x: 355 +/- 10, y: 375 +/- 10
			double xCmdVel = 0;
			double yCmdVel = 0;
			const tf::Vector3& finalBlockLoc = pBlockInfo->GetBlockAlignmentPosition();
			if( finalBlockLoc.getX() > 350.0 + 2.0 )
			{
				// Move towards front of base when grasping right, opposite when left
				xCmdVel = graspingLeft ? -adjustmentBaseSpeed : adjustmentBaseSpeed;
			}
			else if( finalBlockLoc.getX() < 350.0 - 2.0 )
			{
				// Move towards rear of base when grasping right, opposite when left
				xCmdVel = graspingLeft ? adjustmentBaseSpeed : -adjustmentBaseSpeed;
			}

			if( finalBlockLoc.getY() > 415.0 + 10.0 )
			{
				// Move to the left when grasping right, opposite when left
				yCmdVel = graspingLeft ? adjustmentBaseSpeed : -adjustmentBaseSpeed;
			}
			else if( finalBlockLoc.getY() < 415.0 - 10.0 )
			{
				// Move to the right when grasping right, opposite when left
				yCmdVel = graspingLeft ? -adjustmentBaseSpeed : adjustmentBaseSpeed;
			}

			bool adjustmentMade = (xCmdVel != 0) || (yCmdVel != 0);
			
			// If no adjustments were made, this should be all zeros which should stop the base.
			pBaseController->CommandBaseVelocity(xCmdVel, yCmdVel, 0);

			// This weird counter is here to force the base to stop.  Due to the face that things
			// are happening asynchronously and the base might still be moving when we send this
			// command, we are going to have a counter that makes sure we don't drift away from
			// our target by accident while we are trying to stop.
			if( !adjustmentMade && (stopBaseCounter < 10) )
			{
				// TODO:  Use a timer instead of just a simple counter.
				++stopBaseCounter;
				currentState = GraspingBlock;
				std::cout << "Exiting AligningToBlock state" << std::endl;
			}
			else
			{
				stopBaseCounter = 0;
			}
			
			break;
		}

		
		case GraspingBlock:
		{
			std::cout << std::endl;
			std::cout << "Entering GraspingBlock state" << std::endl;
			std::cout << "Waiting 2 seconds to allow arm to finish moving." << std::endl;
			ros::Duration(2).sleep();

			std::cout << "Opening grippers" << std::endl;
			pArmInterface->OpenGrippers();
			std::cout << "Waiting 4 seconds to allow grippers to open." << std::endl;
			ros::Duration(4).sleep();


			// --- Establish Goal Position --- //

			std::cout << "Reaching to grasp." << std::endl;
			if( graspingLeft )
			{
				pArmInterface->GoToLeftGraspPose();
			}
			else
			{
				pArmInterface->GoToRightGraspPose();
			}
			std::cout << "Waiting 3 seconds to allow arm to finish moving." << std::endl;
			ros::Duration(3.0).sleep();

			std::cout << "Closing grippers" << std::endl;
			pArmInterface->CloseGrippers();
			std::cout << "Waiting 4 seconds to allow grippers to close." << std::endl;
			ros::Duration(4).sleep();
			
			currentState = PuttingArmInCarryPose;
			std::cout << "Exiting GraspingBlock state" << std::endl;
			break;
		}

		
		case PuttingArmInCarryPose:
		{
			std::cout << std::endl;
			std::cout << "Entered PuttingArmInCarryPose state" << std::endl;
			std::cout << "Putting arm back into search pose" << std::endl;
			if( graspingLeft )
			{
				pArmInterface->GoToLeftHomePose();
			}
			else
			{
				pArmInterface->GoToRightHomePose();
			}

			std::cout << "Waiting 3 seconds to allow arm to finish moving." << std::endl;
			ros::Duration(3.0).sleep();
			
			
			std::cout << "Driving arm to carry pose." << std::endl;
			pArmInterface->GoToCameraSearchPose();
			std::cout << "Waiting 3 seconds to allow arm to reach pose" << std::endl;
			ros::Duration(3.0).sleep();  // Allow the arm to reach the pose.

			currentState = InitiatingReturnToStart;
			std::cout << "Exiting PuttingArmInCarryPose state" << std::endl;
			break;
		}


		case InitiatingReturnToStart:
		{
			std::cout << std::endl;
			std::cout << "Entering InitiatingReturnToStart state" << std::endl;
			std::cout << "Publishing goal and waiting for status to change" << std::endl;

			pBaseController->MoveToWorldPosition( g_StartingPose_w );

			currentState = ReturningToStart;
			std::cout << "Exiting InitiatingReturnToStart" << std::endl;
			std::cout << std::endl;
			std::cout << "Entering ReturningToStart state" << std::endl;

			break;
		}

		
		case ReturningToStart:
		{
			if( !pBaseController->MovingToMoveBaseGoal() )
			{
				currentState = PrepareForDrop;
				std::cout << "Exiting the ReturningToStart state" << std::endl;
				std::cout << "Entering PrepareForDrop state" << std::endl;
				std::cout << std::endl;
			}
			break;
		}

		
		case PrepareForDrop:
		{
			std::cout << "Putting arm into drop pose." << std::endl;
			if( graspingLeft )
			{
				pArmInterface->GoToLeftGraspPose();
			}
			else
			{
				pArmInterface->GoToRightGraspPose();
			}
			
			std::cout << "Waiting 3 seconds to allow arm to finish moving." << std::endl;
			ros::Duration(3).sleep();  // Wait for the arm to get to the position.

			std::cout << "Exiting PrepareForDrop state" << std::endl;
			std::cout << "Entering DropBlock state" << std::endl << std::endl;
			currentState = DropBlock;
			break;
		}

		
		case DropBlock:
		{
			std::cout << "Opening grippers" << std::endl;
			pArmInterface->OpenGrippers();
			std::cout << "Waiting 2 seconds to allow grippers to open." << std::endl;
			ros::Duration(2).sleep();			

			std::cout << "Moving arm back to get out of the way." << std::endl;
			if( graspingLeft )
			{
				pArmInterface->GoToLeftAlignPose();
			}
			else
			{
				pArmInterface->GoToRightAlignPose();
			}
			ros::Duration(2).sleep();			
			
			std::cout << "Driving arm to carry pose." << std::endl;
			pArmInterface->GoToCameraSearchPose();
			ros::Duration(2.0).sleep();  // Allow the arm to reach the pose.

			pArmInterface->CloseGrippers();
			ros::Duration(1.0).sleep();  // Allow the arm to reach the pose.

			std::cout << "Publishing nav goal to go back to the pickup location" << std::endl;
			tf::Transform rotate;
			rotate.setIdentity();
			rotate.getBasis().setRPY(0, 0, 3.14);
			pBaseController->MoveRelativeToArmLink0( pickupGoal * rotate );

			currentState = MoveToFinish;
			std::cout << "Exiting DropBlock state" << std::endl;
			std::cout << "Entering ReturnToPickupPoint state" << std::endl;
			std::cout << std::endl;
			break;
		}

		
		case ReturnToPickupPoint:
		{
			if( !pBaseController->MovingToMoveBaseGoal() )
			{
				std::cout << "Exiting ReturnToPickupPoint state" << std::endl;
				currentState = Finished;
			}
			
			break;
		}
		

		default:
			break;
		}

		ros::spinOnce();
	}

	delete pListener;
	pListener = 0;

	delete pArmInterface;
	pArmInterface = 0;
	
	delete pBaseController;
	pBaseController = 0;
	
	return 0;
}

