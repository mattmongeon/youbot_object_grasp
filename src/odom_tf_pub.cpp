#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>

#include <iostream>
#include <string>

tf::TransformBroadcaster* odom_broadcaster;
std::string odomFrameName;
std::string baseFrameName;

void odom_callback( const nav_msgs::Odometry& odom )
{
	geometry_msgs::TransformStamped odom_trans;
	odom_trans.header.stamp = ros::Time::now();
	odom_trans.header.frame_id = odomFrameName;
	odom_trans.child_frame_id = baseFrameName;

	odom_trans.transform.translation.x = odom.pose.pose.position.x;
	odom_trans.transform.translation.y = odom.pose.pose.position.y;
	odom_trans.transform.translation.z = odom.pose.pose.position.z;

	odom_trans.transform.rotation = odom.pose.pose.orientation;

	odom_broadcaster->sendTransform(odom_trans);
}


int main( int argc, char** argv )
{
	ros::init(argc, argv, "odom_tf_pub");
	ros::NodeHandle nh("~");

	nh.param("odom_frame_id", odomFrameName, std::string("/odom"));
	nh.param("base_frame_id", baseFrameName, std::string("/base_footprint"));

	odom_broadcaster = new tf::TransformBroadcaster();
	
    ros::Subscriber blockPoseSub = nh.subscribe ("/odom", 1, odom_callback);

	ros::spin();

	delete odom_broadcaster;
	odom_broadcaster = 0;

	return 0;
}
