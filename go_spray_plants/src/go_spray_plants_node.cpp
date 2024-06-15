#include <array>
#include <utility>
#include <ros/ros.h>
#include <memory>
#include <visualization_msgs/Marker.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_listener.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <actionlib/client/simple_action_client.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "go_spray_plants_node");
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);

    ros::NodeHandle n;
    ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("goal_pose_marker", 0);
    visualization_msgs::Marker target;

    geometry_msgs::PointStamped::ConstPtr pos = ros::topic::waitForMessage<geometry_msgs::PointStamped>("/the_target_pos", n);

    if (!pos) return 1;
	
    while (marker_pub.getNumSubscribers() < 1) {
	ROS_WARN("Please create a subscriber to the marker");
	sleep(1);
    }
    
    try {
	geometry_msgs::PointStamped marker_pos = tfBuffer.transform(*pos, "odom", ros::Duration(3.0));

	visualization_msgs::Marker marker;
	marker.header.frame_id = marker_pos.header.frame_id;
	marker.header.stamp = marker_pos.header.stamp;

	marker.ns = "go_spray_plants";
    
	marker.type = visualization_msgs::Marker::ARROW;
	marker.action = visualization_msgs::Marker::ADD;
	
	marker.pose.position = marker_pos.point;
	marker.pose.orientation.w = 1.0;

	marker.scale.x = 0.1;
	marker.scale.y = 0.1;
	marker.scale.z = 0.1;

	marker.color.r = 1.0f;
	marker.color.g = 0.0f;
	marker.color.b = 0.0f;
	marker.color.a = 1.0;

	marker.lifetime = ros::Duration();
	marker_pub.publish(marker);


    } catch (tf2::TransformException &ex) {
	ROS_WARN("Could NOT transform pos to odom: %s", ex.what());
	return 2;
    }
    
    // Tell the action client that we want to spin a thread by default
    MoveBaseClient ac("move_base", true);

    // Wait for the action server to come up
    while(!ac.waitForServer(ros::Duration(5.0))){
	ROS_INFO("Waiting for the move_base action server to come up");
    }

    move_base_msgs::MoveBaseGoal goal;

    try { 
	geometry_msgs::PointStamped target_pos = tfBuffer.transform(*pos, "map");
	goal.target_pose.header.frame_id = target_pos.header.frame_id;
	goal.target_pose.header.stamp = target_pos.header.stamp;

	goal.target_pose.pose.position = target_pos.point;
	goal.target_pose.pose.orientation.w = 1.0;

	ROS_INFO_STREAM("Sending goal: " << goal.target_pose.pose);

	ac.sendGoal(goal);
	ac.waitForResult();

	if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
	    ROS_INFO("The base moved to pose :)");
	else
	    ROS_INFO("The base failed to move for some reason, check logs :(");

    } catch (tf2::TransformException &ex) {
	ROS_WARN("Could NOT transform pos to map: %s", ex.what());
	return 3; 
    }
    
    return 0;
}
