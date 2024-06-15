#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf2/LinearMath/Quaternion.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <actionlib/client/simple_action_client.h>
#include <vector>
#include <array>
#include <utility>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

// {{0.8, 0.62, 0.0}, tf2::Quaternion{0.0, 0.0, 0.538, 0.84}},  // sample point
// TODO: A yaml parser that parses points, to avoid recompiling each time!
std::vector<std::pair<std::array<float, 3>, tf2::Quaternion>> positions = {
	{{1.486, 0.12998, 0.0}, tf2::Quaternion{0.0, 0.0, 0.728713, 0.684819}},
    {{1.3191378, 0.6981, 0.0}, tf2::Quaternion{0.0, 0.0, 0.999, 0.0011}},
    {{0.06104, 0.7942, 0.0}, tf2::Quaternion{0.0, 0.0, 0.7465, 0.6654}},
    {{0.06476, 1.4153, 0.0}, tf2::Quaternion{0.0, 0.0, 0.0084, 0.999}},
    {{1.38477, 1.48915, 0.0}, tf2::Quaternion{0.0, 0.0, 0.00523688, 0.998}},
    {{0.0, 0.0, 0.0}, tf2::Quaternion{0.0, 0.0, 0.0, 1.0}}

};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "agribot_inno_demo_node");
    ros::NodeHandle n;
    ros::Publisher marker_pub = n.advertise<visualization_msgs::MarkerArray>("goal_poses_marker", 0);
    visualization_msgs::MarkerArray marker_array;
    int id = 0;

   /* while (marker_pub.getNumSubscribers() < 1) {
        if (!ros::ok())
            return 0;
        ROS_WARN_ONCE("Please create a subscriber to the marker");
        sleep(1);
    }*/

    // publish a marker for eac os the positions, in positions
    for (auto& pos : positions)	{
        visualization_msgs::Marker marker;
        marker.header.frame_id = "odom";
        marker.header.stamp = ros::Time();

        marker.ns = "go_spray_plants";
        marker.id = id++;

        marker.type = visualization_msgs::Marker::ARROW;
        marker.action = visualization_msgs::Marker::ADD;

        marker.pose.position.x = pos.first[0];
        marker.pose.position.y = pos.first[1];
        marker.pose.position.z = pos.first[2];
        marker.pose.orientation = tf2::toMsg(pos.second);

        marker.scale.x = 0.3;
        marker.scale.y = 0.1;
        marker.scale.z = 0.1;

        marker.color.r = 1.0f;
        marker.color.g = 0.0f;
        marker.color.b = 0.0f;
        marker.color.a = 1.0;

        marker.lifetime = ros::Duration();
        marker_array.markers.push_back(marker);
    }

    marker_pub.publish(marker_array);

    // tell the action client that we want to spin a thread by default
    MoveBaseClient ac("move_base", true);

    //wait for the action server to come up
    while(!ac.waitForServer(ros::Duration(5.0))){
        ROS_INFO("Waiting for the move_base action server to come up");
    }

    move_base_msgs::MoveBaseGoal goal;

    // go to each of the positions in order
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();

    for (auto& pos : positions)	{
        ROS_INFO("Sending goal");

        goal.target_pose.pose.position.x = pos.first[0];
        goal.target_pose.pose.position.y = pos.first[1];
        goal.target_pose.pose.position.z = pos.first[2];
        goal.target_pose.pose.orientation = tf2::toMsg(pos.second);

        ac.sendGoal(goal);
        ac.waitForResult();

    // add python subprocess here, add in if statement below

        if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
            ROS_INFO_STREAM("The base moved to pose :)" << goal.target_pose.pose);
        else
            ROS_INFO("The base failed to move for some reason, check logs :(");

    }


    return 0;
}
