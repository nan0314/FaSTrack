#include <ros/ros.h>
#include <ros/package.h>
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"

int main(int argc, char **argv)
{
    using visualization_msgs::Marker;
    using visualization_msgs::MarkerArray;

    // initialize node/node handles
    ros::init(argc, argv, "controller_node");
    ros::NodeHandle n;

    // publishers and subscribers
    ros::Publisher pose_pub = n.advertise<MarkerArray>("/pose", 10,true);

    // ROS parameters


    // initialize messages
    MarkerArray pose_msg;
    Marker vehicle;
    Marker TEB;

    vehicle.header.frame_id = "map";
    vehicle.id = 0;
    vehicle.type = 1;
    vehicle.pose.position.x = 3;
    vehicle.pose.position.y = 3;
    vehicle.scale.x = 0.05;
    vehicle.scale.y = 0.05;
    vehicle.color.b = 1;
    vehicle.color.a = 1;

    TEB.header.frame_id = "map";
    TEB.id = 1;
    TEB.type = 1;
    TEB.pose = vehicle.pose;
    TEB.scale.x = 0.1;
    TEB.scale.y = 0.1;
    TEB.color.r = 1;
    TEB.color.a = 0.5;
    
    // set publishing frequency
    ros::Rate loop_rate(20);


    int count = 0;
    while (ros::ok())
    {
        ros::spinOnce();

        pose_msg.markers = {vehicle,TEB};
        pose_pub.publish(pose_msg);


        loop_rate.sleep();
        ++count;
  }


  return 0;
}