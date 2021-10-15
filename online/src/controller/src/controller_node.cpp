#include <ros/ros.h>
#include <ros/package.h>
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"

int main(int argc, char **argv)
{
    using std::string;
    using std::vector;
    using visualization_msgs::Marker;
    using visualization_msgs::MarkerArray;

    // initialize node/node handles
    ros::init(argc, argv, "controller_node");
    ros::NodeHandle n;

    // publishers and subscribers
    ros::Publisher pose_pub = n.advertise<MarkerArray>("/pose", 10,true);

    // ROS parameters
    vector<double> start;
    vector<double> model_size;
    vector<double> B;
    string frame_id;


    n.getParam("start", start);
    n.getParam("model_size", model_size);
    n.getParam("B", B);
    n.getParam("frame_id",frame_id);


    // initialize messages
    MarkerArray pose_msg;
    Marker vehicle;
    Marker TEB;

    vehicle.header.frame_id = frame_id;
    vehicle.id = 0;
    vehicle.type = 1;
    vehicle.pose.position.x = start[0];
    vehicle.pose.position.y = start[1];
    vehicle.scale.x = model_size[0];
    vehicle.scale.y = model_size[1];
    vehicle.scale.z = 0.05;
    vehicle.color.r = 0.0;
    vehicle.color.g = 0.0;
    vehicle.color.b = 1.0;
    vehicle.color.a = 1.0;

    TEB.header.frame_id = frame_id;
    TEB.id = 1;
    TEB.type = 1;
    TEB.pose = vehicle.pose;
    TEB.scale.x = model_size[0] + B[0];
    TEB.scale.y = model_size[1] + B[1];
    TEB.scale.z = 0.025;
    TEB.color.r = 1.0;
    TEB.color.g = 0.0;
    TEB.color.b = 0.0;
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