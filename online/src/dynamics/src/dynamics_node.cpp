#include <ros/ros.h>
#include <ros/package.h>
#include "dynamics/dynamics.hpp"
#include "nav_msgs/Path.h"
#include "std_msgs/Float64.h"
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"

static double u;

void control_callback(const std_msgs::Float64& msg){
  u = msg.data;
}

int main(int argc, char **argv)
{
    using std::string;
    using std::vector;
    using visualization_msgs::Marker;
    using visualization_msgs::MarkerArray;
    using arma::vec;
    using dynamics::Dubins;
    using dynamics::Q2D;

    ros::init(argc, argv, "dynamics_node");
    ros::NodeHandle n;

    // publishers and subscribers
    ros::Publisher pose_pub = n.advertise<MarkerArray>("/pose", 10,true);
    ros::Subscriber control_sub = n.subscribe("control", 10, control_callback);

    // ROS parameters
    vector<double> x0;
    vector<double> model_size;
    vector<double> B;
    double v;
    string frame_id;

    n.getParam("x0",x0);
    n.getParam("v",v);
    n.getParam("model_size", model_size);
    n.getParam("B", B);
    n.getParam("frame_id",frame_id);

    // initialize model
    Dubins tracker(v, x0[0],x0[1],x0[2]);
    vec state = tracker.get_state();
    u = 0;
    // Q2D planner(x0[0],x0[1]);

    // initialize messages
    MarkerArray pose_msg;
    Marker vehicle;
    Marker TEB;

    vehicle.header.frame_id = frame_id;
    vehicle.id = 0;
    vehicle.type = 1;
    vehicle.pose.position.x = x0[0];
    vehicle.pose.position.y = x0[1];
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
    int hz = 50;
    ros::Rate loop_rate(hz);


    int count = 0;
    while (ros::ok())
    {
        ros::spinOnce();

        state = tracker.dynamics(u,1./hz);

        vehicle.pose.position.x = state(0);
        vehicle.pose.position.y = state(1);
        TEB.pose = vehicle.pose;


        pose_msg.markers = {vehicle,TEB};
        pose_pub.publish(pose_msg);


        loop_rate.sleep();
        ++count;
  }
}
