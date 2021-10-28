#include <ros/ros.h>
#include <ros/package.h>
#include "dynamics/dynamics.hpp"
#include "nav_msgs/Path.h"
#include "std_msgs/Float64MultiArray.h"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"

static double u;
static bool control_recieved;

void control_callback(const std_msgs::Float64MultiArray& msg){
  u = msg.data[0];
  if (fabs(u) > 1e-5){
      control_recieved = true;
  }
}

int main(int argc, char **argv)
{
    using std::string;
    using std::vector;
    using visualization_msgs::Marker;
    using visualization_msgs::MarkerArray;
    using arma::vec;
    using dynamics::Plane_5D;
    using dynamics::Q2D;
    using dynamics::Dubins;

    ros::init(argc, argv, "dynamics_node");
    ros::NodeHandle n;

    // publishers and subscribers
    ros::Publisher pose_pub = n.advertise<Marker>("/pose", 10,true);
    ros::Subscriber control_sub = n.subscribe("control", 10, control_callback);

    // ROS parameters
    vector<double> x0;
    double v;
    string frame_id;

    n.getParam("x0",x0);
    n.getParam("frame_id",frame_id);
    n.getParam("v",v);

    // initialize model
    Dubins tracker(v,x0[0],x0[1],3.14/3);
    vec state = tracker.get_state();
    u = 0;
    control_recieved = false;

    // initialize messages
    Marker pose_msg;
    tf2::Quaternion quat;
    quat.setRPY(0,0,state(2));

    pose_msg.header.frame_id = frame_id;
    pose_msg.id = 2;
    pose_msg.type = 2;
    pose_msg.pose.position.x = state(0);
    pose_msg.pose.position.y = state(1);
    pose_msg.scale.x = 0.05;
    pose_msg.scale.y = 0.05;
    pose_msg.scale.z = 0.05;
    pose_msg.color.r = 1.0;
    pose_msg.color.g = 0.0;
    pose_msg.color.b = 1.0;
    pose_msg.color.a = 1.0;

    tf2_ros::TransformBroadcaster br;
    geometry_msgs::TransformStamped transformStamped;
    
    // set publishing frequency
    int hz = 50;
    ros::Rate loop_rate(hz);


    int count = 0;
    while (ros::ok())
    {
        ros::spinOnce();

        if (control_recieved){
            std::cout << "\nState:\n";
            for (int i = 0; i<2;i++){
                state = tracker.dynamics(u,0.01/2);
                quat.setRPY(0,0,state(2));
                control_recieved = false;
            }
            state.print();
            std::cout << std::endl;
            
        }

        transformStamped.header.stamp = ros::Time::now();
        transformStamped.header.frame_id = frame_id;
        transformStamped.child_frame_id = "model";
        transformStamped.transform.translation.x = state(0);
        transformStamped.transform.translation.y = state(1);
        transformStamped.transform.rotation.x = quat[0];
        transformStamped.transform.rotation.y = quat[1];
        transformStamped.transform.rotation.z = quat[2];
        transformStamped.transform.rotation.w = quat[3];
        br.sendTransform(transformStamped);

        pose_msg.pose.position.x = state(0);
        pose_msg.pose.position.y = state(1);
        pose_msg.pose.orientation.x = quat[0];
        pose_msg.pose.orientation.y = quat[1];
        pose_msg.pose.orientation.z = quat[2];
        pose_msg.pose.orientation.w = quat[3];

        pose_pub.publish(pose_msg);


        loop_rate.sleep();
        ++count;
  }
}
