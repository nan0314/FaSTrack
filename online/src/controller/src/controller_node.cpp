#include <ros/ros.h>
#include <ros/package.h>
#include <iostream>
#include <cstdio>
#include <Eigen/Core>
#include "controller/P5Dubins.hpp"
#include "nav_msgs/Path.h"
#include "trajectories/Path.h"
#include "trajectories/Trajectory.h"
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"

static nav_msgs::Path path;
static bool path_recieved;
static bool arrived;

void path_callback(const nav_msgs::Path& msg){

  if (path.poses.size() != msg.poses.size()){
    path = msg;
    path_recieved = true;
    arrived = false;
  } else if (msg.poses.size() > 0){
    for (int i = 0; i < msg.poses.size(); i++){
      if (msg.poses[i].pose.position.x != path.poses[i].pose.position.x){
        path = msg;
        path_recieved = true;
        arrived = false;
        break;
      }
    }
  }   

}

int main(int argc, char **argv)
{
    using std::string;
    using std::vector;
    using visualization_msgs::Marker;
    using visualization_msgs::MarkerArray;
    using arma::vec;
    using namespace std;
    using namespace Eigen;

    // initialize node/node handles
    ros::init(argc, argv, "controller_node");
    ros::NodeHandle n;

    // publishers and subscribers
    ros::Publisher pose_pub = n.advertise<MarkerArray>("/pose", 10,true);
    ros::Subscriber path_sub = n.subscribe("path", 10, path_callback);

    // ROS parameters
    vector<double> x0;
    vector<double> uMin;
    vector<double> uMax;
    vector<double> pMin;
    vector<double> pMax;
    double v;
    vector<double> gMin;
    vector<double> gMax;
    vector<int> gN;
    vector<double> model_size;
    vector<double> B;
    string frame_id;


    n.getParam("x0", x0);
    n.getParam("uMin", uMin);
    n.getParam("uMax", uMax);
    n.getParam("pMin", pMin);
    n.getParam("pMax", pMax);
    n.getParam("v", v);
    n.getParam("gN", gN);
    n.getParam("gMin",gMin);
    n.getParam("gMax",gMax);

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
    ros::Rate loop_rate(10);

    double dt = 0.1;
    arrived = true;

    list<VectorXd> waypoints;
	  VectorXd waypoint(2);
    VectorXd maxAcceleration(2);
    maxAcceleration << uMax[0], uMax[0];
    VectorXd maxVelocity(2);
    maxVelocity << v, v;

    
    string filename = ros::package::getPath("controller") + "/config/deriv";
    vec s = {x0[0],x0[1],x0[2],x0[3],x0[4]};
    P5Dubins::P5Dubins model(s, dt,uMin,uMax,gN,gMin,gMax,filename);
    // string num;
    // std::ifstream in(filename.c_str());
    // for(int i = 0; i < 30000000-1; ++i)
    //    std::getline(in, num);

    // std::getline(in,num);
    // std::cout << std::stod(num) + 1 << std::endl;

    int count = 0;
    while (ros::ok())
    {
        ros::spinOnce();
        if (path_recieved){
          // std::cout << uMax[0] << " " << v << std::endl;
          for (auto point : path.poses){
            waypoint << point.pose.position.x, point.pose.position.y;
            waypoints.push_back(waypoint);
          }
          Trajectory trajectory(Path(waypoints, 0.1), maxVelocity, maxAcceleration);
          trajectory.outputPhasePlaneTrajectory();
          if(trajectory.isValid()) {
            cout << "New trajectory generated" << endl;
            model.update_trajectory(trajectory);
          }
          else {
            cout << "Trajectory generation failed." << endl;
          }

          // save current time for trajectory timing
          count = 0;
          path_recieved = false;

        }

        if (!arrived){
          
          arrived = model.step();
          s = model.get_state();
          vehicle.pose.position.x = s[0];
          vehicle.pose.position.y = s[1];
          TEB.pose = vehicle.pose;

        }

        pose_msg.markers = {vehicle,TEB};
        pose_pub.publish(pose_msg);


        loop_rate.sleep();
        ++count;
  }


  return 0;
}