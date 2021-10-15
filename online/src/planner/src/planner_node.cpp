#include <ros/ros.h>
#include <ros/package.h>
#include "planner/planner.hpp"
#include "geometry_msgs/Pose.h"
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"

static nav_msgs::OccupancyGrid grid;
static bool map_recieved = false;

static geometry_msgs::Pose pose;

void map_callback(const nav_msgs::OccupancyGrid& msg){

    grid = msg;
    map_recieved = true;

}

void pose_callback(const visualization_msgs::MarkerArray &msg){

    pose = msg.markers[0].pose;
}


int main(int argc, char **argv)
{
    using std::string;
    using std::pair;
    using std::vector;
    using planner::Map;
    using planner::Node;
    using nav_msgs::OccupancyGrid;
    using nav_msgs::Path;
    using planner::nav_path;

    srand ( time(NULL) );

    // initialize node/node hanresolutiones
    ros::init(argc, argv, "planner_node");
    ros::NodeHandle n;

    // publishers and subscribers
    ros::Publisher path_pub = n.advertise<Path>("/path", 10,true);
    ros::Subscriber map_sub = n.subscribe("map", 10, map_callback);
    ros::Subscriber pose_sub = n.subscribe("pose",10,pose_callback);

    // ROS parameters
    double w;
    double h;
    double resolution;
    vector<double> goal;
    vector<double> model_size;
    vector<double> B;
    string frame_id;

    n.getParam("width",w);
    n.getParam("height",h);
    n.getParam("resolution",resolution);
    n.getParam("goal",goal);
    n.getParam("model_size", model_size);
    n.getParam("B", B);
    n.getParam("frame_id",frame_id);

    arma::mat init(w,h,arma::fill::zeros);
    Map map(init,w,h,resolution);
    
    pair<double,double> end = {goal[0],goal[1]};
    vector<pair<double,double>> path;
    vector<double> TEB;
    Path path_msg;

    for (int i = 0; i<model_size.size(); i++){
      TEB.push_back(model_size[i] + B[i]);
    }
    
  
    // set publishing frequency
    ros::Rate loop_rate(20);


    while (ros::ok())
    {
        ros::spinOnce();

        if (map_recieved){

          map = Map(grid);
          map_recieved = false;

          // make new path if path is empty or invalid
          if (path.size() == 0 or !map.valid_path(path,TEB)){

            if (!map.is_valid(pose.position.x,pose.position.y,{0,0})){
              ROS_WARN_STREAM("START LOCATION IS INVALID!");
            }
            if (!map.is_valid(end.first,end.second,{0,0})){
              ROS_WARN_STREAM("END LOCATION IS INVALID!");
            }

            path = map.rrt_connect({pose.position.x,pose.position.y},end,TEB,5000);
            path_msg = nav_path(path,frame_id);

            if (path.size() == 0){
              ROS_WARN_STREAM("NO PATH FOUND");
            }
          }

        }

        path_pub.publish(path_msg);

        loop_rate.sleep();
  }


  return 0;
}