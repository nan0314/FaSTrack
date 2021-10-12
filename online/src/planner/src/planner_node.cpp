#include <ros/ros.h>
#include <ros/package.h>
#include "planner/planner.hpp"

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

    // initialize node/node handles
    ros::init(argc, argv, "planner_node");
    ros::NodeHandle n;

    // publishers and subscribers
    ros::Publisher map_pub = n.advertise<OccupancyGrid>("/map", 10,true);
    ros::Publisher path_pub = n.advertise<Path>("/path", 10,true);

    // Ros parameters
    double w;
    double h;
    double dl;
    n.getParam("width",w);
    n.getParam("height",h);
    n.getParam("dl",dl);

    string filepath = ros::package::getPath("planner") + "/config/map.csv";

    arma::mat test(500,500,arma::fill::zeros);
    Map map(filepath,w,h,dl);

    pair<double,double> start = {3,3};
    pair<double,double> end = {25,25};

    if (!map.is_valid(new Node(start.first,start.second,nullptr),{0,0})){
      ROS_WARN_STREAM("START LOCATION IS INVALID!");
    }
    if (!map.is_valid(new Node(end.first,end.second,nullptr),{0,0})){
      ROS_WARN_STREAM("END LOCATION IS INVALID!");
    }

    vector<pair<double,double>> path = map.rrt_connect(start,end,{.1,.1},1000);
    OccupancyGrid map_msg = map.get_grid("map");
    Path path_msg = nav_path(path,"map");

    if (path.size() == 0){
      ROS_WARN_STREAM("NO PATH FOUND");
    }
  
    // set publishing frequency
    ros::Rate loop_rate(20);


    int count = 0;
    while (ros::ok())
    {
        ros::spinOnce();

        map_pub.publish(map_msg);
        path_pub.publish(path_msg);

        loop_rate.sleep();
        ++count;
  }


  return 0;
}