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

    srand ( time(NULL) );

    // initialize node/node handles
    ros::init(argc, argv, "planner_node");
    ros::NodeHandle n;

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
    pair<double,double> end = {24,24};

    std::cout << map.is_valid(new Node(start.first,start.second,nullptr),{0,0}) << "\n";
    std::cout << map.is_valid(new Node(end.first,end.second,nullptr),{0,0}) << std::endl;

    vector<pair<double,double>> path = map.rrt_connect(start,end,{.1,.1},1000);

    if (path.size() == 0){
      std::cout << "No path found" << std::endl;
    } else {
      // for (auto node : path){
      //   std::cout << node.first << " " << node.second << std::endl;
      // }
    }
    


    // set publishing frequency
    ros::Rate loop_rate(20);


    int count = 0;
    while (ros::ok())
    {
        ros::spinOnce();

        loop_rate.sleep();
        ++count;
  }


  return 0;
}