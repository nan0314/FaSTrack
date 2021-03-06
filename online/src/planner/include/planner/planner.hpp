#ifndef REACH_PLANNER_INCLUDE_GUARD_HPP
#define REACH_PLANNER_INCLUDE_GUARD_HPP

#include "geometry_msgs/PoseStamped.h"
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/Path.h"
#include <armadillo>
#include <cmath>
#include <string>

namespace planner{

    using arma::mat;
    using std::string;
    using std::pair;
    using std::vector;
    using nav_msgs::OccupancyGrid;
    using nav_msgs::Path;


    bool is_equal(const double &r, const double &l);

    Path nav_path(const vector<pair<double,double>> &path, const std::string &frame_id);

    vector<pair<double,double>> interpolate(const double &x0, const double &y0,
                                                const double &x1, const double &y1, const double &epsilon);


    struct Node{
        double x,y;
        Node* parent;

        Node();

        Node(double x, double y, Node* parent);

        double distance(Node *q);

        vector<pair<double,double>> get_path();

    };

    class Map{

        double width, height, resolution;
        mat grid;

        Node random_config();

        
        
        public:

        OccupancyGrid get_grid(string frame_id);

        bool new_config(Node* q, Node* q_near, Node* &q_new, const vector<double> &TEB);

        bool is_valid(const double &x, const double &y, const vector<double> &TEB);

        string extend(vector<Node*> &tree, Node* q, Node* &q_new, const vector<double> &TEB);

        string connect(vector<Node*> &tree, Node* q, const vector<double> &TEB);

        Node* nearest_neighbor(vector<Node*> &tree, Node* q);

        Map(mat grid, double w, double h, double resolution);

        Map(string filepath, double w, double h, double resolution);

        Map(const OccupancyGrid &map);

        mat get_grid();

        vector<pair<double,double>> rrt_connect(pair<double,double> start, pair<double,double> end, vector<double> TEB, int K);

        vector<pair<double,double>> smooth(const vector<pair<double,double>> &path,const vector<double> &TEB);

        bool valid_path(const vector<pair<double,double>> &path, const vector<double> &TEB);
    };



}


#endif
