#ifndef P5DUBINS_INCLUDE_GUARD_HPP
#define P5DUBINS_INCLUDE_GUARD_HPP

#include <armadillo>
#include <vector>
#include <list>
#include <string>
#include "trajectories/Trajectory.h"
#include <Eigen/Core>



namespace P5Dubins{

    using arma::vec;
    using std::vector;
    using std::list;
    using std::pair;
    using std::string;
    using Eigen::VectorXd;

    class P5Dubins{

        vec s;
        double dt;
        vector<pair<double,double>> traj;
        int count;
        string filename;

        vector<double> uMin;
        vector<double> uMax;
        vector<int> gN;
        vector<double> gMin;
        vector<double> gMax;

        arma::mat Q;

        vec p_next();

        vec uOpt(vec p_next);

        void dynamics(vec u);

        bool arrived();

        public:

        P5Dubins(vec s,  double dt, vector<double> uMin, vector<double> uMax, vector<int> gN, vector<double> gMin, vector<double> gMax, string filepath);

        void update_trajectory(Trajectory trajectory);

        bool step();

        vec get_state();



    };
}


#endif
