#include "controller/P5Dubins.hpp"
#include <cmath>

// P5Dubins Functions

namespace P5Dubins{

    using arma::vec;
    using std::vector;
    using std::list;
    using std::string;
    using std::pair;
    using Eigen::VectorXd;

    P5Dubins::P5Dubins(vec s, double dt, vector<double> uMin, vector<double> uMax, vector<int> gN, vector<double> gMin, vector<double> gMax, string filename) : 
        s(s), dt(dt), uMin(uMin), uMax(uMax), count(0), gN(gN), gMin(gMin), gMax(gMax), filename(filename) {

        for (int i = 0; i < gN.size(); i++){
            gN[i] -= 1;
        }

        Q = { {1, 0, 0},
              {0, 1, 0},
              {0, 0, 1},
              {0, 0, 0},
              {0, 0, 0}};

    }

    void P5Dubins::update_trajectory(Trajectory trajectory){
        double duration = trajectory.getDuration();
        pair<double,double> point;
		for(double t = 0.0; t < duration; t += 0.1) {
            point = {trajectory.getPosition(t)[0], trajectory.getPosition(t)[1]};
			traj.push_back(point);
		}
        point = {trajectory.getPosition(duration)[0], trajectory.getPosition(duration)[1]};
        traj.push_back(point);
        count = 0;
    };

    vec P5Dubins::p_next(){

        double x_prev = traj[count].first;
        double y_prev = traj[count].first;

        ++count;

        double x = traj[count].first;
        double y = traj[count].second;

        // calculate theta
        double theta = atan2(y - y_prev,x - x_prev);

        vec out = {x,y,theta};

        return out;
    }

    vec P5Dubins::uOpt(vec p_next){
        vec u = {0,0,0,uMax[0],uMax[1]};

        // calculate r
        
        arma::mat phi = { {cos(p_next(2)), sin(p_next(2)), 0, 0, 0},
                          {-sin(p_next(2)), cos(p_next(2)), 0, 0, 0},
                          {0, 0, 1, 0, 0},
                          {0, 0, 0, 1, 0},
                          {0, 0, 0, 0, 1}};

        vec r = phi*(s - Q*p_next);
        int r0 = round(((r(0) - gMin[0]) / (gMax[0] - gMin[0])) * gN[0]);
        int r1 = round(((r(1) - gMin[1]) / (gMax[1] - gMin[1])) * gN[1]);
        int r2 = round(((r(2) - gMin[2]) / (gMax[2] - gMin[2])) * gN[2]);
        int r3 = round(((r(3) - gMin[3]) / (gMax[3] - gMin[3])) * gN[3]);
        int r4 = round(((r(4) - gMin[4]) / (gMax[4] - gMin[4])) * gN[4]);

        int r_index = r4*gN[3]*gN[2]*gN[1]*gN[0] + r3*gN[2]*gN[1]*gN[0] + r2*gN[1]*gN[0] + r1*gN[0] + r0;
        string path4 = filename + "3.csv";
        string path5 = filename + "4.csv";
        std::cout << path4 << std::endl;

        string num4;
        string num5;

        std::ifstream in4(path4.c_str());
        std::ifstream in5(path5.c_str());

        for(int i = 0; i < r_index; ++i){
            std::getline(in4, num4);
            std::getline(in5,num5);
        }


        std::getline(in4,num4);
        std::getline(in5,num5);

        std::cout << num4 << " " << num5 << std::endl;

        // Might have to do gN[n] - 1
        if (std::stod(num4) >= 0){
            u(3) = uMin[0];
        }

        if (std::stod(num5) >= 0){
            u(4) = uMin[1];
        }

        return u;
    }

    void P5Dubins::dynamics(vec s_dot){
        s_dot(0) = s(3)*cos(s(2));
        s_dot(1) = s(3)*sin(s(2));
        s_dot(2) = s(4);

        s += s_dot*dt;

    }

    bool P5Dubins::arrived(){
        // double duration = traj.getDuration();
        // vec goal = {traj[traj.size()-1].first,traj[traj.size()-1].second,0,0,0};

        // vec diff = s - goal;
        return count == traj.size()-1;
    }

    bool P5Dubins::step(){

        // Path planner Block
        vec p = p_next();

        // Controller Block

        vec u = uOpt(p);

        // Tracking Model Block

        dynamics(u);

        // Planning Model Block

        return arrived();
    }

    vec P5Dubins::get_state(){
        return s;
    }
}