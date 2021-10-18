#include "dynamics/dynamics.hpp"
#include <cmath>

namespace dynamics{

    using arma::vec;
    using std::vector;
    using std::cout;
    using std::endl;


    /**********************
     * Dubins Functions
     *********************/

    Dubins::Dubins(double v, double x, double y, double theta) : v(v), x(x), y(y), theta(theta) {}

    vec Dubins::dynamics(double u, double dt){
        double xdot = v*cos(theta);
        double ydot = v*sin(theta);
        double omega = u;

        x += xdot*dt;
        y += ydot*dt;
        theta += omega*dt;

        vec out = {x,y,theta};
        return out;

    }

    vec Dubins::get_state(){

        vec out = {x,y,theta};
        return out;
    }

    /*******************
     * Q2D Functions
     ******************/

    Q2D::Q2D(double x, double y) : x(x), y(y) {}

    vec Q2D::dynamics(vector<double> u, double dt){

        if (u.size() != 2){
            cout << "WARNING: input (vector<double> u) should have size 2" << std::endl;
        }

        double xdot = u[0];
        double ydot = u[1];

        x += xdot * dt;
        y += ydot * dt;

        vec out = {x,y};
        return out;

    }

    vec Q2D::get_state(){
        vec out = {x,y};
        return out;
    }



}