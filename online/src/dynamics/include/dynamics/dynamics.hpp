#ifndef DYNAMICS_INCLUDE_GUARD_HPP
#define DYNAMICS_INCLUDE_GUARD_HPP

#include <vector>
#include <armadillo>

namespace dynamics{

    using std::vector;
    using arma::vec;

    class Dubins{

        double v;
        double x;
        double y;
        double theta;

        public:

        Dubins(double v, double x, double y, double theta);

        vec dynamics(double u, double dt);
        
        vec get_state();

    };

    class Plane_5D{

        double x;
        double y;
        double theta;
        double v;
        double omega;

        public:

        Plane_5D(const double &x, const double &y, const double &theta, const double &v, const double &omega);

        vec dynamics(vector<double> u, double dt);

        vec get_state();

    };


    class Q2D{

        double x;
        double y;

        public:

        Q2D(double x, double y);

        vec dynamics(vector<double> u, double dt);

        vec get_state();

        void set_state(vec p);
    };

}

#endif
