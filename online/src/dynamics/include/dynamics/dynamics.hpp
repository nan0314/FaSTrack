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


    class Q2D{

        double x;
        double y;

        public:

        Q2D(double x, double y);

        vec dynamics(vector<double> u, double dt);

        vec get_state();
    };

}

#endif
