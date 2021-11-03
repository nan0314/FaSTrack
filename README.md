# Fast and Safe Tracking

This repository contains code inspired by some of the incredible work on reachability coming out of the Berkeley Hybrid Systems Lab, specifically [FaSTrack](https://arxiv.org/abs/2102.07039). I'm interested in learning the theory, implementing the algorithms, and replicating their results.

## Contents

This repository is split into two parts offline computations and online planning and control

Offline
- deriv - contains functions for spatial derivatives of different degree systems
- dynamics - contains dynamics for various systems for reachability analysis
- build_map.py - Builds an obstacle map of the environment for the tracker to navigate
- P3D_Q2D_RS.py - Dubins car tracking 2D point planner reachability precomputations
- P5D_Q2D_RS.py - 5D Plane tracking 2D point planner reachability precomputations

Online
- controller - handles update of planning model and calculated control node
- dynamics - contains dynamic models library and handles updating of the tracking model
- fastrack - launch and yaml files to bring the other packages together
- mapping - handles the obstacle sensing
- planner - planning library and node to execute planning and replanning

## Getting Started

### Dependencies

* Python 3
* Numpy
* [optimized_dp](https://github.com/SFU-MARS/optimized_dp)

### Installing

These directions assume you have completed installation of Python 3 with pip and successfully set up all of the optimized_dp dependencies

* install numpy using the command `pip install numpy`
* Clone this repository into your workspace
* Run the following commands to clone the optimized_dp submodule:

```
cd offline/optimized_dp
git submodule init
git submodule update
```

The repository should be ready to use.

### Executing program

#### 3D Dubins Car tracking a 2D point planner.

Run the following command from the offline directory to verify proper installation and execute the HJ Reachability precomputations:

```
conda activate hcl-env
./P3D_Q2D_RS.py 
```

In the online directory, run the following commands to start the FaSTrack algorithm.

```
catkin_make
source devel/setup.bash
roslaunch fastrack P3D_Q2D.launch
```

An Rviz window should open with something that looks like [this](https://www.youtube.com/watch?v=r95p-M-J58g).

## TODO

There are opportunities several opportunities for future development.

* Implement other path planning algorithms and utilize more complex planner models 
* Complete implementation and debugging of 5D Plane FaSTracking (planar)
* Investigate RRT Connect Planner for possible origin connection bug
* Consider path splining (making path differentiable)
* Replan from planner position instead of tracker position

## Authors

Nathaniel Nyberg

## Acknowledgments

Inspiration, code snippets, etc.

* I was inspired to do this work by the incredible work on reachability coming out of the Berkeley Hybrid Systems Lab, in particular [FaSTrack](https://arxiv.org/abs/2102.07039)




