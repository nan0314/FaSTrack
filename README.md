# Reachability

This repository contains code inspired by some of the incredible work on reachability coming out of the Berkeley Hybrid Systems Lab, in particular [FaSTrack](https://arxiv.org/abs/2102.07039) and [this paper on UAV platooning](https://arxiv.org/abs/1602.08150). I'm hoping to understand the contnet in the papers enough to replicate the results with different systems.

## Package List

This repository is split into two parts offline computations and online planning and control

Offline
- deriv - contains functions for spatial derivatives of different degree systems
- dynamics - contains dynamics for various systems for reachability analysis
Online
- TODO

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
`cd offline\optimized_dp`
`git submodule init`
`git submodule update`

The repository should be ready to use.

### Executing program

#### 3D Dubins Car tracking a 2D point planner.

Run the following command from the offline directory to verify proper installation:

`./P3D_Q2D_RS.py`

## TODO

There are opportunities several opportunities for future development.

* Offline results need to be saved in an accessible form for the online computations
* Online computations need to be added (planning and control)

## Authors

Nathaniel Nyberg

## Acknowledgments

Inspiration, code snippets, etc.

* I was inspired to do this work by the incredible work on reachability coming out of the Berkeley Hybrid Systems Lab, in particular [FaSTrack](https://arxiv.org/abs/2102.07039) and [this paper on UAV platooning](https://arxiv.org/abs/1602.08150). 




