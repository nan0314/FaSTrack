#!/usr/bin/env python3

# add optimized_dp to python path
import sys
sys.path.append('./optimized_dp')

from Plots.plotting_utilities import plot_isosurface
import numpy as np
# Utility functions to initialize the problem
from Grid.GridProcessing import Grid
from Shapes.ShapesFunctions import *

# Specify the  file that includes dynamic systems
from dynamics.P3D_Q2D_Rel import *
from deriv.gradient3D import Gradient3D

# Plot options
from plot_options import *

from mpl_toolkits.mplot3d import Axes3D
from matplotlib import cm
import matplotlib.pyplot as plt
import matplotlib
from proj import *

# Solver core
from solver import HJSolver
import math

### \P3D_Q2D_RS
### \brief This file tests the FaSTracking precomputation of a 3D "plane" (dubins car) model tracking a 2D 
###        point mass planningmodel. It is inteded to match the work done by the Berkeley Hybrid Systems Lab, 
###        to help me better understand how to use optimized_dp to solve HJ Reachability problems. I've attempted
###        to match the syntax as much as possible.

## Grid and Cost
gN = np.array([80,80,50])   # number of grid points (more grid points --> better results,
#    but slower computation)
gMin = np.array([-3.5, -3.5, -math.pi])
gMax = np.array([4.5, 3.5, math.pi])

# create grid with 3rd dimension periodic
g = Grid(gMin, gMax, gMin.size, gN, [2])

# define cost (aka target) function l(x)
# cost is distance to origin (quadratic because derivatives are smoother)
target = np.zeros(g.pts_each_dim)
target += np.power(g.vs[0],2) + np.power(g.vs[1],2)

# visualize cost function
[g2D, data02D] = proj(g,target,[0,0,1],'min')
fig = plt.figure()
ax = fig.gca(projection='3d')
surf = ax.plot_surface(g2D.vs[0], g2D.vs[1], np.sqrt(data02D), rstride=1, cstride=1, antialiased=True)
# plt.show()

## dynamic system

# tracker control bounds
uMin = -4
uMax = 4

# planner control bounds
pMax = [0.1,0.1]
pMin = [-0.1,-0.1]

# disturbance control bounds
dMax = [0,0]
dMin = [0,0]

# initial state
x0 = [0,0,0,0]

# create relative dynamics
dynamics = P3D_Q2D_Rel(x0,uMin,uMax,pMin,pMax,dMin,dMax)

## Other Parameters

# backward timing
lookback_length = 100
t_step = 0.1
small_number = 1e-5
tau = np.arange(start=0, stop=lookback_length + small_number, step=t_step)  # time stamps

po2 = PlotOptions(do_plot=False, plot_type="3d_plot", plotDims=[0,1,2],
                  slicesCut=[])

# in FaSTrack we want to take the max with the cost function l(x) over time
compMethods = { "PrevSetsMode": "maxVWithVInit"}
# HJSolver(dynamics object, grid, initial value function, time length, system objectives, plotting options)
data = HJSolver(dynamics, g, target, tau, compMethods, po2,0.05,"medium") # The main function to run reachability code

# Plot converged cost
[g2D, data02D] = proj(g,data,[0,0,1],'min')
fig = plt.figure()
ax = fig.gca(projection='3d')
surf = ax.plot_surface(g2D.vs[0], g2D.vs[1], np.sqrt(data02D), rstride=1, cstride=1, antialiased=True)
plt.show()

# Get TEB
TEB = np.min(np.sqrt(data))+small_number
print(TEB*1.05)

print(data[0,0,0])

deriv1 = hcl.asarray(np.zeros(data.shape))
deriv2 = hcl.asarray(np.zeros(data.shape))
deriv3 = hcl.asarray(np.zeros(data.shape))
computeGradient = Gradient3D(g)
computeGradient(hcl.asarray(data),deriv1,deriv2,deriv3)
spat_derivs = [deriv1.asnumpy(),deriv2.asnumpy(),deriv3.asnumpy()]
print(spat_derivs[0][0,0,0])
# plt.show()
