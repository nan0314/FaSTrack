#!/usr/bin/env python3

import yaml

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
gN = np.array([91,91,51])   # number of grid points (more grid points --> better results,
#    but slower computation)
gMin = np.array([-2, -2, -3.2])
gMax = np.array([2, 2, 3.2])

# create grid with 3rd dimension periodic
g = Grid(gMin, gMax, gMin.size, gN, [2])
g0 = g.vs[0][:,0,0]
g1 = g.vs[1][0,:,0]
g2 = g.vs[2][0,0,:]


# define cost (aka target) function l(x)
# cost is distance to origin (quadratic because derivatives are smoother)
target = np.zeros(g.pts_each_dim)
target += np.sqrt(np.power(g.vs[0],2) + np.power(g.vs[1],2))

# visualize cost function
[g2D, data02D] = proj(g,target,[0,0,1],'min')
fig = plt.figure()
ax = fig.gca(projection='3d')
surf = ax.plot_surface(g2D.vs[0], g2D.vs[1], np.sqrt(data02D), rstride=1, cstride=1, antialiased=True)
# plt.show()

## dynamic system

# tracker control bounds
uMin = -2.
uMax = 2.

# planner control bounds
pMax = [0.2828,0.2828]
pMin = [-.2828,-0.2828]

# disturbance control bounds
dMax = [0,0]
dMin = [0,0]

# initial state
x0 = [0,0,0]

# constant velocity
v = 0.405

# create relative dynamics
dynamics = P3D_Q2D_Rel(x0,uMin,uMax,pMin,pMax,dMin,dMax,v)
dynamic_attributes = {"v" : v, "uMin" : [uMin],"uMax" : [uMax], "pMin" : pMin, "pMax" : pMax}
with open("../online/src/dynamics/config/P3D_Q2D/dynamic_params.yaml", "w") as fh:  
  yaml.dump(dynamic_attributes, fh)
grid_attributes = {"gN" : gN.tolist(), "gMin" : gMin.tolist(), "gMax" : gMax.tolist()}
with open("../online/src/controller/config/P3D_Q2D/grid_params.yaml", "w") as fh:  
  yaml.dump(grid_attributes, fh)
## Other Parameters

# backward timing
lookback_length = 100
t_step = 0.01
small_number = 1e-5
tau = np.arange(start=0, stop=lookback_length + small_number, step=t_step)  # time stamps

po2 = PlotOptions(do_plot=False, plot_type="3d_plot", plotDims=[0,1,2],
                  slicesCut=[])

# in FaSTrack we want to take the max with the cost function l(x) over time
compMethods = { "PrevSetsMode": "maxVWithV0"}
thresh = 0.05
# HJSolver(dynamics object, grid, initial value function, time length, system objectives, plotting options)
data = HJSolver(dynamics, g, target, tau, compMethods, po2,thresh,"medium") # The main function to run reachability code

# Plot converged cost
[g2D, data02D] = proj(g,data,[0,0,1],'min')
fig = plt.figure()
ax = fig.gca(projection='3d')
surf = ax.plot_surface(g2D.vs[0], g2D.vs[1], np.sqrt(data02D), rstride=1, cstride=1, antialiased=True)
# plt.show()

# Get TEB
TEB = np.min(data) * (1.03)
bound = np.less_equal(data,TEB)

max0 = 0
max1 = 0

for i in range(gN[0]):
  for j in range(gN[1]):
    for k in range(gN[2]):

      if (bound[i,j,k]):
        diff0 = abs(round(gN[0]/2) - i)
        diff1 = abs(round(gN[1]/2) - j)
        # diff2 = abs(round(gN[2]/2) - k)

        if diff0 > max0:
          max0 = diff0
        if diff1 > max1:
          max1 = diff1

B = [float(g0[round(gN[0]/2) + max0]), float(g1[round(gN[1]/2) + max1])]
print(B)
TEB_dict = { "B" : B}
with open("../online/src/fastrack/config/TEB_params.yaml", "w") as fh:  
  yaml.dump(TEB_dict, fh)

# deriv3 = np.genfromtxt('test3.csv', delimiter=',')
# print(deriv3[45*80*80+7*80+71])

# deriv_dict = {"deriv2" : deriv3.tolist()}
# with open("../online/src/controller/config/P3D_Q2D/spat_derivs.yaml", "w") as fh:  
#   yaml.dump(deriv_dict, fh)

deriv1 = hcl.asarray(np.zeros(data.shape))
deriv2 = hcl.asarray(np.zeros(data.shape))
deriv3 = hcl.asarray(np.zeros(data.shape))
computeGradient = Gradient3D(g)
computeGradient(hcl.asarray(data),deriv1,deriv2,deriv3)
spat_derivs = [deriv1.asnumpy(),deriv2.asnumpy(),deriv3.asnumpy()]


deriv_dict = {"deriv2" : spat_derivs[2].flatten().tolist()}
with open("../online/src/controller/config/P3D_Q2D/spat_derivs.yaml", "w") as fh:  
  yaml.dump(deriv_dict, fh)

