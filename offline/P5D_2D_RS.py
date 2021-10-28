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
from dynamics.P5D_2D_Rel import *
from deriv.gradient5D import Gradient5D

# Plot options
from plot_options import *

from mpl_toolkits.mplot3d import Axes3D
from matplotlib import cm
import matplotlib.pyplot as plt
from proj import *

# Solver core
from solver import HJSolver
import math


## Grid and Cost
gN = [20, 20, 15, 10, 15]  # number of grid points (more grid points --> better results,
#    but slower computation)
gMin = [-0.35, -0.35, -np.pi, -0.25, -3.75]
gMax = [ 0.35,  0.35,  np.pi,  0.25,  3.75]

# create grid with 3rd dimension periodic
g = Grid(np.array(gMin), np.array(gMax), len(gN), np.array(gN), [2])

# define cost (aka target) function l(x)
# cost is distance to origin (quadratic because derivatives are smoother)
target = np.zeros(g.pts_each_dim)
target += np.power(g.vs[0],2) + np.power(g.vs[1],2)

## dynamic system

# initial state
x0=[0,0,0,0,0]

# acceleration bounds
aRange = [-0.15,0.15]

# angular acceleration bounds
alphaMax = 3

# planning bounds
pMax  = [0.25,0.25]

# disturbance bounds
dMax = [0.02, 0.02, 0, 0.02, 0.02]

# create relative dynamics
dynamics = P5D_2D_Rel()
dynamic_attributes = { "aRange" : aRange, "alphaMax" : alphaMax, "pMax" : pMax}
with open("../online/src/dynamics/config/dynamic_params.yaml", "w") as fh:  
  yaml.dump(dynamic_attributes, fh)

grid_attributes = {"gN" : gN, "gMin" : gMin, "gMax" : gMax}
with open("../online/src/controller/config/grid_params.yaml", "w") as fh:  
  yaml.dump(grid_attributes, fh)

## Other Parameters

# backward timing
lookback_length = 25
t_step = 0.1
small_number = 1e-5
tau = np.arange(start=0, stop=lookback_length + small_number, step=t_step)  # time stamps

po2 = PlotOptions(do_plot=False, plot_type="3d_plot", plotDims=[0,1,2],
                  slicesCut=[])

# Converge Parameters
thresh = 0.05

# in FaSTrack we want to take the max with the cost function l(x) over time
compMethods = { "PrevSetsMode": "maxVWithV0"}
# HJSolver(dynamics object, grid, initial value function, time length, system objectives, plotting options)
data = HJSolver(dynamics, g, target, tau, compMethods, po2, accuracy="low", convergeThresh=thresh) # The main function to run reachability code


# Get TEB
TEB = np.min(np.sqrt(data)) * (1 + 3*thresh)
print(TEB)

deriv1 = hcl.asarray(np.zeros(data.shape))
deriv2 = hcl.asarray(np.zeros(data.shape))
deriv3 = hcl.asarray(np.zeros(data.shape))
deriv4 = hcl.asarray(np.zeros(data.shape))
deriv5 = hcl.asarray(np.zeros(data.shape))
computeGradient = Gradient5D(g)
computeGradient(hcl.asarray(data),deriv1,deriv2,deriv3,deriv4,deriv5)
spat_derivs = [deriv1.asnumpy(),deriv2.asnumpy(),deriv3.asnumpy(),deriv4.asnumpy(),deriv5.asnumpy()]
# print(np.max(spat_derivs[4]))

deriv_dict = {"deriv3" : spat_derivs[3].flatten().tolist()}
with open("../online/src/controller/config/spat_derivs3.yaml", "w") as fh:  
  yaml.dump(deriv_dict, fh)

deriv_dict = {"deriv4" : spat_derivs[4].flatten().tolist()}
with open("../online/src/controller/config/spat_derivs4.yaml", "w") as fh:  
  yaml.dump(deriv_dict, fh)

# for i in range(len(spat_derivs)):
#     filepath = "../online/src/controller/config/deriv" + str(i) + ".csv"
#     np.savetxt(filepath, spat_derivs[i].T.flatten(), delimiter=",")

