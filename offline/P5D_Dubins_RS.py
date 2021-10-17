#!/usr/bin/env python3

import yaml

# add optimized_dp to python path
from optimized_dp.dynamics.DubinsCar4D2 import DubinsCar4D2
import sys
sys.path.append('./optimized_dp')

from Plots.plotting_utilities import plot_isosurface
import numpy as np
# Utility functions to initialize the problem
from Grid.GridProcessing import Grid
from Shapes.ShapesFunctions import *

# Specify the  file that includes dynamic systems
from dynamics.P5D_Dubins_Rel import *
from deriv.gradient5D import Gradient5D


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

gN = np.array([50, 50, 30, 20, 20])
gMin = np.array([-1.0, -1.0, -math.pi, 0,-8])
gMax = np.array([1.0, 1.0, math.pi, 3,8])

g = Grid(gMin, gMax, 5, gN, [2])

# Define my object
x0 = [3,3,0,0,0]

uMin=[-0.5,-6]
uMax=[0.5,6]

pMin=[-1.5]
pMax=[1.5]

dMin=[-0.02,-0.02,0,-0.02,-0.2]
dMax=[0.02,0.02,0,0.02,0.2]

v=0.1
dims=[0,1,2,3,4]
uMode="min"
dMode="max"

dynamics = P5D_Dubins_Rel(x0,uMin,uMax,pMin,pMax,dMin,dMax,v,dims,uMode,dMode)
dynamic_attributes = {"x0" : x0, "uMin" : uMin,"uMax" : uMax, "pMin" : pMin, "pMax" : pMax, "v" : v, "gN" : gN.tolist(), "gMin" : gMin.tolist(), "gMax" : gMax.tolist()}

with open("../online/src/controller/config/dynamic_attributes.yaml", "w") as fh:  
  yaml.dump(dynamic_attributes, fh)

# Use the grid to initialize initial value function
data0 = np.zeros(g.pts_each_dim)
data0 += np.power(g.vs[0],2) + np.power(g.vs[1],2)

# backward timing
lookback_length = 100
t_step = 0.1
small_number = 1e-5
tau = np.arange(start=0, stop=lookback_length + small_number, step=t_step)  # time stamps

tau = np.arange(start=0, stop=lookback_length + small_number, step=t_step)

po = PlotOptions(do_plot=False, plot_type="3d_plot", plotDims=[0,1,2],
                  slicesCut=[19])

thresh = 0.035
compMethods = { "PrevSetsMode": "minVWithVInit"}
data = HJSolver(dynamics, g, data0, tau, compMethods, po,accuracy="low",convergeThresh=thresh)

TEB = np.sqrt(np.min((data)))*(1+5*thresh)

deriv1 = hcl.asarray(np.zeros(data.shape))
deriv2 = hcl.asarray(np.zeros(data.shape))
deriv3 = hcl.asarray(np.zeros(data.shape))
deriv4 = hcl.asarray(np.zeros(data.shape))
deriv5 = hcl.asarray(np.zeros(data.shape))
computeGradient = Gradient5D(g)
computeGradient(hcl.asarray(data),deriv1,deriv2,deriv3,deriv4,deriv5)
spat_derivs = [deriv1.asnumpy(),deriv2.asnumpy(),deriv3.asnumpy(),deriv4.asnumpy(),deriv5.asnumpy()]
for i in range(len(spat_derivs)):
    filepath = "../online/src/controller/config/deriv" + str(i) + ".csv"
    np.savetxt(filepath, spat_derivs[i].T.flatten(), delimiter=",")

# print(spat_derivs[0][0,0,0,0,0])
