#!/usr/bin/env python3

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

gN = np.array([40,30,20,20])
gMin = np.array([-0.5,-math.pi,0,-8])
gMax = np.array([0.5,math.pi,4,8])

g = Grid(np.array([-1.0, -1.0, -math.pi, 0,-8]), np.array([1.0, 1.0, math.pi, 3,8]), 5, np.array([50, 50, 30, 20, 20]), [2])

# Define my object
my_car = P5D_Dubins_Rel()

# Use the grid to initialize initial value function
data0 = np.zeros(g.pts_each_dim)
data0 += np.power(g.vs[0],2) + np.power(g.vs[1],2)

# backward timing
lookback_length = 100
t_step = 0.1
small_number = 1e-5
tau = np.arange(start=0, stop=lookback_length + small_number, step=t_step)  # time stamps

tau = np.arange(start=0, stop=lookback_length + small_number, step=t_step)

po = PlotOptions(do_plot=False, plot_type="3d_plot", plotDims=[0,1,3],
                  slicesCut=[19])

thresh = 0.01
compMethods = { "PrevSetsMode": "minVWithVInit"}
data = HJSolver(my_car, g, data0, tau, compMethods, po,accuracy="low",convergeThresh=thresh)

TEB = np.sqrt(np.min((data)))*(1+5*thresh)
print(TEB)