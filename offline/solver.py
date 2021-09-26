import heterocl as hcl
import numpy as np
import time

from Plots.plotting_utilities import *
from argparse import ArgumentParser

# Backward reachable set computation library
from computeGraphs.graph_3D import *
from computeGraphs.graph_4D import *
from computeGraphs.graph_5D import *
from computeGraphs.graph_6D import *

"""
NOTE: THIS IS NOT MY ORIGINAL CODE. It was adopted from the solver from 
optimized_dp Optimal Control toolbox. I simply adjusted the solver to terminate
if converged. See the original file at 
https://github.com/SFU-MARS/optimized_dp.git in optimized_dp/solver.py
"""

def HJSolver(dynamics_obj, grid, multiple_value, tau, compMethod,
             plot_option, convergeThresh=0.1, accuracy="low"):
    print("Welcome to optimized_dp \n")
    if type(multiple_value) == list:
        init_value = multiple_value[0]
        constraint = multiple_value[1]
    else:
        init_value = multiple_value
    
    hcl.init()
    hcl.config.init_dtype = hcl.Float(32)

    ################# INITIALIZE DATA TO BE INPUT INTO EXECUTABLE ##########################

    print("Initializing\n")

    V_0 = hcl.asarray(init_value)
    V_1 = hcl.asarray(np.zeros(tuple(grid.pts_each_dim)))
    l0 = hcl.asarray(init_value)
    probe = hcl.asarray(np.zeros(tuple(grid.pts_each_dim)))

    list_x1 = np.reshape(grid.vs[0], grid.pts_each_dim[0])
    list_x2 = np.reshape(grid.vs[1], grid.pts_each_dim[1])
    list_x3 = np.reshape(grid.vs[2], grid.pts_each_dim[2])
    if grid.dims >= 4:
        list_x4 = np.reshape(grid.vs[3], grid.pts_each_dim[3])
    if grid.dims >= 5:
        list_x5 = np.reshape(grid.vs[4], grid.pts_each_dim[4])
    if grid.dims >= 6:
        list_x6 = np.reshape(grid.vs[5], grid.pts_each_dim[5])


    # Convert to hcl array type
    list_x1 = hcl.asarray(list_x1)
    list_x2 = hcl.asarray(list_x2)
    list_x3 = hcl.asarray(list_x3)
    if grid.dims >= 4:
        list_x4 = hcl.asarray(list_x4)
    if grid.dims >= 5:
        list_x5 = hcl.asarray(list_x5)
    if grid.dims >= 6:
        list_x6 = hcl.asarray(list_x6)

    # Get executable
    if grid.dims == 3:
        solve_pde = graph_3D(dynamics_obj, grid, compMethod["PrevSetsMode"], accuracy)

    if grid.dims == 4:
        solve_pde = graph_4D(dynamics_obj, grid, compMethod["PrevSetsMode"], accuracy)

    if grid.dims == 5:
        solve_pde = graph_5D(dynamics_obj, grid, compMethod["PrevSetsMode"], accuracy)

    if grid.dims == 6:
        solve_pde = graph_6D(dynamics_obj, grid, compMethod["PrevSetsMode"], accuracy)

    # Print out code for different backend
    #print(solve_pde)

    ################ USE THE EXECUTABLE ############
    # Variables used for timing
    execution_time = 0
    iter = 0
    tNow = tau[0]
    print("Started running\n")
    for i in range (1, len(tau)):
        #tNow = tau[i-1]
        t_minh= hcl.asarray(np.array((tNow, tau[i])))
        prev = V_1.asnumpy()
        while tNow <= tau[i] - 1e-4:
             tmp_arr = V_0.asnumpy()
             # Start timing
             iter += 1
             start = time.time()

             # Run the execution and pass input into graph
             if grid.dims == 3:
                solve_pde(V_1, V_0, list_x1, list_x2, list_x3, t_minh, l0)
             if grid.dims == 4:
                solve_pde(V_1, V_0, list_x1, list_x2, list_x3, list_x4, t_minh, l0, probe)
             if grid.dims == 5:
                solve_pde(V_1, V_0, list_x1, list_x2, list_x3, list_x4, list_x5 ,t_minh, l0)
             if grid.dims == 6:
                solve_pde(V_1, V_0, list_x1, list_x2, list_x3, list_x4, list_x5, list_x6, t_minh, l0)

             tNow = np.asscalar((t_minh.asnumpy())[0])

             # Calculate computation time
             execution_time += time.time() - start

             # If TargetSetMode is specified by user
             if "TargetSetMode" in compMethod:
                if compMethod["TargetSetMode"] == "max":
                    tmp_val = np.maximum(V_0.asnumpy(), constraint)
                elif compMethod["TargetSetMode"] == "min":
                    tmp_val = np.minimum(V_0.asnumpy(), constraint)
                # Update final result
                V_1 = hcl.asarray(tmp_val)

                
                # Update input for next iteration
                V_0 = hcl.asarray(tmp_val)

             # Some information printing
             print(t_minh)
             print("Computational time to integrate (s): {:.5f}".format(time.time() - start))
    
        ## EDITED CODE: Terminate if converged
        compare = np.max(np.abs(V_1.asnumpy()-prev))
        print("Maximum change at iteration: {:.5f}".format(compare))
        if compare < convergeThresh:
            break

    # Time info printing
    print("Total kernel time (s): {:.5f}".format(execution_time))
    print("Finished solving\n")

    ##################### PLOTTING #####################
    if plot_option.do_plot :
        # Only plots last value array for now
        plot_isosurface(grid, V_1.asnumpy(), plot_option)

    return V_1.asnumpy()
