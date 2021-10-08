import heterocl as hcl
import numpy as np
from spatialDerivatives.second_orderENO3D import *
from spatialDerivatives.first_orderENO3D import *


def Gradient3D(g):

    def compute(V):

        deriv_diff1 = hcl.compute(V.shape, lambda *x: 0, "deriv_diff1")
        deriv_diff2 = hcl.compute(V.shape, lambda *x: 0, "deriv_diff2")
        deriv_diff3 = hcl.compute(V.shape, lambda *x: 0, "deriv_diff3")

        with hcl.Stage("Hamiltonian"):  
            with hcl.for_(0, V.shape[0], name="i") as i:  # Plus 1 as for loop count stops at V.shape[0]
                with hcl.for_(0, V.shape[1], name="j") as j:
                    with hcl.for_(0, V.shape[2], name="k") as k:
                        # Variables to calculate dV_dx
                        dV_dx_L = hcl.scalar(0, "dV_dx_L")
                        dV_dx_R = hcl.scalar(0, "dV_dx_R")
                        dV_dx = hcl.scalar(0, "dV_dx")
                        # Variables to calculate dV_dy
                        dV_dy_L = hcl.scalar(0, "dV_dy_L")
                        dV_dy_R = hcl.scalar(0, "dV_dy_R")
                        dV_dy = hcl.scalar(0, "dV_dy")
                        # Variables to calculate dV_dtheta
                        dV_dT_L = hcl.scalar(0, "dV_dT_L")
                        dV_dT_R = hcl.scalar(0, "dV_dT_R")
                        dV_dT = hcl.scalar(0, "dV_dT")
                        # Variables to keep track of dynamics
                        # dx_dt = hcl.scalar(0, "dx_dt")
                        # dy_dt = hcl.scalar(0, "dy_dt")
                        # dtheta_dt = hcl.scalar(0, "dtheta_dt")

                        # No tensor slice operation
                        # dV_dx_L[0], dV_dx_R[0] = spa_derivX(i, j, k, V, g)
                        # dV_dy_L[0], dV_dy_R[0] = spa_derivY(i, j, k, V, g)
                        # dV_dT_L[0], dV_dT_R[0] = spa_derivT(i, j, k, V, g)
                        dV_dx_L[0], dV_dx_R[0] = secondOrderX(i, j, k, V, g)
                        dV_dy_L[0], dV_dy_R[0] = secondOrderY(i, j, k, V, g)
                        dV_dT_L[0], dV_dT_R[0] = secondOrderT(i, j, k, V, g)

                        # Saves spatial derivative diff into tables
                        deriv_diff1[i, j, k] = (dV_dx_R[0] + dV_dx_L[0]) / 2
                        deriv_diff2[i, j, k] = (dV_dy_R[0] + dV_dy_L[0]) / 2
                        deriv_diff3[i, j, k] = (dV_dT_R[0] + dV_dT_L[0]) / 2
        
        return deriv_diff1,deriv_diff2,deriv_diff3

    V = hcl.placeholder(tuple(g.pts_each_dim), name="V", dtype=hcl.Float())
    s = hcl.create_schedule([V],compute)

    f = hcl.build(s)

    return f
    
