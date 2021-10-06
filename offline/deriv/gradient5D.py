import heterocl as hcl
from spatialDerivatives.second_orderENO5D import *

def computeGradient(g,V):

    deriv_diff1 = hcl.compute(V.shape, lambda *x: 0, "deriv_diff1")
    deriv_diff2 = hcl.compute(V.shape, lambda *x: 0, "deriv_diff2")
    deriv_diff3 = hcl.compute(V.shape, lambda *x: 0, "deriv_diff3")
    deriv_diff4 = hcl.compute(V.shape, lambda *x: 0, "deriv_diff4")
    deriv_diff5 = hcl.compute(V.shape, lambda *x: 0, "deriv_diff5")

   

    # Calculate Hamiltonian for every grid point in V
    with hcl.for_(0, V.shape[0], name="i") as i:
        with hcl.for_(0, V.shape[1], name="j") as j:
            with hcl.for_(0, V.shape[2], name="k") as k:
                with hcl.for_(0, V.shape[3], name="l") as l:
                    with hcl.for_(0, V.shape[4], name="m") as m:
                            # Variables to calculate dV_dx
                            dV_dx1_L = hcl.scalar(0, "dV_dx1_L")
                            dV_dx1_R = hcl.scalar(0, "dV_dx1_R")
                            dV_dx1 = hcl.scalar(0, "dV_dx1")
                            dV_dx2_L = hcl.scalar(0, "dV_dx2_L")
                            dV_dx2_R = hcl.scalar(0, "dV_dx2_R")
                            dV_dx2 = hcl.scalar(0, "dV_dx2")
                            dV_dx3_L = hcl.scalar(0, "dV_dx3_L")
                            dV_dx3_R = hcl.scalar(0, "dV_dx3_R")
                            dV_dx3 = hcl.scalar(0, "dV_dx3")
                            dV_dx4_L = hcl.scalar(0, "dV_dx4_L")
                            dV_dx4_R = hcl.scalar(0, "dV_dx4_R")
                            dV_dx4 = hcl.scalar(0, "dV_dx4")
                            dV_dx5_L = hcl.scalar(0, "dV_dx5_L")
                            dV_dx5_R = hcl.scalar(0, "dV_dx5_R")
                            dV_dx5 = hcl.scalar(0, "dV_dx5")

                            # No tensor slice operation
                            # dV_dx_L[0], dV_dx_R[0] = spa_derivX(i, j, k)
                            dV_dx1_L[0], dV_dx1_R[0] = secondOrderX1_5d(i, j, k, l, m, V, g)
                            dV_dx2_L[0], dV_dx2_R[0] = secondOrderX2_5d(i, j, k, l, m, V, g)
                            dV_dx3_L[0], dV_dx3_R[0] = secondOrderX3_5d(i, j, k, l, m, V, g)
                            dV_dx4_L[0], dV_dx4_R[0] = secondOrderX4_5d(i, j, k, l, m, V, g)
                            dV_dx5_L[0], dV_dx5_R[0] = secondOrderX5_5d(i, j, k, l, m, V, g)

                            # Saves spatial derivative diff into tables
                            deriv_diff1[i, j, k, l, m] = dV_dx1_R[0] - dV_dx1_L[0]
                            deriv_diff2[i, j, k, l, m] = dV_dx2_R[0] - dV_dx2_L[0]
                            deriv_diff3[i, j, k, l, m] = dV_dx3_R[0] - dV_dx3_L[0]
                            deriv_diff4[i, j, k, l, m] = dV_dx4_R[0] - dV_dx4_L[0]
                            deriv_diff5[i, j, k, l, m] = dV_dx5_R[0] - dV_dx5_L[0]


    return [deriv_diff1[0],deriv_diff2[0],deriv_diff3[0],deriv_diff4[0],deriv_diff5[0]]