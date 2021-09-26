import heterocl as hcl
import numpy as np
import time

"""
NOTE: THIS IS NOT MY ORIGINAL CODE. It was adopted from the Matlab .m
files from helperOC Optimal Control toolbox. I simply rewrote it in python
and added some documentation. See the original .m files at
https://github.com/HJReachability/helperOC.git in helperOC/dynSys/@P3D_Q2D_Rel
"""


class P3D_Q2D_Rel:
    def __init__(self, x=[0,0,0], uMin=-1, uMax=1, pMin=[-1,1], pMax=[1,1], dMin=[-0.1,-0.1], dMax=[0.1,0.1], v=5, dims=[0,1,2], uMode="min", dMode="max"):
        self.x = x              # state vector
        self.uMin = uMin        # control input (angular velocity) minimum
        self.uMax = uMax        # control input (angular velocity) maximum
        self.pMin = pMin        # planning step minimums
        self.pMax = pMax        # planning step maximums
        self.dMin = dMin        # system disturbance minimums
        self.dMax = dMax        # system disturbance maximums
        self.v = v              # system speed
        self.dims = dims        # state dimensions
        self.nx = len(dims)     # number of state dimensions
        self.nu = 1             #
        self.nd = 4
        self.pdim = [0,1]
        self.vdim = [2]
        self.uMode = uMode      # controller acting as maximizer or minimizer
        self.dMode = dMode      # disturbing agent acting as minimizer or maximizer

    def opt_ctrl(self, t, state, spat_deriv):
        opt_w = hcl.scalar(self.uMax, "opt_w")
        # Just create and pass back, even though they're not used
        in2 = hcl.scalar(0, "in2")
        in3 = hcl.scalar(0, "in3")
        in4 = hcl.scalar(0, "in4")
        with hcl.if_(spat_deriv[2] >= 0):
            with hcl.if_(self.uMode == "min"):
                opt_w[0] = hcl.scalar(self.uMin, "opt_w")
        with hcl.elif_(spat_deriv[2] < 0):
            with hcl.if_(self.uMode == "max"):
                opt_w[0] = hcl.scalar(self.uMin, "opt_w")
        return (opt_w[0], in3[0], in4[0])

    def opt_dstb(self, t, state, spat_deriv):
        """
        :param spat_deriv: tuple of spatial derivative in all dimensions
        :return: a tuple of optimal disturbances
        """
        d1 = hcl.scalar(self.dMax[0] + self.pMax[0], "d1")
        d2 = hcl.scalar(self.dMax[1] + self.pMax[1], "d2")
        d3 = hcl.scalar(0, "d3")
        # d4 = hcl.scalar(self.pMax[1], "d4")

        with hcl.if_(self.dMode == "max"):
            with hcl.if_(spat_deriv[0]<0):
                d1[0] = hcl.scalar(self.dMin[0] + self.pMin[0],"d1")
                # d2[0] = hcl.scalar(self.pMin[0],"d2")
            with hcl.if_(spat_deriv[1]<0):
                d2[0] = hcl.scalar(self.dMin[1] + self.pMin[1],"d2")
                # d3[0] = hcl.scalar(self.dMin[1],"d3")
                # d4[0] = hcl.scalar(self.pMin[1],"d4")
        with hcl.if_(self.dMode == "min"):
            with hcl.if_(spat_deriv[0]>=0):
                d1[0] = hcl.scalar(self.dMin[0] + self.pMin[0],"d1")
                # d2[0] = hcl.scalar(self.pMin[0],"d2")
            with hcl.if_(spat_deriv[1]>=0):
                d2[0] = hcl.scalar(self.dMin[1] + self.pMin[1],"d2")
                # d3[0] = hcl.scalar(self.dMin[1],"d3")
                # d4[0] = hcl.scalar(self.pMin[1],"d4")

        return (d1[0], d2[0], d3[0])

    def dynamics(self, t, state, uOpt, dOpt):
        """
        Dynamics:
            \dot x_1 = v*cos(x_3)  + d{1}  - d{2}
            \dot x_2 = v*sin(x_3)  + d{3}  - d{4}               
            \dot x_3 = u{1}
                uMin <= u <= uMax
        
        u       <- control of 3D plane (tracker)
        d{1,2}  <- disturbance on x,y + control of 2D quadrotor (planner)
        d{3}    <- zero
        """

        x_dot = hcl.scalar(0, "x_dot")
        y_dot = hcl.scalar(0, "y_dot")
        theta_dot = hcl.scalar(0, "theta_dot")

        x_dot[0] = self.v*hcl.cos(state[2]) + dOpt[0]
        y_dot[0] = self.v*hcl.sin(state[2]) + dOpt[1]
        theta_dot[0] = uOpt[0]

        return (x_dot[0], y_dot[0], theta_dot[0])
