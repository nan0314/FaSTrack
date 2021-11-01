import heterocl as hcl
import numpy as np
import time


class P5D_2D_Rel:
    def __init__(self, x=[0,0,0,0,0], aRange = [-0.15,0.15], alphaMax = 3, pMax  = [0.25,0.25], dMax = [0.02, 0.02, 0, 0.02, 0.02], dims=[0,1,2,3,4], uMode="min", dMode="max"):
        self.x = x              # state vector
        self.aRange = aRange    # acceleration control input bounds
        self.alphaMax = alphaMax # angular acceleration maximum
        self.pMax = pMax        # planning step maximums
        self.dMax = dMax        # system disturbance maximums
        self.dims = dims        # state dimensions
        self.nx = len(dims)     # number of state dimensions
        self.uMode = uMode      # controller acting as maximizer or minimizer
        self.dMode = dMode      # disturbing agent acting as minimizer or maximizer


    def opt_ctrl(self, t, state, spat_deriv):

        # initialize actual control inputs
        opt_a = hcl.scalar(self.aRange[1], "opt_a")
        opt_alpha = hcl.scalar(self.alphaMax, "opt_alpha")

        # Just create and pass back, even though they're not used
        in0 = hcl.scalar(0, "in2")
        in1 = hcl.scalar(0, "in3")
        in2 = hcl.scalar(0, "in4")
        with hcl.if_(self.uMode == "min"):
            with hcl.if_(spat_deriv[3] >= 0):
                opt_a[0] = hcl.scalar(self.aRange[0], "opt_a")
            with hcl.if_(spat_deriv[4] >= 0):
                opt_alpha[0] = hcl.scalar(-self.alphaMax, "opt_alpha")
        with hcl.if_(self.uMode == "max"):
            with hcl.if_(spat_deriv[3] < 0):
                opt_a[0] = hcl.scalar(self.aRange[0], "opt_a")
            with hcl.if_(spat_deriv[4] < 0):
                opt_alpha[0] = hcl.scalar(-self.alphaMax, "opt_alpha")
        return (in0[0],in1[0],in2[0],opt_a[0],opt_alpha[0])

    def opt_dstb(self, spat_deriv):
        """
        :param spat_deriv: tuple of spatial derivative in all dimensions
        :return: a tuple of optimal disturbances
        """

        d0 = hcl.scalar(self.dMax[0] + self.pMax[0], "d0")
        d1 = hcl.scalar(self.dMax[1] + self.pMax[1], "d1")
        d2 = hcl.scalar(self.dMax[2], "d2")
        d3 = hcl.scalar(self.dMax[3], "d3")
        d4 = hcl.scalar(self.dMax[4], "d4")

        with hcl.if_(self.dMode == "max"):
            with hcl.if_(spat_deriv[0]<0):
                d0[0] = hcl.scalar(-self.dMax[0] + self.pMax[0],"d0")
            with hcl.if_(spat_deriv[1]<0):
                d1[0] = hcl.scalar(-self.dMax[1] + self.pMax[1],"d1")
            with hcl.if_(spat_deriv[3]<0):
                d3[0] = hcl.scalar(-self.dMax[3],"d3")
            with hcl.if_(spat_deriv[4]<0):
                d4[0] = hcl.scalar(-self.dMax[4],"d4")

        with hcl.if_(self.dMode == "min"):
            with hcl.if_(spat_deriv[0]>=0):
                d0[0] = hcl.scalar(-self.dMax[0] + self.pMax[0],"d0")
            with hcl.if_(spat_deriv[1]>=0):
                d1[0] = hcl.scalar(-self.dMax[1] + self.pMax[1],"d1")
            with hcl.if_(spat_deriv[3]>=0):
                d3[0] = hcl.scalar(-self.dMax[3],"d3")
            with hcl.if_(spat_deriv[4]>=0):
                d4[0] = hcl.scalar(-self.dMax[4],"d4")

        return (d0[0], d1[0], d2[0], d3[0], d4[0])

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

        xr = state[0]
        yr = state[1]
        th = state[2]
        v = state[3]
        w = state[4]

        x_dot = hcl.scalar(0, "x_dot")
        y_dot = hcl.scalar(0, "y_dot")
        theta_dot = hcl.scalar(0, "theta_dot")
        a = hcl.scalar(0,"a")
        alpha = hcl.scalar(0, "alpha")

        x_dot[0] = v*hcl.cos(th) + dOpt[0]
        y_dot[0] = v*hcl.sin(th) + dOpt[1]
        theta_dot[0] = w
        a[0] = uOpt[3] + dOpt[3]
        alpha[0] = uOpt[4] + dOpt[4]

        return (x_dot[0], y_dot[0], theta_dot[0], a[0], alpha[0])