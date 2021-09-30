import heterocl as hcl
import numpy as np
import time

"""
NOTE: THIS IS NOT MY ORIGINAL CODE. It was adopted from the Matlab .m
files from helperOC Optimal Control toolbox. I simply rewrote it in python
and added some documentation. See the original .m files at
https://github.com/HJReachability/helperOC.git in helperOC/dynSys/@P3D_Q2D_Rel
"""


class P5D_Dubins_Rel:
    def __init__(self, x=[0,0,0,0,0], uMin=[-0.5,-6], uMax=[0.5,6], pMin=[-1.5], pMax=[1.5],
                dMin=[-0.02,-0.02,0,-0.02,-0.2], dMax=[0.02,0.02,0,0.02,0.2], v=0.1, dims=[0,1,2,3,4], uMode="min", dMode="max"):
        # System decomposition not possible due to coupling of states
        
        self.x = x              # state vector
        self.uMin = uMin        # control input (angular velocity) minimum
        self.uMax = uMax        # control input (angular velocity) maximum
        self.pMin = pMin        # planning step minimums
        self.pMax = pMax        # planning step maximums
        self.dMin = dMin        # system disturbance minimums
        self.dMax = dMax        # system disturbance maximums
        self.v = v              # system speed
        self.dims = dims        # state dimensions
        self.uMode = uMode      # controller acting as maximizer or minimizer
        self.dMode = dMode      # disturbing agent acting as minimizer or maximizer

        

    def opt_ctrl(self, t, state, spat_deriv):
        opt_a = hcl.scalar(self.uMax[0], "opt_a")
        opt_alpha = hcl.scalar(self.uMax[1],"opt_alpha")
        # Just create and pass back, even though they're not used
        in1 = hcl.scalar(0, "in2")
        in2 = hcl.scalar(0, "in3")
        in3 = hcl.scalar(0, "in4")
        with hcl.if_(spat_deriv[2] >= 0):
            with hcl.if_(self.uMode == "min"):
                opt_a[0] = hcl.scalar(self.uMin[0], "opt_a")
                opt_alpha[0] = hcl.scalar(self.uMin[1], "opt_alpha")
        with hcl.elif_(spat_deriv[2] < 0):
            with hcl.if_(self.uMode == "max"):
                opt_a[0] = hcl.scalar(self.uMin[0], "opt_a")
                opt_alpha[0] = hcl.scalar(self.uMin[1], "opt_alpha")
        return (in1[0],in2[0],in3[0],opt_a[0],opt_alpha[0])

    def opt_dstb(self, spat_deriv):
        """
        :param spat_deriv: tuple of spatial derivative in all dimensions
        :return: a tuple of optimal disturbances
        """
        d1 = hcl.scalar(self.dMax[0], "d1")
        d2 = hcl.scalar(self.dMax[1], "d2")
        d3 = hcl.scalar(self.pMax[0], "d3")
        d4 = hcl.scalar(self.dMax[3], "d4")
        d5 = hcl.scalar(self.dMax[4], "d5")
        
        with hcl.if_(self.dMode == "max"):
            with hcl.if_(spat_deriv[0]<0):
                d1[0] = hcl.scalar(self.dMin[0],"d1")
            with hcl.if_(spat_deriv[1]<0):
                d2[0] = hcl.scalar(self.dMin[1],"d2")
            with hcl.if_(spat_deriv[2]<0):
                d3[0] = hcl.scalar(self.pMin[0],"d3")
            with hcl.if_(spat_deriv[3]<0):
                d4[0] = hcl.scalar(self.dMin[3],"d4")
            with hcl.if_(spat_deriv[4]<0):
                d5[0] = hcl.scalar(self.dMin[4],"d5")
        with hcl.if_(self.dMode == "min"):
            with hcl.if_(spat_deriv[0]>=0):
                d1[0] = hcl.scalar(self.dMin[0],"d1")
            with hcl.if_(spat_deriv[1]>=0):
                d2[0] = hcl.scalar(self.dMin[1],"d2")
            with hcl.if_(spat_deriv[2]>=0):
                d3[0] = hcl.scalar(self.pMin[0],"d3")
            with hcl.if_(spat_deriv[3]>=0):
                d4[0] = hcl.scalar(self.dMin[3],"d4")
            with hcl.if_(spat_deriv[4]>=0):
                d5[0] = hcl.scalar(self.dMin[4],"d5")

        return (d1[0], d2[0], d3[0], d4[0], d5[0])

    def dynamics(self, t, state, uOpt, dOpt):
        """
        Dynamics:
            \dot x_1 = -v + x_4*cos(x_3)  + d{3}*x_2 + d{1}
            \dot x_2 = x_4*sin(x_3)  - d{3}*x_1 + d{2]}            
            \dot x_3 = x_5 - d{3}
            \dot x_4 = u{4} + d{4}
            \dot x_5 = u{5} + d{5}
                uMin <= u <= uMax
        
        u       <- control of 3D plane (tracker)
        d{1,2,4,5}  <- disturbance on x,y,v,omega
        d{3}    <- planner control
        """
        x_dot = hcl.scalar(0,"x_dot")
        y_dot = hcl.scalar(0,"y_dot")
        theta_dot = hcl.scalar(0,"theta_dot")
        v_dot = hcl.scalar(0,"v_dot")
        omega_dot = hcl.scalar(0,"omega_dot")

        x_dot[0] = -self.v + state[3]*hcl.cos(state[2]) + dOpt[2]*state[1] + dOpt[0]
        y_dot[0] = state[3]*hcl.sin(state[2]) - dOpt[2]*state[0] + dOpt[1]
        theta_dot[0] = state[4] - dOpt[2]
        v_dot[0] = uOpt[3] + dOpt[3]
        omega_dot[0] = uOpt[4] + dOpt[4]
        return (x_dot[0], y_dot[0], theta_dot[0], v_dot[0], omega_dot[0])
