# Lab 2

import PID

import numpy as np
import math

# For NumPy, refer to NumPy quickstart
#   https://numpy.org/devdocs/user/quickstart.html

# How to make a matrix M = [a b].
#                          [c d]
#
#   M = np.array([[a, b], [c, d]])

# How to access elements of M
#
# m11 = M[0, 0]   # Index starts from 0 in Python

# How to multiply two matrices A and B: C = A * B
#
#   C = np.matmul(A, B)
# or
#   C = A @ B

# Transpose of matrix A: B = A^T
#
#   B = A.transpose()

# Trigonometric functions
#
#   math.cos(x), math.sin(x)



# Return [w_fl, w_fr, w_rl, w_rr]
def control(cx, cy, cz, croll, cpitch, cyaw, # current position
            vx, vy, vz, wroll, wpitch, wyaw, # current velocity
            rx, ry, rz, rroll, rpitch, ryaw, # reference(desired) position
            dt):

    w_fl = ...
    w_fr = ...
    w_rl = ...
    w_rr = ...

    return [w_fl, w_fr, w_rl, w_rr]
