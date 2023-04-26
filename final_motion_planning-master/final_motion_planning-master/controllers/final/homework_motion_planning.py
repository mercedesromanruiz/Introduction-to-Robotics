# Objective: Implement artificial potential field method.
#
# I recommend you should read the code and try to understand how artificial potential field works.
# You do NOT need to change any code but the parameters.

# Hints
#  MATH  : PYTHON
# --------------------------------
#  x^2   : x**2
#  √x    : math.sqrt(x)
#  pi    : math.pi
#  e(x)  : math.exp(x)
#  ln(x) : math.log(x)

# Normal (Gaussian) random
#  MATH  : X1 is from X ~ N(mu, sigma^2)
#  PYTHON: X1 = random.gauss(mu, sigma)

import math
import numpy as np

# This is a constant
ROBOT_RADIUS = 0.3

# 1. Following parameters are not good. Try different values to get a better result.
#   Hint: If your estimation often gets far from the true state,
#         it means that the wheels slip too much due to high speed.

# Artificial potential field parameters
K_att = 1            # attractive force gain
dist_effective_g = 1 # attractive force: dist2goal == dist_effective_g if dist2goal > dist_effective_g
K_rep = 1            # repulsive force gain
dist_effective_o = 1 # repulsive force is 0 if dist to obstacle is >dist_effective_o

# Tracker parameters
K_lin = 1            # linear velocity gain
K_ang = 1            # angular velocity gain


# Calculate attractive force
#   cur  : [x, y]^T
#   dest : [x, y]^T
def attractive_force(cur, dest):
    v = dest - cur
    dist = np.linalg.norm(v)
    v = v / dist # normalize the vector

    if dist > dist_effective_g:
        dist = dist_effective_g

    return K_att * dist * v

# Calculate repulsive force
#   cur : [x, y]^T
#   obs : [x, y, r]
def repulsive_force(cur, obs):
    o_x, o_y, o_r = obs
    o = np.array([[o_x], [o_y]])

    v = cur - o
    dist = np.linalg.norm(v)
    v = v / dist # normalize the vector

    dist = dist - (ROBOT_RADIUS + o_r) # real distance

    if dist < 0:
        dist = 0.001

    if dist > dist_effective_o:
        dist = dist_effective_o

    return K_rep * v / dist / dist * (1 / dist - 1 / dist_effective_o)

# Calculate the net force
#   cur  : [x, y]^T
#   dest : [x, y]^T
#   obstacles    : [[x1, y1, r1],
#                   ...
#                   [xN, yN, rN]]
#   -----------------------------
#   return [fx, fy]^T
def net_force(cur, dest, obs):
    F = attractive_force(cur, dest)
    for o in obs:
        F = F + repulsive_force(cur, o)
    return F

# Control the robot
#   current state: [x, y, th]^T
#   dest         : [x, y]^T
#   obstacles    : [[x1, y1, r1],
#                   ...
#                   [xN, yN, rN]]
#   -----------------------------
#   return [wl, wr]
def control(cur, dest, obstacles, dt):
    F = net_force(cur[:2], dest, obstacles)

    cur_th = cur[2, 0]
    dest_th = math.atan2(F[1,0], F[0,0])
    e_th = dest_th - cur_th
    # make e_th in [-pi, pi)
    while e_th < -np.pi: e_th += 2 * np.pi
    while e_th >= np.pi: e_th -= 2 * np.pi

    # Linear velocity
    w_lin = K_lin * np.linalg.norm(F) * math.cos(e_th)

    # Angular velocity
    w_ang = K_ang * e_th

    return [w_lin - w_ang, w_lin + w_ang]
