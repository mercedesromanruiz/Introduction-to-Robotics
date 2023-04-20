# Lab 0

# In this example, roll & pitch is externally controlled.
# Controlling roll & pitch is in Lab 1.

import PID
import math

# Try to tune it but don't spend too much time
pid_z = PID.PID(1, 0, 0)

# Return [w_fl, w_fr, w_rl, w_rr]
def control(cx, cy, cz, croll, cpitch, cyaw, # current position
            vx, vy, vz, wroll, wpitch, wyaw, # current velocity
            rx, ry, rz, rroll, rpitch, ryaw, # reference(desired) position
            dt):
    w_z = pid_z.get(rz - cz, dt)
    w_offset = 68.5

    w_fl = w_offset + w_z
    w_fr = w_offset + w_z
    w_rl = w_offset + w_z
    w_rr = w_offset + w_z

    return [w_fl, w_fr, w_rl, w_rr]
