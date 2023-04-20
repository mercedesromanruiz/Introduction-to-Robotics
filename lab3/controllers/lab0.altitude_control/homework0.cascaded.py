# Lab 0

# In this example, roll & pitch is externally controlled.
# Controlling roll & pitch is in Lab 1.

import PID
import math

pid_z = PID.PID(20, 0.01, 50)
pid_vz = PID.PID(1, 0.01, 0.2)

# Return [w_fl, w_fr, w_rl, w_rr]
def control(cx, cy, cz, croll, cpitch, cyaw, # current position
            vx, vy, vz, wroll, wpitch, wyaw, # current velocity
            rx, ry, rz, rroll, rpitch, ryaw, # reference(desired) position
            dt):
    r_vz = pid_z.get(rz - cz, dt)    # outer PID: position -> velocity
    w_z  = pid_vz.get(r_vz - vz, dt) # inner PID: velocity -> rotor speed
    w_offset = 68.5

    w_fl = w_offset + w_z
    w_fr = w_offset + w_z
    w_rl = w_offset + w_z
    w_rr = w_offset + w_z

    return [w_fl, w_fr, w_rl, w_rr]
