# Lab 1

# Roll and pitch are not externally controlled anymore in this lab.

import PID
import math



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
