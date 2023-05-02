# ENGR 321. Introduction to Robotics. Final Project.
## Robotic Arm. Camera at the end-effector.
The objective of the project is to control a robotic arm to pick and place all object to a know location. The robotic arm used is the 
```
def trans_dh(dh):
    d, th, r, al = dh

    ct  = math.cos(th)
    st  = math.sin(th)
    cal = math.cos(al)
    sal = math.sin(al)

    return np.array([[ct, -st * cal, st * sal, r * ct],
                     [st, ct * cal, -ct * sal, r * st],
                     [0, sal, cal, d],
                     [0, 0, 0, 1]])
```
