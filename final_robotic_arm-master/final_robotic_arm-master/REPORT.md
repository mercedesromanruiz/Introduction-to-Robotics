# ENGR 321. Introduction to Robotics. Final Project.
## Robotic Arm. Camera at the end-effector.
The objective of the project is to control a robotic arm to pick and place all object to a know location. The robotic arm used is the *Universal Robotics UR10e* which has an integrated camera at the end-effector that will be used to lacate the objects.
### Foward Kinematics
#### Transformation matrix for the fiven DH parameters
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
#### Transformation matrix for rotation about z-axis
```
def trans_rot_z(th):
    ct = math.cos(th)
    st = math.sin(th)

    return np.array([[ct, -st, 0, 0],
                     [st, ct, 0, 0],
                     [0, 0, 1, 0],
                     [0, 0, 0, 1]])
```                     
#### DH parameters of UR10e
```
dh_param = [[0.1807, 0, 0, math.pi/2],
            [0, 0, -0.6127, 0],
            [0, 0, -0.57155, 0],
            [0.17415, 0, 0, math.pi/2],
            [0.11985, 0, 0, -math.pi/2],
            [0.11655, 0, 0, 0]]
```
#### Foward kinematics
```
def fwd_kin(q):
    # Translation between webot's frame and ur10e's base frame
    M = np.array([[-1, 0, 0, 0],
                  [ 0,-1, 0, 0],
                  [ 0, 0, 1, 0],
                  [ 0, 0, 0, 1]])

    for i in range(6):
        rot_z = trans_rot_z(q[i])     # rotation about z axis by q[i]. You created it above.
        dh    = trans_dh(dh_param[i]) # dh-transformation     for dh_param[i]. You created it above.
        M = M @ rot_z @ dh

    return M
```

### Inverse Kinematics
```
```
### Pick & Place
```
```
