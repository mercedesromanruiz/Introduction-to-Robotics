# ENGR 321. Introduction to Robotics. Final Project.
## Robotic Arm. Camera at the end-effector.
The objective of the project is to control a robotic arm to pick and place all object to a know location. The robotic arm used is the *Universal Robotics UR10e* which has an integrated camera at the end-effector that will be used to lacate the objects.
### 1. Foward Kinematics
#### 1.1. Transformation matrix for the fiven DH parameters
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
#### 1.2. Transformation matrix for rotation about z-axis
```
def trans_rot_z(th):
    ct = math.cos(th)
    st = math.sin(th)

    return np.array([[ct, -st, 0, 0],
                     [st, ct, 0, 0],
                     [0, 0, 1, 0],
                     [0, 0, 0, 1]])
```                     
#### 1.3. DH parameters of UR10e
```
dh_param = [[0.1807, 0, 0, math.pi/2],
            [0, 0, -0.6127, 0],
            [0, 0, -0.57155, 0],
            [0.17415, 0, 0, math.pi/2],
            [0.11985, 0, 0, -math.pi/2],
            [0.11655, 0, 0, 0]]
```
#### 1.4. Foward kinematics
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

### 2. Inverse Kinematics
#### 2.1. Inverse kinematics that moves EE_loc to 'loc'
```
def inv_kin(q0, loc):
    def objective(q): # objective function to minimize
        M = fwd_kin(q)
        e = np.linalg.norm(M[0:3, 3] - loc)
        return e

    return scipy.optimize.minimize(objective, q0).x
```
#### 2.2. Inverse kinematics that moves EE_loc to 'loc' & EE_z_axis to 'z_axis'
```
def inv_kin_z(th, loc, z_axis):
    def objective(q): # objective function to minimize
        M = fwd_kin(q)

        #error
        e_loc = np.linalg.norm(M[0:3, 3] - loc)
        e_z   = np.linalg.norm(M[0:3, 2] - z_axis)

        #error weight
        w_loc = 10
        w_z   = 1

        return w_loc * e_loc + w_z * e_z

    return scipy.optimize.minimize(objective, th).x
```
#### 2.2. Inverse Kinematics that moves EE_loc to 'loc' & EE_z_axis to 'z_axis' & EE_x_axis to 'x_axis'
```
def inv_kin_zx(th, loc, z_axis, x_axis):
    def objective(q): # objective function to minimize
        M = fwd_kin(q)

        #error
        e_loc = np.linalg.norm(M[0:3, 3] - loc)
        e_z   = np.linalg.norm(M[0:3, 2] - z_axis)
        e_x   = np.linalg.norm(M[0:3, 0] - x_axis)

        #error weight
        w_loc = 10
        w_z   = 1
        w_x   = 1

        return w_loc * e_loc + w_z * e_z + w_x * e_x

    return scipy.optimize.minimize(objective, th).x
```
### 3. Pick & Place
#### 3.1. Transform (ix, iy) to (gx, gy)_global
#### 3.2. Move EE to somewhere above the object location
#### 3.3. Move EE close to the object to pick
#### 3.4. Move EE to somewhere above the object location
#### 3.5. Move EE to somewhere above the dropzone
![Alt Text](https://media.giphy.com/media/vFKqnCdLPNOKc/giphy.gif)
```
```
