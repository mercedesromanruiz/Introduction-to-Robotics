# Objective: implement forward kinematics, inverse kinematics, and pick&place application

# Hints
#  MATH  : PYTHON
# --------------------------------
#  x^2   : x**2
#  √x    : math.sqrt(x)
#  pi    : math.pi
#  e(x)  : math.exp(x)
#  ln(x) : math.log(x)
#  cos(x): math.cos(x)
#  sin(x): math.sin(x)
#
# Given a matrix M,
#  first  row: M[0, :]
#  second row: M[1, :]
#  first  col: M[:, 0]
#  second col: M[:, 1]
#
#  1,2,3-rd row & 1st col: M[0:3, 0] or M[:3, 0]
#  1,2,3-rd row & 3rd col: M[0:3, 2] or M[:3, 2]


import numpy as np
import scipy.optimize

import math

# 1.1. Transformation matrix for the given DH parameters
#      Fill out the underscore parts.
#      See https://en.wikipedia.org/wiki/Denavit%E2%80%93Hartenberg_parameters#Denavit%E2%80%93Hartenberg_matrix
#
#      dh: [d, theta, r(a), alpha]
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

# 1.2. Transformation matrix for rotation about z-axis
#      Fill out the underscore parts.
#      See the lecture slides.
#
#      th: rotation angle about z-axis
def trans_rot_z(th):
    ct = math.cos(th)
    st = math.sin(th)

    return np.array([[ct, -st, 0, 0],
                     [st, ct, 0, 0],
                     [0, 0, 1, 0],
                     [0, 0, 0, 1]])

# 1.3. DH parameters of UR10e
#      Fill out the underscore parts.
#      See https://www.universal-robots.com/articles/ur/application-installation/dh-parameters-for-calculations-of-kinematics-and-dynamics/
#      Make sure that you're using the UR10e parameters
#
#           [d, theta, r(a), alpha]
dh_param = [[0.1807, 0, 0, math.pi/2],
            [0, 0, -0.6127, 0],
            [0, 0, -0.57155, 0],
            [0.17415, 0, 0, math.pi/2],
            [0.11985, 0, 0, -math.pi/2],
            [0.11655, 0, 0, 0]]

# 1.4. Forward kinematics
#      Given joint angles q[0] to q[5], calculate the transformation matrix of the end-effector frame.
#      Fill out the underscore parts.
#
# q: joint angles: [q0, q1, q2, q3, q4, q5]
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


# CONGRATULATIONS, you have completed forward kinematics.
# Open worlds/1.fwd_kin.wbt, run it, and see if EE moves toward the arrow.
# Confirm that the location and orientation match.


# 2.1. Inverse kinematics that moves EE_loc to 'loc'
#
#      Given current joint angles 'q0' and the desired EE location 'loc',
#      solve the IK problem using a minimization algorithm.
#      Fill out the underscore parts.
#
#      Hint: You are trying to move EE to the desired location.
#        <=> You are trying to minimize the distance between EE location and the desired location
#        <=> You are trying to minimize the distance between EE location 'M[_____]' and the desired location 'loc'
#        <=> You are trying to minimize the magnitude of 'M[_____] - loc'
#        <=> You are trying to minimize np.linalg.norm(M[_____] - loc)
#
def inv_kin(q0, loc):
    def objective(q): # objective function to minimize
        M = fwd_kin(q)
        e = np.linalg.norm(M[0:3, 3] - loc)
        return e

    return scipy.optimize.minimize(objective, q0).x

# 2.2. Inverse Kinematics that moves EE_loc    to 'loc',
#                                    EE_z_axis to 'z_axis'
#
#      You are minimizing two things together.
#      It is best to use multiobjective optimization, but it's out of scope.
#      Instead of multiobjective optimization, let's use a linear combination of the objectives.
#      Fill out the underscore parts.
#
#      Hint: You are trying to minimize the distance   between EE_loc    and 'loc'   : say e_loc
#                                     & the difference between EE_z_axis and 'z_axis': say e_z
#        <=> You are trying to minimize a linear combination of e_loc and e_z
#        <=> You are trying to minimize 'w_loc * e_loc + w_z * e_z'
#
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

# 2.3. Inverse Kinematics that moves EE_loc    to 'loc',
#                                    EE_z_axis to 'z_axis',
#                                    EE_x_axis to 'x_axis'
#      If you completed 2.2, no more explanation will be needed.
#      Fill out the underscore parts.
#
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

# CONGRATULATIONS, you have completed inverse kinematics.
# Open worlds/2.inv_kin.wbt, run it, and see if EE moves toward the arrow.
# Confirm that the location and orientation match.


# 3. Pick & place application
#    Objects are randomly placed. Height of the object is 0.004 m
#    Dropzone for red is at (0, -0.35, 0),
#                 green  at (0, -0.5, 0),
#                 blue   at (0, -0.65, 0),
# ---
# get_joint_angles()   : returns current joint angles
# set_joint_angles(q)  : sets the joint angles
#
# detect_object(color) : detects an object of color r, g, b &
#                        returns [ix, iy] if an object is detected
#                                None     otherwise
# image_width          : width  of the camera image in pixels
# image_height         : height of the camera image in pixels
#
# grip()    : grip an object
# release() : release an object
#
# sleep_for(s) : sleep for 's' seconds

def control(get_joint_angles, set_joint_angles,
            detect_object, image_width, image_height,
            grip, release,
            sleep_for):
    # Predefined colors
    RED   = (1, 0, 0)
    GREEN = (0, 1, 0)
    BLUE  = (0, 0, 1)

    # Locations of the dropzones
    DROPZONES = {
        RED:   [0, -0.35, 0.1],
        GREEN: [0, -0.5,  0.1],
        BLUE:  [0, -0.65, 0.1],
    }

    # Q for the initial pose
    INITIAL_Q = [0, -math.pi / 2, 0, 0, 0, 0]

    # for each color Red, Green, Blue
    for c in [RED, GREEN, BLUE]:
        while True:
            # Move EE to the location where it can see the object.
            # The viewpoint angle matters too, because we will locate the objects from the camera image.
            # Do we need to move only EE (inv_kin)?
            #            or move EE with z-axis (inv_kin_z)?
            #            or move EE with z-axis and x-axis (inv_kin_zx)?
            #
            loc = [0.5, 0, 0.4]
            x_axis = [0, -1, 0] # We need it.
            z_axis = [0, 0, -1] # We need it.
            q = inv_kin_zx(INITIAL_Q, loc, z_axis, x_axis)
            set_joint_angles(q)
            sleep_for(2) # wait until it reaches the target location

            obj = detect_object(*c)

            # if no object with color c exists, then break the loop and move on to the next color
            if obj is None:
                break

            # An object of color 'c' is on (ix, iy) in the image
            # IT IS THE LOCATION ON THE IMAGE.
            #
            #          ix
            #     -------->
            #    |
            #    |  Image
            # iy |
            #    v
            #
            ix, iy = obj;


            # 3.1. Transform (ix, iy) to (gx, gy)_global
            #      You are supposed to use a projection matrix, which wasn't part of the topics of this course.
            #      However, the camera's view angle is 90 degress on both height and width,
            #           and the image_width and image_height are the same.
            #
            #                /----------
            #               /  Robotic arm
            #              /    ________
            #             /    /
            #      ___    |___|  <- camera is at EE
            #       |      / \
            #       |     /90°\
            #    h  |    /     \
            #       |   /       \
            #       |  /         \
            # -----===---------------------  Floor
            #         |-----------|
            #               h
            #
            #      As you know the location of EE including the height, (See the target location of EE above)
            #      Transformation of the image location to the global frame should be straightforward.
            #      For example,
            #         If ix is 0,            gy is 0.4.  (0   + 0.4)
            #         If ix is image_width,  gy is -0.4. (0   - 0.4)
            #         If iy is 0,            gx is 0.9.  (0.5 + 0.4)
            #         If iy is image_height, gx is 0.1.  (0.5 - 0.4)
            #
            gx = - (iy - 0.5 * image_height) + 0.5
            gy = - (ix - 0.5 * image_width) 

            # 3.2. Move EE to somewhere above the object location, like 0.05m above
            #      Do we need to move only EE (inv_kin)?
            #                 or move EE with z-axis (inv_kin_z)?
            #                 or move EE with z-axis and x-axis (inv_kin_zx)?
            #
            loc = [gx, gy, 0.05]
            z_axis = [0, 0, -1] # Do we need it?
            x_axis = [1, 0, 0]  # Do we need it?
            q = inv_kin_zx(get_joint_angles(), loc, z_axis, x_axis)
            set_joint_angles(q)
            sleep_for(1)

            # 3.3. Move EE close to the object to pick. Don't forget that the height of the object is 0.004.
            #      Do we need to move only EE (inv_kin)?
            #                 or move EE with z-axis (inv_kin_z)?
            #                 or move EE with z-axis and x-axis (inv_kin_zx)?
            #
            loc = [gx, gy, 0.004]
            z_axis = [0, 0, -1] # Do we need it?
            x_axis = [1, 0, 0]  # Do we need it?
            q = inv_kin_z(get_joint_angles(), loc, z_axis)
            set_joint_angles(q)
            sleep_for(1)

            # Grip the object
            grip()
            sleep_for(1)

            # 3.4. Move EE to somewhere above the object location, like 0.05m above.
            #      Do we need to move only EE (inv_kin)?
            #                 or move EE with z-axis (inv_kin_z)?
            #                 or move EE with z-axis and x-axis (inv_kin_zx)?
            #
            loc = [gx, gy, 0.05]
            z_axis = [0, 0, -1] # Do we need it?
            x_axis = [1, 0, 0]  # Do we need it?
            q = inv_kin_z(get_joint_angles(), loc, z_axis)
            set_joint_angles(q)
            sleep_for(1)

            # 3.5. Move EE to somewhere above the dropzone.
            #      Do we need to move only EE (inv_kin)?
            #                 or move EE with z-axis (inv_kin_z)?
            #                 or move EE with z-axis and x-axis (inv_kin_zx)?
            #
            loc = DROPZONES[c]
            z_axis = [0, 0, -1] # Do we need it?
            x_axis = [1, 0, 0]  # Do we need it?
            q = inv_kin_zx(get_joint_angles(), loc, z_axis, x_axis)
            set_joint_angles(q)
            sleep_for(1)

            # Release the object
            release()

    # Go back to the initial pos
    set_joint_angles([0, -math.pi / 2, 0, 0, 0, 0])
    sleep_for(1)
