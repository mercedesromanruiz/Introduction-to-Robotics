# Objective: Given observations and control command, localize the robot

# Hints
#  MATH  : PYTHON
# --------------------------------
#  x^2   : x**2
#  √x    : math.sqrt(x)
#  pi    : math.pi
#  e(x)  : math.exp(x)
#  ln(x) : math.log(x)

import math
import random
import numpy as np
from scipy import optimize
from scipy import stats

# These are constants.
sensor_sigma = 0.2

# 1. Calculate the log-likelihood of the expected, given a (single) observation.
def log_likelihood_single(expected, observed):
    mu = expected
    x = observed
    sigma = sensor_sigma

    # 1.1. Calculate the log-likelihood.
    #    Fill out the underscore parts.
    #    Assume that observation error is known to follow N(0, sigma^2)
    #    See https://en.wikipedia.org/wiki/Normal_distribution#Operations_on_a_single_normal_variable for the equation.
    return _________________________________________________________________________

# 2. Calculate the log-likelihood of the given estimate and observations
#    estimate: [x, y, th]
#    landmark: [[l_x1, l_y1, observed_distance1],
#               [l_x2, l_y2, observed_distance2],
#               ...
#               [l_xN, l_yN, observed_distanceN]]
def log_likelihood_joint(estimate, landmark):
    x, y, th = estimate

    llh = 0 # log likelihood accumulator

    # for each landmark
    for lm in landmark:
        # location of the landmark
        l_x = lm[0]
        l_y = lm[1]

        # observed distance to the landmark
        observed = lm[2]

        # 2.1. Calculate the expected distance to the landmark (l_x, l_y) from (x, y, th)
        #      Fill out the underscore parts.
        expected = ______________________________________

        # 2.2. Accumulate log-likelihood
        #      Fill out the underscore parts.
        llh = llh _ log_likelihood_single(expected, observed)

    return llh

# 3. Maximum Likelihood Estimator
def mle(landmark, prev_estimate):
    # 3.1. Fix this objective function.
    #      scipy.optimize has 'minimize', but not 'maximize', while we need to maximize the likelihood.
    #      We can easily convert a maximization problem to a minimization problem.
    def objective(x):
        return  log_likelihood_joint(x, landmark)

    return optimize.minimize(objective, # Objective function to minimize
                             prev_estimate # Initial guess. The robot is expected to be somewhere near the previous estimate.
                             ).x


# Constants.
WHEEL_RADIUS = 0.031
AXLE_LENGTH = 0.271756

# These are values that don't work well.
# Complete the following code & try different values to get a better result
sigma_x = 1
sigma_y = 1
sigma_th = 1
N_PARTICLE = 10


# 4. Predict
#    particle: [x, y, th]
#    u       : [wl, wr]
#    dt      : dt
def predict(particle, u, dt):
    prev_x  = particle[0]
    prev_y  = particle[1]
    prev_th = particle[2]

    wl = u[0]
    wr = u[1]

    # 4.1. Calculate the linear velocities of the left and right wheels.
    vl = _________________
    vr = _________________

    # 4.2. Calculate the linear and angular velocities of the robot
    v = _____________
    w = _______________________

    # 4.3. Calculate the robot's location & orientation
    th = prev_th + ______
    x  = prev_x  + _____________________
    y  = prev_y  + _____________________

    # Add gaussian noise
    th += random.gauss(0, sigma_th)
    x  += random.gauss(0, sigma_x)
    y  += random.gauss(0, sigma_y)

    return [x, y, th]

# 5. Particle Filter
#    You don't have anything to write/modify, but read the code.
def particle_filter(particle, landmark, u, dt):
    # if no particle, initialize particles
    if particle is None:
        # Particles are normally picked uniformly randomly but I don't want you to suffer.
        particle = np.hstack((np.random.normal(3.5,         sigma_x, size=(N_PARTICLE, 1)),
                              np.random.normal(0,           sigma_y, size=(N_PARTICLE, 1)),
                              np.random.normal(math.pi / 2, sigma_th, size=(N_PARTICLE, 1))))

    N, _ = np.shape(particle)

    # predict
    particle = np.vstack(tuple(predict(p, u, dt) for p in particle))

    # Calculate log-likelihood of each particle
    llh = np.array([log_likelihood_joint(p, landmark) for p in particle])

    # Likelihood as resampling weight.
    # Normalized to make max(llh) == 0.
    weight = np.exp(llh - max(llh))

    # Resample
    xk = np.arange(N)
    pk = [p/sum(weight) for p in weight]
    custm = stats.rv_discrete(values=(xk, pk))
    particle = particle[custm.rvs(size=N_PARTICLE), :]
    estimator = np.average(particle, axis=0)

    return estimator, particle