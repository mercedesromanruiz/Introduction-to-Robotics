from controller import Robot

import PID

import homework_localization
import homework_motion_planning

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.widgets as wid
import matplotlib.patches as patches

import math
import multiprocessing as mp
import random
import sys

class ProcessPlotter:
    def __init__(self):
        self.rng = [-1, 1, -1, 1]
        self.true_state = [0, 0, 0]
        self.estimated_state = [0, 0, 0]
        self.particle = [[]]
        self.prev_goal = None
        self.cur_goal = None

        self.ts_trail = np.empty((0, 3))
        self.es_trail = np.empty((0, 3))

        self.fig, self.ax = plt.subplots()

        self.plot_ts_trail = self.ax.plot([], [], color='red',   alpha=0.2)[0]
        self.plot_es_trail = self.ax.plot([], [], color='green', alpha=0.2)[0]
        self.plot_particle = self.ax.scatter([], [], s = 1, color='black', alpha=0.2)
        r = 0.3
        # true
        self.robot_t    = self.ax.add_patch(patches.Circle((0, 0), r, fill=False, color='red', label='true'))
        self.robot_tdir = self.ax.add_patch(patches.FancyArrow(0, 0, r, 0, color='red'))
        # estimated
        self.robot_e    = self.ax.add_patch(patches.Circle((0, 0), r, fill=False, color='green', label='estimate'))
        self.robot_edir = self.ax.add_patch(patches.FancyArrow(0, 0, r, 0, color='green'))
        # goal
        self.plot_goal  = None
        # obstacles
        self.plot_obs   = []
        # vector field
        self.plot_field = None

        self.ax.set_aspect('equal')
        self.ax.legend()

    def terminate(self):
        plt.close('all')

    def call_back(self):
        if self.q.empty():
            return True

        while not self.q.empty():
            command = self.q.get()

        if command is None:
            self.terminate()
            return False

        self.rng = command[0]
        self.true_state = command[1]
        self.estimated_state = command[2]
        self.particle = command[3]
        self.cur_goal = command[4]
        self.obstacles = command[5]

        self.ts_trail = np.vstack((self.ts_trail, self.true_state))
        self.es_trail = np.vstack((self.es_trail, self.estimated_state))

        # Draw obstacles
        for p in self.plot_obs[min(len(self.obstacles), len(self.plot_obs)):]:
            p.remove()
        self.plot_obs = self.plot_obs[:min(len(self.obstacles), len(self.plot_obs))]
        self.plot_obs += [self.ax.add_patch(patches.Circle((0, 0), 0, color='black')) for _ in range(len(self.obstacles) - len(self.plot_obs))]
        for i in range(len(self.obstacles)):
            self.plot_obs[i].set(center = (self.obstacles[i][0], self.obstacles[i][1]),
                                 radius = self.obstacles[i][2])

        # Draw potential field if goal has changed
        if self.cur_goal is not None and (self.prev_goal is None or np.any(self.prev_goal != self.cur_goal)):
            self.prev_goal = self.cur_goal

            if self.plot_goal is None:
                self.plot_goal  = self.ax.add_patch(patches.Circle((0, 0), 0.5, fill=False, color='blue', label='goal'))
            self.plot_goal.set(center=(self.cur_goal[0], self.cur_goal[1]))

            X = np.linspace(self.rng[0], self.rng[1], 50)
            Y = np.linspace(self.rng[2], self.rng[3], 50)
            U = []
            V = []
            for y in Y:
                u = []
                v = []
                for x in X:
                    # if xy is in obstacle, don't draw the arrow
                    min_dist = min([math.sqrt((o[0] - x)**2 + (o[1] - y)**2) - o[2] for o in self.obstacles])

                    if min_dist < homework_motion_planning.ROBOT_RADIUS:
                        u.append(0)
                        v.append(0)
                    else:
                        uv = homework_motion_planning.net_force(np.array([[x, y]]).T, self.cur_goal, self.obstacles)
                        if np.linalg.norm(uv) > 20:
                            uv = uv / np.linalg.norm(uv) * 20

                        u.append(uv[0, 0])
                        v.append(uv[1, 0])
                U.append(u)
                V.append(v)

            if self.plot_field is None:
                self.plot_field = self.ax.quiver(X, Y, U, V, alpha = 0.5)
            else:
                self.plot_field.set_UVC(U, V)

        self.plot_ts_trail.set_data(self.ts_trail[:,:2].T)
        self.plot_es_trail.set_data(self.es_trail[:,:2].T)
        if self.particle is not None:
            self.plot_particle.set_offsets(self.particle[:, :2])

        r = 0.3
        self.robot_t.set(center=(self.true_state[0], self.true_state[1]))
        self.robot_tdir.set_data(x  = self.true_state[0],               y  = self.true_state[1],
                                 dx = r * math.cos(self.true_state[2]), dy = r * math.sin(self.true_state[2]))

        self.robot_e.set(center=(self.estimated_state[0], self.estimated_state[1]))
        self.robot_edir.set_data(x  = self.estimated_state[0],               y  = self.estimated_state[1],
                                 dx = r * math.cos(self.estimated_state[2]), dy = r * math.sin(self.estimated_state[2]))

        self.ax.set_xlim([self.rng[0], self.rng[1]])
        self.ax.set_ylim([self.rng[2], self.rng[3]])

        self.fig.canvas.draw()
        return True

    def __call__(self, q):
        self.q = q

        timer = self.fig.canvas.new_timer(interval=50)
        timer.add_callback(self.call_back)
        timer.start()

        plt.show()

class MyController(Robot):
    def __init__(self, q):
        super().__init__()

        self.q = q

        self.gps  = self.getDevice("gps")
        self.imu  = self.getDevice("inertial unit")

        self.wheel_sensor = [
            self.getDevice("left wheel sensor"),
            self.getDevice("right wheel sensor"),
        ]

        self.motor = [
            self.getDevice("left wheel motor"),
            self.getDevice("right wheel motor"),
        ]

        self.gps.enable(1)
        self.imu.enable(1)

        for s in self.wheel_sensor:
            s.enable(1)

        for m in self.motor:
            m.setPosition(float('+inf'))
            m.setVelocity(0)

    def set_motor(self, w):
        for i in range(len(self.motor)):
            self.motor[i].setVelocity(max(min(w[i], self.motor[i].getMaxVelocity()), 0))

    def run(self):
        LIN_OMEGA = 10
        RADIUS = 3.5
        pid = PID.PID(2, 0.1, 20)
        prev_encoder = np.array([0, 0])

        # MLE
        # initial estimate
        prev_estimate = [3.5, 0, math.pi / 2]

        # Particle
        particle = None

        # Motion planning
        goal = np.array([[0, 3.5],
                         [-3.5, 0],
                         [0, -3.5],
                         [3.5, 0]])
        goal_idx = 0

        while True:
            if self.step(int(self.getBasicTimeStep())) == -1:
                self.q.put(None)
                return

            # get current loc & yaw
            cx, cy, _ = self.gps.getValues()
            _, _, cth = self.imu.getRollPitchYaw()

            # read encoder --> diff_encoder
            cur_encoder = np.array([s.getValue() for s in self.wheel_sensor])
            diff_encoder = cur_encoder - prev_encoder
            prev_encoder = cur_encoder

            # tokenize customData
            customData = [float(x) for x in self.getCustomData().split()]

            # Read range
            rng, customData = customData[0:4], customData[4:]

            # Read landmark information
            landmark = []
            for _ in range(int(customData.pop(0))):
                landmark.append([customData.pop(0), customData.pop(0), customData.pop(0)])

            # If no observation, don't do anything
            if len(landmark) == 0:
                continue

            # Read obstacle information
            obstacles = []
            for _ in range(int(customData.pop(0))):
                obstacles.append([customData.pop(0), customData.pop(0), customData.pop(0)])

            # update the goal
            if np.linalg.norm(np.array([cx, cy]) - goal[goal_idx]) < 0.5:
                goal_idx = (goal_idx + 1) % len(goal)

            # 0: MLE not moving
            # 1: MLE moving
            # 2: particle filter
            # 3: pf + motion planning

            # localization
            if len(sys.argv) < 2 or sys.argv[1] in ["0", "1"]: # MLE
                est = homework_localization.mle(landmark, prev_estimate)
                prev_estimate = est
            else:
                est, particle = homework_localization.particle_filter(particle, # particle
                                                                      landmark, # observation
                                                                      diff_encoder/(self.getBasicTimeStep() / 1000), # u
                                                                      self.getBasicTimeStep() / 1000) # dt

            # movement
            if len(sys.argv) < 2 or sys.argv[1] == "0": # MLE not moving
                pass
            elif sys.argv[1] in ["1", "2"]:
                # move the robot along a circle
                u = pid.get(RADIUS - math.sqrt(cx*cx+cy*cy), self.getBasicTimeStep() / 1000)
                self.set_motor([LIN_OMEGA + u, LIN_OMEGA - u])
            else:
                self.set_motor(homework_motion_planning.control(np.reshape(est, (3,1)),
                                                                goal[[goal_idx]].T,
                                                                np.array(obstacles),
                                                                self.getBasicTimeStep() / 1000
                                                                ))

            # chart
            g = None if (len(sys.argv) < 2 or sys.argv[1] in ["0", "1", "2"]) else goal[[goal_idx]].T
            self.q.put((rng, [cx, cy, cth], est, particle, g, np.array(obstacles)))

if __name__ == '__main__':
    q = mp.Queue()
    plotter_process = mp.Process(target=ProcessPlotter(), args=(q,), daemon=True)
    plotter_process.start()

    controller = MyController(q)
    controller.run()
