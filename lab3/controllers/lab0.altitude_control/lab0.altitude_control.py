from controller import Robot

from homework0 import control

import PID

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.widgets as wid

import math
import multiprocessing as mp
import random

class ProcessPlotter:
    def __init__(self):
        self.t = []
        self.r = []
        self.z = []

    def terminate(self):
        plt.close('all')

    def call_back(self):
        while self.pipe.poll():
            command = self.pipe.recv()
            if command is None:
                self.terminate()
                return False
            else: # append
                self.t.append(command[0])
                self.r.append(command[1])
                self.z.append(command[2])

        self.t = [t for t in self.t if t >= self.t[-1] - self.duration]
        del self.r[:(len(self.r) - len(self.t))]
        del self.z[:(len(self.z) - len(self.t))]

        if not self.t:
            return True

        self.lr.set_xdata(self.t)
        self.lr.set_ydata(self.r)
        self.lz.set_xdata(self.t)
        self.lz.set_ydata(self.z)
        self.ax.set_xlim([self.t[-1] - self.duration, self.t[-1]])
        self.ax.set_ylim([0, 40])

        self.fig.canvas.draw()
        return True

    def __call__(self, pipe, duration):
        self.pipe = pipe
        self.duration = duration

        self.fig, self.ax = plt.subplots()

        self.lr,  = self.ax.plot(self.t, self.r, label="ref")
        self.lz,  = self.ax.plot(self.t, self.z, label="z")
        self.ax.set_title('Altitude')
        self.ax.legend()

        timer = self.fig.canvas.new_timer(interval=200)
        timer.add_callback(self.call_back)
        timer.start()

        plt.show()

class MyController(Robot):
    def __init__(self, pipe):
        super().__init__()

        self.pipe = pipe

        self.gps  = self.getDevice("gps")
        self.imu  = self.getDevice("inertial unit")
        self.gyro = self.getDevice("gyro")

        self.motor = [
            self.getDevice("front left propeller"),
            self.getDevice("front right propeller"),
            self.getDevice("rear left propeller"),
            self.getDevice("rear right propeller"),
        ]

        self.getDevice("camera").enable(1)
        self.gps.enable(1)
        self.imu.enable(1)
        self.gyro.enable(1)

        for m in self.motor:
            m.setPosition(float('+inf'))
            m.setVelocity(0)

        self.pid_roll  = PID.PID(25, 0.1, 15)
        self.pid_pitch = PID.PID(25, 0.1, 15)
        self.pid_yaw   = PID.PID(1, 0, 1.5)

    def set_motor(self, w):
        croll, cpitch, cyaw = self.imu.getRollPitchYaw()

        def norm_rad(x):
            while x < -math.pi: x += 2 * math.pi
            while x >= math.pi: x -= 2 * math.pi
            return x

        u_roll  = self.pid_roll.get(-croll, self.getBasicTimeStep() / 1000.);
        u_pitch = self.pid_pitch.get(-cpitch, self.getBasicTimeStep() / 1000.);
        #u_yaw   = self.pid_yaw.get(norm_rad(-cyaw), self.getBasicTimeStep() / 1000.);
        u_yaw = 0

        w[0] +=  u_roll - u_pitch - u_yaw
        w[1] += -u_roll - u_pitch + u_yaw
        w[2] +=  u_roll + u_pitch + u_yaw
        w[3] += -u_roll + u_pitch - u_yaw

        for i in range(len(self.motor)):
            w[i] = max(min(w[i], self.motor[i].getMaxVelocity()), 0)

        self.motor[0].setVelocity( w[0])
        self.motor[1].setVelocity(-w[1])
        self.motor[2].setVelocity(-w[2])
        self.motor[3].setVelocity( w[3])

    def run(self):
        # while self.step(int(self.getBasicTimeStep())) != -1:
        #     pass
        # return 0

        rx = 0
        ry = 0
        rz = 0
        rroll = 0
        rpitch = 0
        ryaw = 0

        # warm up for 5 seconds
        self.set_motor([67, 67, 67, 67])
        if self.step(5000) == -1:
            self.pipe.send(None)
            return

        while True:
            if self.step(int(self.getBasicTimeStep())) == -1:
                self.pipe.send(None)
                return

            rz = float(self.getCustomData())

            dt = self.getBasicTimeStep() / 1000.
            cx, cy, cz = self.gps.getValues()
            vx, vy, vz = self.gps.getSpeedVector()
            px, py, pz = cx, cy, cz
            croll, cpitch, cyaw = self.imu.getRollPitchYaw()
            wroll, wpitch, wyaw = self.gyro.getValues()

            u = control(cx, cy, cz, croll, cpitch, cyaw,
                        vx, vy, vz, wroll, wpitch, wyaw,
                        rx, ry, rz, rroll, rpitch, ryaw,
                        self.getBasicTimeStep() / 1000)

            self.set_motor(u)
            self.pipe.send((self.getTime(), rz, cz))

if __name__ == '__main__':
    controller_pipe, plotter_pipe = mp.Pipe()
    plotter_process = mp.Process(target=ProcessPlotter(), args=(plotter_pipe, 120), daemon=True)
    plotter_process.start()

    controller = MyController(controller_pipe)
    controller.run()
