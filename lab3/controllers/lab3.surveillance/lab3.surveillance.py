from controller import Robot

from homework3 import control

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.widgets as wid

import math
import multiprocessing as mp
import random

class ProcessPlotter:
    def __init__(self):
        self.t = []
        self.rz = []
        self.z = []
        self.rroll = []
        self.roll = []
        self.rpitch = []
        self.pitch = []
        self.ryaw = []
        self.yaw = []
        self.rx = []
        self.x = []
        self.ry = []
        self.y = []

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
                self.rz.append(command[1])
                self.z.append(command[2])
                self.rroll.append(command[3])
                self.roll.append(command[4])
                self.rpitch.append(command[5])
                self.pitch.append(command[6])
                self.ryaw.append(command[7])
                self.yaw.append(command[8])
                self.rx.append(command[9])
                self.x.append(command[10])
                self.ry.append(command[11])
                self.y.append(command[12])

        self.t = [t for t in self.t if t >= self.t[-1] - self.duration]
        del self.rz[:(len(self.rz) - len(self.t))]
        del self.z [:(len(self.z)  - len(self.t))]
        del self.rroll[:(len(self.rroll) - len(self.t))]
        del self.roll [:(len(self.roll)  - len(self.t))]
        del self.rpitch[:(len(self.rpitch) - len(self.t))]
        del self.pitch [:(len(self.pitch)  - len(self.t))]
        del self.ryaw[:(len(self.ryaw) - len(self.t))]
        del self.yaw [:(len(self.yaw)  - len(self.t))]
        del self.rx[:(len(self.rx) - len(self.t))]
        del self.x [:(len(self.x)  - len(self.t))]
        del self.ry[:(len(self.ry) - len(self.t))]
        del self.y [:(len(self.y)  - len(self.t))]

        if not self.t:
            return True

        self.lrz.set_xdata(self.t)
        self.lrz.set_ydata(self.rz)
        self.lz.set_xdata(self.t)
        self.lz.set_ydata(self.z)
        self.axs[0,0].set_xlim([self.t[-1] - self.duration, self.t[-1]])
        self.axs[0,0].set_ylim([0, 15])

        self.lrroll.set_xdata(self.t)
        self.lrroll.set_ydata(self.rroll)
        self.lroll.set_xdata(self.t)
        self.lroll.set_ydata(self.roll)
        self.axs[0,1].set_xlim([self.t[-1] - self.duration, self.t[-1]])
        self.axs[0,1].set_ylim([-15, 15])

        self.lrpitch.set_xdata(self.t)
        self.lrpitch.set_ydata(self.rpitch)
        self.lpitch.set_xdata(self.t)
        self.lpitch.set_ydata(self.pitch)
        self.axs[1,0].set_xlim([self.t[-1] - self.duration, self.t[-1]])
        self.axs[1,0].set_ylim([-15, 15])

        self.lryaw.set_xdata(self.t)
        self.lryaw.set_ydata(self.ryaw)
        self.lyaw.set_xdata(self.t)
        self.lyaw.set_ydata(self.yaw)
        self.axs[1,1].set_xlim([self.t[-1] - self.duration, self.t[-1]])
        self.axs[1,1].set_ylim([-180, 180])

        self.lrx.set_xdata(self.t)
        self.lrx.set_ydata(self.rx)
        self.lx.set_xdata(self.t)
        self.lx.set_ydata(self.x)
        self.axs[2,0].set_xlim([self.t[-1] - self.duration, self.t[-1]])
        self.axs[2,0].set_ylim([-45, 45])

        self.lry.set_xdata(self.t)
        self.lry.set_ydata(self.ry)
        self.ly.set_xdata(self.t)
        self.ly.set_ydata(self.y)
        self.axs[2,1].set_xlim([self.t[-1] - self.duration, self.t[-1]])
        self.axs[2,1].set_ylim([-45, 45])

        self.fig.canvas.draw()
        return True

    def __call__(self, pipe, duration):
        self.pipe = pipe
        self.duration = duration

        self.fig, self.axs = plt.subplots(3, 2)

        self.lrz, = self.axs[0,0].plot(self.t, self.rz, label="ref")
        self.lz,  = self.axs[0,0].plot(self.t, self.z, label="z")
        self.axs[0,0].set_title('Z (m)')
        self.axs[0,0].legend()

        self.lrroll, = self.axs[0,1].plot(self.t, self.rroll, label="ref")
        self.lroll,  = self.axs[0,1].plot(self.t, self.roll, label="roll")
        self.axs[0,1].set_title('Roll (deg)')
        self.axs[0,1].legend()

        self.lrpitch, = self.axs[1,0].plot(self.t, self.rpitch, label="ref")
        self.lpitch,  = self.axs[1,0].plot(self.t, self.pitch, label="pitch")
        self.axs[1,0].set_title('Pitch (deg)')
        self.axs[1,0].legend()

        self.lryaw, = self.axs[1,1].plot(self.t, self.ryaw, label="ref")
        self.lyaw,  = self.axs[1,1].plot(self.t, self.yaw, label="yaw")
        self.axs[1,1].set_title('Yaw (deg)')
        self.axs[1,1].legend()

        self.lrx, = self.axs[2,0].plot(self.t, self.rx, label="ref")
        self.lx,  = self.axs[2,0].plot(self.t, self.x, label="x")
        self.axs[2,0].set_title('X (m)')
        self.axs[2,0].legend()

        self.lry, = self.axs[2,1].plot(self.t, self.ry, label="ref")
        self.ly,  = self.axs[2,1].plot(self.t, self.y, label="y")
        self.axs[2,1].set_title('Y (m)')
        self.axs[2,1].legend()

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

    def set_motor(self, w):
        for i in range(len(self.motor)):
            w[i] = max(min(w[i], self.motor[i].getMaxVelocity()), 0)

        self.motor[0].setVelocity(w[0])
        self.motor[1].setVelocity(-w[1])
        self.motor[2].setVelocity(-w[2])
        self.motor[3].setVelocity(w[3])

    def run(self):
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

            rx, ry, rz, ryaw = [float(x) for x in self.getCustomData().split()]

            dt = self.getBasicTimeStep() / 1000.
            cx, cy, cz = self.gps.getValues()
            vx, vy, vz = self.gps.getSpeedVector()
            croll, cpitch, cyaw = self.imu.getRollPitchYaw()
            wroll, wpitch, wyaw = self.gyro.getValues()

            u = control(cx, cy, cz, croll, cpitch, cyaw,
                        vx, vy, vz, wroll, wpitch, wyaw,
                        rx, ry, rz, rroll, rpitch, ryaw,
                        self.getBasicTimeStep() / 1000)

            self.set_motor(u)
            self.pipe.send((self.getTime(),
                            rz, cz,
                            rroll * 180 / math.pi, croll * 180 / math.pi,
                            rpitch * 180 / math.pi, cpitch * 180 / math.pi,
                            ryaw * 180 / math.pi, cyaw * 180 / math.pi,
                            rx, cx,
                            ry, cy))

if __name__ == '__main__':
    controller_pipe, plotter_pipe = mp.Pipe()
    plotter_process = mp.Process(target=ProcessPlotter(), args=(plotter_pipe, 120), daemon=True)
    plotter_process.start()

    controller = MyController(controller_pipe)
    controller.run()
