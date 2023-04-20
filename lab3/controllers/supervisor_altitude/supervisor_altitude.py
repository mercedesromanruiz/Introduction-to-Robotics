from controller import Supervisor

import numpy as np

import math
import random

class PID:
    def __init__(self, Kp, Ki, Kd):
        self.prev_e = 0
        self.int_e = 0
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd

    def get(self, e, dt):
        dedt = (e - self.prev_e) / dt
        int_e = 0.999 * self.int_e + e * dt

        self.prev_e = e
        self.int_e = int_e

        return self.Kp * e + self.Ki * int_e + self.Kd * dedt

class MyController(Supervisor):
    def __init__(self):
        super().__init__()
        self.drone  = self.getFromDef("DRONE")

        self.pzx = 0
        self.pzy = 0
        self.pid_x = PID(.1, 0.1, 0.1)
        self.pid_y = PID(.1, 0.1, 0.1)

    def fix_drone_orientation(self):
        ori = np.array(self.drone.getOrientation()).reshape((3, 3))
        zx = ori[0, 2]
        zy = ori[1, 2]

        ty = self.pid_x.get(zx, self.getBasicTimeStep()/1000)
        tx = self.pid_y.get(zy, self.getBasicTimeStep()/1000)

        self.drone.addTorque([tx, -ty, 0], False)

    def run(self):
        period = 15
        last = self.getTime()
        target_altitude = 1
        self.drone.getField("customData").setSFString("{}".format(target_altitude))

        while True:
            if self.step(int(self.getBasicTimeStep())) == -1:
                break

            if self.getTime() > last + period:
                last = self.getTime()

                ta = 10 + random.uniform(0, 20)

                diff = ta - target_altitude

                if diff > 5:
                    diff = 5
                elif diff < -5:
                    diff = -5

                target_altitude += diff

                self.drone.getField("customData").setSFString("{}".format(target_altitude))

            #self.fix_drone_orientation()

controller = MyController()
controller.run()
