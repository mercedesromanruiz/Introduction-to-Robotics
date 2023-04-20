from controller import Supervisor

import math
import random

class MyController(Supervisor):
    def __init__(self):
        super().__init__()
        self.drone  = self.getFromDef("DRONE")

    def run(self):
        period = 15
        last = self.getTime()
        target_altitude = 1
        target_yaw = 0
        self.drone.getField("customData").setSFString("{} {}".format(target_altitude, target_yaw))

        while True:
            if self.step(int(self.getBasicTimeStep())) == -1:
                break

            if self.getTime() > last + period:
                last = self.getTime()
                target_altitude = 10 + random.uniform(0, 20)
                target_yaw = random.uniform(-math.pi, math.pi)
                self.drone.getField("customData").setSFString("{} {}".format(target_altitude, target_yaw))

controller = MyController()
controller.run()
