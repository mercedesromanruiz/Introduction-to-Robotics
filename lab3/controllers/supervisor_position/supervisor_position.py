from controller import Supervisor

import math
import random

class MyController(Supervisor):
    def __init__(self):
        super().__init__()
        self.drone  = self.getFromDef("DRONE")
        self.target = self.getFromDef("TARGET")

    def run(self):
        period = 15
        last = self.getTime()
        target_x = 0
        target_y = 0
        target_z = 1
        target_yaw = 0

        self.drone.getField("customData").setSFString("{} {} {} {}".format(target_x, target_y, target_z, target_yaw))
        self.target.getField("translation").setSFVec3f([target_x, target_y, target_z])
        self.target.getField("rotation").setSFRotation([0, 0, 1, target_yaw])

        while True:
            if self.step(int(self.getBasicTimeStep())) == -1:
                break

            if self.getTime() > last + period:
                last = self.getTime()
                target_yaw = random.uniform(-math.pi, math.pi)
                target_x = random.uniform(-20, 20)
                target_y = random.uniform(-20, 20)
                target_z = 10 + random.uniform(0, 20)
                self.drone.getField("customData").setSFString("{} {} {} {}".format(target_x, target_y, target_z, target_yaw))
                self.target.getField("translation").setSFVec3f([target_x, target_y, target_z])
                self.target.getField("rotation").setSFRotation([0, 0, 1, target_yaw])

controller = MyController()
controller.run()
