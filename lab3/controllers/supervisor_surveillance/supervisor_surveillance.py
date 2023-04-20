from controller import Supervisor

import math
import random

class MyController(Supervisor):
    def __init__(self):
        super().__init__()
        self.drone = self.getFromDef("DRONE")

    def run(self):
        radius = 40
        round_freq = 1. / 120

        alt_min = 5
        alt_max = 10
        alt_freq = 0.05

        tx = -40
        ty = 0
        tz = 1
        tyaw = 0

        self.drone.getField("customData").setSFString("{} {} {} {}".format(tx, ty, tz, tyaw))

        while True:
            if self.step(int(self.getBasicTimeStep())) == -1:
                break

            tx = -radius * math.cos(self.getTime() * 2 * math.pi * round_freq)
            ty = -radius * math.sin(self.getTime() * 2 * math.pi * round_freq)
            tz = alt_min + (alt_max - alt_min) * (math.sin(self.getTime() * 2 * math.pi * alt_freq) + 1) / 2

            tyaw = self.getTime() * round_freq * 2 * math.pi
            while tyaw >= math.pi: tyaw -= 2 * math.pi

            self.drone.getField("customData").setSFString("{} {} {} {}".format(tx, ty, tz, tyaw))


controller = MyController()
controller.run()
