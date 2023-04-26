from controller import Supervisor

import math
import random
import sys

class MyController(Supervisor):
    minX = -5
    maxX = 5
    minY = -5
    maxY = 5
    N = 20
    sigma = 0.2

    def __init__(self):
        super().__init__()

        self.robot = self.getFromDef("ROBOT")

        # randomized landmark locations
        xy = [[random.random() * (self.maxX - self.minX) + self.minX,
               random.random() * (self.maxY - self.minY) + self.minY]
              for _ in range(self.N)]
        # put 4 landmarks in the corners
        xy[0] = [self.minX, self.minY]
        xy[1] = [self.minX, self.maxY]
        xy[2] = [self.maxX, self.minY]
        xy[3] = [self.maxX, self.maxY]

        self.landmark = xy

        children = self.getFromDef("LANDMARK").getField("children")
        for _ in range(children.getCount()):
            children.removeMF(-1)

        for [x, y] in self.landmark:
            str = \
    """Transform {{
      translation {} {} 0.1
      children [
        Shape {{
          appearance Appearance {{
            material Material {{
              diffuseColor 1 0 0
              transparency 0.5
            }}
          }}
          geometry Box {{
            size 0.1 0.1 0.1
          }}
        }}
      ]
    }}
    """
            children.importMFNodeFromString(-1, str.format(x, y))

        # Obstacles
        self.obstacle = []

        if len(sys.argv) < 2 or sys.argv[1] != "1":
            return

        self.obstacle = [[1.6, 1.6, 0.2],
                         [1.6, -1.6, 0.4],
                         [-1.6, 1.6, 0.4],
                         [-1.6, -1.6, 0.2],
                         ]

        children = self.getFromDef("OBSTACLE").getField("children")
        for _ in range(children.getCount()):
            children.removeMF(-1)

        for [x, y, r] in self.obstacle:
            str = \
    """Transform {{
      translation {} {} 0.25
      children [
        Shape {{
          appearance Appearance {{
            material Material {{
            }}
          }}
          geometry Cylinder {{
            height 0.5
            radius {}
          }}
        }}
      ]
    }}
    """
            children.importMFNodeFromString(-1, str.format(x, y, r))

    def run(self):
        def distance(cur, xy):
            d0 = cur[0] - xy[0]
            d1 = cur[1] - xy[1]
            return math.sqrt(d0 * d0 + d1 * d1)

        while True:
            if self.step(int(self.getBasicTimeStep())) == -1:
                break

            str = "{} {} {} {} {}".format(self.minX, self.maxX, self.minY, self.maxY, self.N)

            x,y,_ = self.robot.getField("translation").getSFVec3f()
            cur = [x, y]

            for i in range(self.N):
                str += " {} {} {}".format(self.landmark[i][0], self.landmark[i][1],
                                          distance(cur, self.landmark[i]) + random.gauss(0, self.sigma))

            str += " {}".format(len(self.obstacle))
            for o in self.obstacle:
                str += " {} {} {}".format(o[0], o[1], o[2])

            self.robot.getField("customData").setSFString(str)

if __name__ == '__main__':
    controller = MyController()
    controller.run()
