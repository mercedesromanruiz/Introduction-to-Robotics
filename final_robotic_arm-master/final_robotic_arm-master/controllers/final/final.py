import homework as hw

from controller import Supervisor
import numpy as np

import math
import random
import sys

object_radius = 0.02
object_height = 0.002

object_string = """
Solid {{
  translation {} {} """ + str(object_height / 2) + """
  children [
    DEF SHAPE Shape {{
      appearance Appearance {{
        material Material {{
          diffuseColor {} {} {}
        }}
      }}
      geometry Cylinder {{
        height """ + str(object_height) + """
        radius """ + str(object_radius) + """
      }}
    }}
    Connector {{
      translation 0 0 """ + str(object_height / 2) + """
      rotation 0 1 0 -1.5708
      model "EM"
      type "passive"
      distanceTolerance 0.01
      numberOfRotations 0
    }}
  ]
  boundingObject USE SHAPE
  physics Physics {{
    density -1
    mass 0.005
  }}
  recognitionColors [
    {} {} {}
  ]
}}
"""

MODE_FWD_KIN    = "0"
MODE_INV_KIN    = "1"
MODE_PICK_PLACE = "2"

class MyController(Supervisor):
    def __init__(self):
        super().__init__()

        # get devices
        self.motor = [
            self.getDevice("shoulder_pan_joint"),
            self.getDevice("shoulder_lift_joint"),
            self.getDevice("elbow_joint"),
            self.getDevice("wrist_1_joint"),
            self.getDevice("wrist_2_joint"),
            self.getDevice("wrist_3_joint")
        ]

        self.sensor = [
            self.getDevice("shoulder_pan_joint_sensor"),
            self.getDevice("shoulder_lift_joint_sensor"),
            self.getDevice("elbow_joint_sensor"),
            self.getDevice("wrist_1_joint_sensor"),
            self.getDevice("wrist_2_joint_sensor"),
            self.getDevice("wrist_3_joint_sensor")
        ]

        self.cam = self.getDevice("camera")
        self.conn = self.getDevice("connector")

        # enable sensors
        for sensor in self.sensor:
            sensor.enable(int(self.getBasicTimeStep()))
        self.cam.enable(1)
        self.cam.recognitionEnable(1)

        if len(sys.argv) < 2:
            self.opmode = MODE_FWD_KIN
        else:
            self.opmode = sys.argv[1]

        # fwd_kin & inv_kin
        if self.opmode in [MODE_FWD_KIN, MODE_INV_KIN]:
            self.dest = self.getFromDef("DEST")

        # pick & place
        elif self.opmode in [MODE_PICK_PLACE]:
            # randomize N objects
            N = 10
            obj_locs = []

            def distance(loc1, loc2):
                dx = loc1[0] - loc2[0]
                dz = loc1[1] - loc2[1]
                return math.sqrt(dx * dx + dz * dz)

            def get_random_position():
                while True:
                    loc = [random.uniform(0.3, 0.85), random.uniform(-0.35, 0.35)]

                    if any(distance(o, loc) < 2 * object_radius for o in obj_locs):
                        continue

                    return loc

            for _ in range(N):
                obj_locs.append(get_random_position())

            # place objects
            root_children = self.getRoot().getField("children")

            color = [[1, 0, 0], [0, 1, 0], [0, 0, 1]]

            for loc in obj_locs:
                c = color[random.randint(0, 2)]
                root_children.importMFNodeFromString(-1, object_string.format(*loc, *c, *c))

    def get_rotation(self, ox, oy, oz):
        ox = (1 + ox) / 2
        oy /= 2
        oz /= 2

        d = math.sqrt(ox**2 + oy**2 + oz**2)

        return [ox / d, oy / d, oz / d, math.pi]

    def set_joint_angles(self, q):
        for i, th in enumerate(q):
            max_ang = math.pi if i == 2 else 2 * math.pi
            while th >= max_ang: th -= 2 * math.pi
            while th < -max_ang: th += 2 * math.pi
            self.motor[i].setPosition(th)

    def detect_object(self, r, g, b):
        objects = self.cam.getRecognitionObjects()
        objects = [objects[i] for i in range(self.cam.getRecognitionNumberOfObjects())]

        if not objects:
            return None

        for o in objects:
            c = o.getColors()
            rgb = [c[i] for i in range(3)]

            if rgb == [r, g, b]:
                ix, iy = o.getPositionOnImage()
                return [ix, iy]

        return None

    def sleep_for(self, t):
        if self.step(int(t * 1000)) == -1:
            raise Exception()

    def run(self):
        if self.opmode == MODE_FWD_KIN:
            INTERVAL = 3
            last_change = -INTERVAL

            while self.step(1) != -1:
                if self.getTime() >= last_change + INTERVAL:
                    last_change += INTERVAL
                    q = [random.uniform(-math.pi, math.pi) for _ in range(len(self.motor))]
                    M = hw.fwd_kin(q)

                    self.dest.getField("translation").setSFVec3f(list(M[:3, 3]))
                    self.dest.getField("rotation").setSFRotation(self.get_rotation(*M[:3, 2]))

                    self.set_joint_angles(q)

        elif self.opmode == MODE_INV_KIN:
            INTERVAL = 3
            last_change = -INTERVAL

            while self.step(1) != -1:
                if self.getTime() >= last_change + INTERVAL:
                    last_change += INTERVAL
                    loc = [random.uniform(-0.5, 0.5),
                           random.uniform(-0.5, 0.5),
                           random.uniform(0, 0.5)]
                    z_axis = np.array([random.uniform(-1, 1),
                                       random.uniform(-1, 1),
                                       random.uniform(-1, 1)])
                    z_axis = list(z_axis / np.linalg.norm(z_axis))

                    q = hw.inv_kin_z([s.getValue() for s in self.sensor], loc, z_axis)

                    self.dest.getField("translation").setSFVec3f(loc)
                    self.dest.getField("rotation").setSFRotation(self.get_rotation(*z_axis))

                    self.set_joint_angles(q)

        elif self.opmode == MODE_PICK_PLACE:
            hw.control(lambda : [s.getValue() for s in self.sensor], # get_joint_angles
                       self.set_joint_angles,                        # set_joint_angles
                       self.detect_object,                           # detect_object
                       self.cam.getWidth(),
                       self.cam.getHeight(),
                       self.conn.lock,                               # grip
                       self.conn.unlock,                             # release
                       self.sleep_for                                # sleep_for
                       )

if __name__ == '__main__':
    controller = MyController()
    controller.run()
