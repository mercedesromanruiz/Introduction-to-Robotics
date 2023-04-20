import homework21

from controller import Robot

import sys

class CPU:
    INTERRUPT_RISING = 1
    INTERRUPT_FALLING = 2
    INTERRUPT_BOTH = INTERRUPT_RISING | INTERRUPT_FALLING

    def __init__(self, robot):
        self.__sleeping = False
        self.__robot = robot
        self.__interrupt_handlers = {}

        self.__old_sensor_value = None
        self.__sensor_value = None

    def __step(self):
        if robot.step(int(robot.getBasicTimeStep())) == -1:
            sys.exit()

        # update sensor values
        self.__old_sensor_value = self.__sensor_value
        self.__sensor_value = self.__robot.get_sensor_value()
        if self.__old_sensor_value == None:
            return

        # Handle interrupt
        for gpio_num, oc in enumerate(zip(self.__old_sensor_value, self.__sensor_value)):
            if not gpio_num in self.__interrupt_handlers:
                continue

            interrupt = self.__interrupt_handlers[gpio_num]
            old, cur = oc

            if (old == 0 and cur == 1 and interrupt['edge'] & self.INTERRUPT_RISING) or \
               (old == 1 and cur == 0 and interrupt['edge'] & self.INTERRUPT_FALLING):
                interrupt['handler']()

    def sleep(self):
        robot =self.__robot

        self.__sleeping = True
        while self.__sleeping == True:
            self.__step()

    def wakeup(self):
        self.__sleeping = False

    def set_interrupt(self, gpio_num, edge, handler):
        self.__interrupt_handlers[gpio_num] = {'edge': edge, 'handler': handler}

    def read_gpio(self, gpio_num):
        if gpio_num < 0 or gpio_num >= len(self.__sensor_value):
            return 0
        return self.__sensor_value[gpio_num]

    def get_time(self):
        return self.__robot.getTime()

    def send_uart(self, data):
        self.__robot.setCustomData(str(data))


class MyRobot(Robot):
    def __init__(self):
        super().__init__()

        num_led = 1 if len(sys.argv) < 2 else int(sys.argv[1])

        self.__sensor = [self.getDevice("led{}".format(i)) for i in range(num_led)]
        for s in self.__sensor:
            s.enable(1)

    def get_sensor_value(self):
        return [int(s.getValue() != 0) for s in self.__sensor]


if __name__ == "__main__":
    robot = MyRobot()
    cpu = CPU(robot)
    homework21.cpu = cpu
    homework21.program()
