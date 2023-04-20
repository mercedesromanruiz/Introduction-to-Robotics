from controller import Supervisor

import numpy as np

import matplotlib.pyplot as plt

import math
import multiprocessing as mp
import random

class ProcessPlotter:
    def __init__(self):
        self.ground_truth = np.empty((0, 2))
        self.calculation  = np.empty((0, 2))
        self.error        = np.empty((0, 2))

    def terminate(self):
        plt.close('all')

    def call_back(self):
        while self.pipe.poll():
            command = self.pipe.recv()
            if command is None:
                self.terminate()
                return False
            else: # append
                time         = command[0]
                ground_truth = command[1]
                calculation  = command[2]
                error        = ground_truth - calculation

                self.ground_truth = np.vstack((self.ground_truth, [time, ground_truth]))
                self.calculation  = np.vstack((self.calculation,  [time, calculation]))
                self.error        = np.vstack((self.error,        [time, error]))

        if self.plot_ground_truth == None:
            self.ax1 = self.fig.add_subplot(2, 1, 1)
            self.plot_ground_truth = self.ax1.plot([], [], label="Ground Truth")[0]
            self.plot_calculation  = self.ax1.plot([], [], label="Calculation")[0]

            self.ax2 = self.fig.add_subplot(2, 1, 2)
            self.plot_error = self.ax2.plot([], [], label="Error")[0]


            self.ax1.set_xlabel('t (s)')
            self.ax1.set_ylabel('velocity (rad/s)')
            self.ax1.grid()
            self.ax1.legend()

            self.ax2.set_xlabel('t (s)')
            self.ax2.set_ylabel('error (rad/s)')
            self.ax2.grid()
            self.ax2.legend()

        if self.ground_truth.size != 0:
            tmax = np.max(self.ground_truth[:, 0])
            self.ground_truth = self.ground_truth[self.ground_truth[:, 0] > tmax - 10, :]
            self.calculation  = self.calculation [self.calculation [:, 0] > tmax - 10, :]
            self.error        = self.error       [self.error       [:, 0] > tmax - 10, :]

        self.plot_ground_truth.set_data(self.ground_truth.T)
        self.plot_calculation.set_data(self.calculation.T)
        self.plot_error.set_data(self.error.T)

        self.ax1.relim()
        self.ax1.autoscale_view(True,True,True)

        self.ax2.relim()
        self.ax2.autoscale_view(True,True,True)

        self.fig.canvas.draw()
        return True

    def __call__(self, pipe):
        self.pipe = pipe

        self.fig = plt.figure()

        self.plot_ground_truth = None
        self.plot_calculation  = None
        self.plot_error        = None

        timer = self.fig.canvas.new_timer(interval=50)
        timer.add_callback(self.call_back)
        timer.start()

        plt.show()


class MyController(Supervisor):
    def __init__(self, pipe):
        super().__init__()

        self.pipe = pipe

        self.motor = self.getDevice("rotational motor")
        self.sensor = self.getDevice("position sensor")

        self.sensor.enable(1)

        self.last_sensor_value = self.sensor.getValue()
        self.last_sensor_time  = self.getTime()

        self.encoder = self.getFromDef("ENCODER")

    def set_pwm_duty(self, duty):
        # We could simulate PWM, but we already know how PWM works.
        # Let's just simplify it.
        self.motor.setTorque((duty - 0.5) * 2 * self.motor.getMaxTorque())

    def display_velocity(self):
        # Calculate current velocity & update
        cur_pos = self.sensor.getValue()
        cur_time = self.getTime()
        vel = (cur_pos - self.last_sensor_value) / (cur_time - self.last_sensor_time)
        vel = 0 if math.isnan(vel) else vel
        self.last_sensor_value = cur_pos
        self.last_sensor_time  = cur_time

        encoder_text = self.encoder.getField("customData").getSFString()

        encoder_vel = float(encoder_text) if encoder_text else 0

        error = vel - encoder_vel

        self.setLabel(0, "Velocity (ground truth): {:.3f} rad/s".format(vel), 0, 0.85, 0.1, 0x000000, 0, "Arial Black")
        self.setLabel(1, "Velocity (encoder): {:.3f} rad/s".format(encoder_vel), 0, 0.90, 0.1, 0x000000, 0, "Arial Black")
        self.setLabel(2, "Error: {:.3f} rad/s".format(error), 0, 0.95, 0.1, 0x000000, 0, "Arial Black")

        self.pipe.send((self.getTime(),
                        vel,
                        encoder_vel))

    def run(self):
        random.seed()

        current_duty = random.uniform(0, 1)
        last_duty_changed = self.getTime()
        duty_period = 2

        self.set_pwm_duty(current_duty)

        while True:
            if self.step(int(self.getBasicTimeStep())) == -1:
                break

            # Update speed
            if self.getTime() - last_duty_changed > duty_period:
                current_duty = random.uniform(0, 1)
                last_duty_changed = self.getTime()
                self.set_pwm_duty(current_duty)

            # display ground truth
            self.display_velocity()

if __name__ == '__main__':
    controller_pipe, plotter_pipe = mp.Pipe()
    plotter_process = mp.Process(target=ProcessPlotter(), args=(plotter_pipe,), daemon=True)
    plotter_process.start()

    controller = MyController(controller_pipe)
    controller.run()
