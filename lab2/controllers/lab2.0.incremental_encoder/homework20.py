# Lab #2.0

# THIS IS AN EXAMPLE.

# Settings
#   The encoder disk has 16 holes.
#   You're given a virtual CPU.
#     - cpu.sleep(): sleeps until awaken.
#     - cpu.wakeup(): wakes up CPU.
#     - cpu.set_interrupt(gpio_num, edge, handler): sets interrupt
#         on GPIO #gpio_num
#         for edge type
#           cpu.INTERRUPT_RISING
#           cpu.INTERRUPT_FALLING
#           cpu.INTERRUPT_BOTH
#         to call the handler.
#     - cpu.read_gpio(gpio_num): reads GPIO #gpio_num
#     - cpu.get_time(): reads the current time
#     - cpu.send_uart(data): sends data via uart
#         Send your calculation with this function.
#   A sensor is connected to GPIO0.

# Objectives
#   Calculate speed in rad/s.
#   (practice) The given code measures time between 2 recent rising edges.
#              Make hypotheses: expect what happens if we measure time between 2 recent both (rising and falling) edges.
#              Comment & uncomment the code, observe what happens, verity your hypotheses, and analyze the reasons.
#   (practice) The calculated speed will be fluctuating. Survey and implement your ideas to reduce the error.

import math

# This is called by CPU when GPIO input change.
def interrupt():
    # Some GPIO pins changed. Let's wake up CPU.
    cpu.wakeup()

def program():
    # variable to store previous time point
    prev_time = 0

    # Ask the CPU to call the above function when GPIO 0 changes.
    cpu.set_interrupt(0, cpu.INTERRUPT_RISING, interrupt) # Interrupt when GPIO0:L->H
    # cpu.set_interrupt(0, cpu.INTERRUPT_BOTH, interrupt) # Interrupt when GPIO0:L->H or GPIO0:H->L

    while True:
        # We don't have any thing to do now until interrupt, so let's sleep.
        cpu.sleep()

        # If we're here, it means that the CPU Woke up.

        # Get current time and measure time difference dt.
        cur_time = cpu.get_time()
        dt = cur_time - prev_time

        # Calculate speed using dt.
        speed = math.pi / 8 / dt # For INTERRUPT_RISING
        # speed = math.pi / 16 / dt # For INTERRUPT_BOTH

        # Send the result to UART
        cpu.send_uart(speed)

        # store the current time so that we can calculate the speed at the next GPIO change.
        prev_time = cur_time
