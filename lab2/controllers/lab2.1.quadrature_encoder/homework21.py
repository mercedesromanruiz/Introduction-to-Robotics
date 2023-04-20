# Lab #2.1

# Settings
#   The encoder disk has 2 sets of 8 holes each.
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
#   The inner and outer sensors are connected to GPIO0 and GPIO1, respectively.

# Objectives
#   Calculate velocity in rad/s.
#   (extra) The calculated velocity will be fluctuating. Survey or bring your own idea.
#   (extra) Implment the idea.

import math

def interrupt():
    cpu.wakeup()

def program():
    prev_time = 0
    prev_num = 0

    cpu.set_interrupt(0, cpu.INTERRUPT_BOTH, interrupt)
    cpu.set_interrupt(1, cpu.INTERRUPT_BOTH, interrupt)

    while True:
        cpu.sleep()

        # Read current time and GPIOs
        ## YOUR CODE GOES HERE

        # Convert the GPIOs to the corresponding number
        ## YOUR CODE GOES HERE

        # Compare prev_num and cur_num to decide the direction
        if (prev_num + 1) % 4 == cur_num:
            direction = 1
        else:
            direction = -1

        # Calculate the speed
        ## YOUR CODE GOES HERE

        ## Combine speed and direction 
        velocity = speed * direction

        # Send the speed through uart
        cpu.send_uart(velocity)

        # Store current time and number
        prev_time = cur_time
        prev_num  = cur_num
