# Lab #2.3

# Settings
#   The encoder disk has 7 sets of holes, using Gray code.
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
#   The sensors from inner to outer are connected from GPIO0 to GPIO6.

# Objectives
#   Calculate position in rad.

import math

def gray_to_bin(num):
    mask = num
    while mask:
        mask = mask >> 1
        num = num ^ mask
    return num

def interrupt():
    cpu.wakeup()

def program():
    for i in range(7):
        cpu.set_interrupt(i, cpu.INTERRUPT_BOTH, interrupt)

    while True:
        cpu.sleep()

        # Read GPIOs from 0 to 6 (Total 7 GPIOs)
        ## YOUR CODE GOES HERE

        # Convert the GPIO values (binary number) to a decimal number
        ## YOUR CODE GOES HERE

        # Decode the decimal number that is encoded in gray code
        num = gray_to_bin(num)

        # Convert the decimal number to angle (degree)
        ## YOUR CODE GOES HERE

        # Send the position through uart
        cpu.send_uart(pos)
