import math
from math import pow, log, pi

from vex import *

brain = Brain()
controller = Controller()
f_left_motor = Motor(Ports.PORT10, True)
b_left_motor = Motor(Ports.PORT20, True)
t_left_motor = Motor(Ports.PORT19, False)
f_right_motor = Motor(Ports.PORT1, False)
b_right_motor = Motor(Ports.PORT11, False)
t_right_motor = Motor(Ports.PORT12, True)
intake = Motor(Ports.PORT9, False)
inertial = Inertial(Ports.PORT2)
pto = DigitalOut(brain.three_wire_port.g)
wings = DigitalOut(brain.three_wire_port.h)


#left_motors = MotorGroup(f_left_motor, b_left_motor, t_left_motor)
#right_motors = MotorGroup(f_right_motor, b_right_motor, t_right_motor)


def pre_autonomous():
    brain.screen.print("pre auton")


def autonomous():
    brain.screen.print("auton")


def user_control():
    fly_wheel_pto = False
    pto.set(fly_wheel_pto)
    wings_extended = False
    wings.set(wings_extended)
    pto_active = False
    fly_buffer = False
    wing_buffer = False
    while (True):
        drive = controller.axis3.value() * .12
        turn = controller.axis1.value() * .12 * .5

        if (controller.buttonA.pressing()):
            #reg_degree_pid(90, 0.45, 5, 0.6, 0.35, 0.1)
            pass
        if abs(drive) + abs(turn) < .25:
            f_left_motor.stop()
            b_left_motor.stop()
            f_right_motor.stop()
            b_right_motor.stop()
            if(not pto_active):
                t_left_motor.stop()
                t_right_motor.stop()
        else:
            f_left_motor.spin(FORWARD, drive + turn, VoltageUnits.VOLT)
            f_right_motor.spin(FORWARD, drive - turn, VoltageUnits.VOLT)
            b_left_motor.spin(FORWARD, drive + turn, VoltageUnits.VOLT)
            b_right_motor.spin(FORWARD, drive - turn, VoltageUnits.VOLT)
            if(not pto_active):
                t_left_motor.spin(FORWARD, drive + turn, VoltageUnits.VOLT)
                t_right_motor.spin(FORWARD, drive - turn, VoltageUnits.VOLT)

        if controller.buttonX.pressing() and not fly_buffer:
            fly_wheel_pto = not fly_wheel_pto
            pto.set(fly_wheel_pto)
            fly_buffer = True
            if(not fly_wheel_pto and not wings_extended):
                pto_active = False
            else:
                pto_active = True
        elif(not controller.buttonX.pressing()):
            fly_buffer = False

        if controller.buttonR2.pressing() and not wing_buffer:
            wings_extended = not wings_extended
            wings.set(wings_extended)
            wing_buffer = True
            """
            if(not wings_extended and not wings_extended):
                pto_active = False
            else:
                pto_active = True
            """
        elif(not controller.buttonR2.pressing()):
            wings_extended = False

        if(fly_wheel_pto):
            if controller.buttonR1.pressing():
                t_left_motor.spin(FORWARD, 12, VoltageUnits.VOLT)
            else:
                t_left_motor.stop()

        if controller.buttonL1.pressing():
            intake.spin(REVERSE, 12, VoltageUnits.VOLT)
        elif controller.buttonL2.pressing():
            intake.spin(FORWARD, 12, VoltageUnits.VOLT)
        else:
            intake.stop()
        wait(20, MSEC)


comp = Competition(user_control, autonomous)
pre_autonomous()