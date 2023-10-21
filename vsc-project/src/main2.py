"""
the entire pid lib
"""
import math
from math import pow, log, pi

from vex import *
fleft_motor = Motor(Ports.PORT5, True)
bleft_motor = Motor(Ports.PORT6, True)
fright_motor = Motor(Ports.PORT21)
bright_motor = Motor(Ports.PORT3)
left_motor = MotorGroup(fleft_motor, bleft_motor)
right_motor = MotorGroup(fright_motor, bright_motor)
#distance between front and back motors
track_width = 0
drivetrain = DriveTrain(left_motor, right_motor, 4 * pi, track_width, units = DistanceUnits.MM)
inertial = Inertial(Ports.PORT19) #for skills replace the inertial with an actual inertial, we assume no inertial avaliable for match auton


def rollover_subtract(last, cur):
    temp = cur-last
    if(temp < -350):
        return temp + 360
    elif(temp > 350):
        return temp - 360
    else:
        return temp
def sigmoid_distance(x, y):
    #this is 1 / (1 + e^(-10(x-0.5)))
    e = 2.718281828459045
    return y - (1 / (math.pow(e, -10 * (x - 0.5)) + 1))
def der_sigmoid_distance(x,y):
    #this is just the analytical derivative of the above
    #if the original function is changed, change this too
    #(10e^-(10(x-0.5)))/(e^(-10(x-0.5)) + 1)^2
    #sec_c =  1 / (math.cosh((10 * x - 5) * 0.5))
    e = 2.718281828459045
    return y - (10 * math.pow(e, 10 * x + 5))/((math.pow(e, 10 * x) + math.pow(e, 5)) * (math.pow(e, 10 * x) + math.pow(e, 5)))
def int_sigmoid_distance(x,y):
    #this is just the analytical antiderivative of the above
    e = 2.718281828459045
    return y - (math.log(math.pow(e, 5 - 10 * x) + 1) * 0.1) + x - 0.5
def reg_degree_pid(degrees, intended_time, min_err, kp, kd, ki):
    #kp,kd,ki should be between 0 and 1
    #inertial.calibrate()
    #inertial.set_heading(90)
    max_time = intended_time * 1.4
    #intended_heading = start_heading + degrees
    inertial.set_heading(0)
    last_heading = inertial.heading(DEGREES)
    print(last_heading)
    cur_heading = 0
    cur_time = 0
    yi = 0
    last_yp = 0
    #we have to keep track of oscillations for tuning
    osc_period = []
    osc_started = 0
    osc_start_time = 0
    err = degrees
    while(abs(err) > min_err and cur_time < max_time):
        cur_time += 0.01
        
        heading_diff = rollover_subtract(last_heading, inertial.heading(DEGREES))
        cur_heading += heading_diff
        last_heading = inertial.heading(DEGREES)
        print("(" + str(cur_time) + ", " + str(last_heading / 90) + ")")
        err = degrees - cur_heading
        #calculating proportions
        x = cur_time / intended_time
        yp = cur_heading / degrees
        yd = heading_diff * 10
        #midpoint reimann sum to account for increasing nature of function
        #on second thought i'm dumb and accidentally implemented Tai's Model [Tai, 1994], so this is an over - estimante
        yi += (yp + last_yp) * 0.05
        cout = kp * sigmoid_distance(x,yp) + kd * der_sigmoid_distance(x,yd) *0.4 + ki * int_sigmoid_distance(x,yi) * 2
        if(abs(cout) > 3.3):
            cout = cout / abs(cout) * 3.3
        if(cout < 0):
            left_motor.spin(FORWARD, cout * 33, VelocityUnits.PERCENT)
            right_motor.spin(REVERSE, cout * 33, VelocityUnits.PERCENT)
        else:
            left_motor.spin(REVERSE, cout * 33, VelocityUnits.PERCENT)
            right_motor.spin(FORWARD, cout * 33, VelocityUnits.PERCENT)
            
        #osc logging
        #print("cout  " + str(cout))
        #print(err)
        """
        if(osc_started == 0 and err < 0):
            osc_started = 1
            osc_start_time = cur_time
        else:
            pass
            #print("heading")
            #print(last_heading)
            #print(inertial.is_calibrating())
        if(osc_started == 1):
            if(err > 0):
                osc_started = 0
                osc_period.append(cur_time-osc_start_time)
        """
        wait(10, MSEC)
    drivetrain.stop()
    print(inertial.heading(DEGREES))
    print(cur_heading)
    print("err:  " + str(err))
    drivetrain.stop()
    return err
"""
    if(len(osc_period) != 0):
        osc_mean = sum(osc_period) / len(osc_period)
        return [sum([abs(osc_mean - i) for i in osc_period]) / len(osc_period), osc_mean]
    else:
        return [9999,9999999,9999999999999]
"""

def autotune_degrees(degrees):
    #this is pretty fast but i'm a speedy gal what can i say
    intended_time = degrees / 45.0
    min_err = degrees / 100.0
    kp = 0.5
    kd = 0.25
    ki = 0.0
    #last_osc = [100000, 0]
    last_err = 10000
    while(True):
        cur_err = reg_degree_pid(degrees, intended_time, min_err, kp, kd, ki)
        #drivetrain.stop()
        wait(100, MSEC)
        print("ran a test")
        print(inertial.heading(DEGREES))
        if(last_err >= cur_err):
            last_err = cur_err
            """
            kp += 0.05
            kd += 0.05
            ki += 0.05

            bottom converges on the k values beter
            """
            kp += 0.05*last_err
            kd += 0.05*last_err
            ki += 0.05*last_err
            print("hes not getting bizzy, he getting rizzy")
        else:
            break
    return [ kp, kd, ki]



# ---------------------------------------------------------------------------- #
#                                                                              #
# 	Module:       main.py                                                      #
# 	Author:       xavie                                                        #
# 	Created:      9/17/2023, 10:48:44 AM                                       #
# 	Description:  V5 project                                                   #
#                                                                              #
# ---------------------------------------------------------------------------- #

# Library imports

# Brain should be defined by default
brain=Brain()
controller = Controller()

#reg_degree_pid(90,2,9,0.15,0,0)
#drivetrain.stop()
#inertial.set_heading(90)
#drivetrain.turn(LEFT, 90, VelocityUnits.PERCENT)
wait(5000, MSEC)
inertial.heading(DEGREES)
#temp = autotune_degrees(90)
reg_degree_pid(90, 0.65, 5, 0.6,0.35,0.1)
temp = [0,0,0]
brain.screen.print("Hello V5")
#brain.screen.print("\n" + str(sigmoid_distance(0.5, 0)) + "  " + str(der_sigmoid_distance(0.5, 0)) + "  " + str(int_sigmoid_distance(0.5, 0)))
#print("\n" + str(sigmoid_distance(0.5, 0)) + "  " + str(der_sigmoid_distance(0.5, 0)) + "  " + str(int_sigmoid_distance(0.5, 0)))
brain.screen.print(" " + str(temp[0] )+ " " + str(temp[1] )+ " " + str(temp[2] )+ " ")
print(str(temp[0]) + " " + str(temp[1]) + " " + str(temp[2]))
while(True):
    if(controller.buttonA.pressing()):
        reg_degree_pid(90, 0.45, 5, 0.6,0.35,0.1)
    wait(10,MSEC)

        
