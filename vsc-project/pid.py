import math
from vex import *

fleft_motor = Motor(2)
bleft_motor = Motor(3)
fright_motor = Motor(4)
bright_motor = Motor(6)
left_motor = MotorGroup(fleft_motor, bleft_motor)
right_motor = MotorGroup(fright_motor, bright_motor)
#distance between front and back motors
track_width = 0
drivetrain = DriveTrain(left_motor, right_motor, 4 * math.pi, track_width, units = DistanceUnits.MM)
gps = Inertial(13) #for skills replace the inertial with an actual GPS, we assume no gps avaliable for match auton

def rollover_subtract(last, cur):
    temp = cur-last
    if(temp < -350):
        return temp + 360
    elif(temp > 350):
        return temp - 360
    else:
        return temp
def sigmoid_distance(x, y):
    #this is 0.5tanh(5(x-0.5))+0.5 | 0
    zero_sig = 0.00669285092428
    return y - ((1 + zero_sig * 2) * (0.5 * math.tanh(5 * (x-0.5)) + 0.5) - zero_sig)
def der_sigmoid_distance(x,y):
    #this is just the analytical derivative of the above
    #if the original function is changed, change this too
    #2.533464255 sech^2((10x-5)*0.5)
    #math doesn't have sech by default, so we use reciprocal of cosh
    sec_c =  1 / (math.cosh((10 * x - 5) * 0.5))
    return y - (2.533464255 * sec_c * sec_c)
def int_sigmoid_distance(x,y):
    #this is just the analytical antiderivative of the above
    #1.013385702((0.5*x) + (ln(cosh(5(x-0.5)))) / 10) - x * 0.006692850924 (+c)
    return y - (1.013385702 * ((0.5*x) + (math.log(math.cosh(5 * (x-0.5)))) / 10) - x * 0.006692850924)
def reg_degree_pid(degrees, intended_time, min_err, kp, kd, ki):
    #kp,kd,ki should be between 0 and 1
    start_heading = gps.heading()
    max_time = intended_time * 1.2
    #intended_heading = start_heading + degrees
    last_heading = gps.heading()
    cur_heading = 0
    cur_time = 0
    yi = 0
    last_yp = 0
    #we have to keep track of oscillations for tuning
    osc_period = []
    osc_started = 0
    osc_start_time = 0
    err = degrees
    while(err > min_err and cur_time < max_time):
        cur_time += 0.1
        wait(100, MSEC)
        heading_diff = rollover_subtract(last_heading, gps.heading())
        cur_heading += heading_diff
        last_heading = gps.heading()
        err = degrees - cur_heading
        #calculating proportions
        x = cur_time / intended_time
        yp = cur_heading / degrees
        yd = heading_diff * 10
        #midpoint reimann sum to account for increasing nature of function
        #on second thought i'm dumb and accidentally implemented Tai's Model [Tai, 1994], so this is an over - estimante
        yi += (yp + last_yp) * 0.05
        cout = kp * sigmoid_distance(x,yp) + kd * der_sigmoid_distance(x,yd) *0.4 + kp * int_sigmoid_distance(x,yi) * 2
        drivetrain.set_turn_velocity(cout * 33.33, PERCENT)
        #osc logging
        if(osc_started == 0 and err < 0):
            osc_started = 1
            osc_start_time = cur_time
        if(osc_started == 1):
            if(err > 0):
                osc_started = 0
                osc_period.append(cur_time-osc_start_time)
    osc_mean = sum(osc_period) / len(osc_period)
    return [sum([abs(osc_mean - i) for i in osc_period]) / len(osc_period), osc_mean]
def autotune_degrees(degrees):
    #this is pretty fast but i'm a speedy gal what can i say
    intended_time = degrees / 45
    min_err = degrees / 100
    kp = 0.25
    kd = 0
    ki = 0
    last_osc = [1000, 0]
    while(True):
        cur_osc = reg_degree_pid(degrees, intended_time, min_err, kp, kd, ki)
        if(last_osc[0] > cur_osc[0]):
            last_osc = cur_osc
            kp += 0.05
        else:
            break
    return [0.6 * kp, last_osc[1] * 0.5, last_osc[1] * 0.125]
