
GAS_DENSITY = 2.858
ONE_MPH = 0.44704

from geometry_msgs.msg import Vector3
from yaw_controller import YawController
from lowpass import LowPassFilter
from pid import PID
from math import tan, sqrt
import rospy
import numpy as np

class Controller(object):
    def __init__(self,wheel_base, steer_ratio, max_lat_accel, max_steer_angle,dec2torque,decel_limit,accel_limit,wheel_radius):
        # TODO: Implement
        R_min = wheel_base/tan(max_steer_angle/steer_ratio)  # minimun radius of curvature followable by the car
        min_speed = sqrt(max_lat_accel*R_min)    #with this speed we garantee to use all the steering range
        self.time = rospy.Time.now()
        self.steer_ratio = steer_ratio
        self.wheel_base = wheel_base
        self.dec2torque = dec2torque
        self.acc2ratio  = 1/accel_limit
        self.yaw_ctrl   = YawController(wheel_base, steer_ratio, min_speed, max_lat_accel, max_steer_angle)
        self.acc_pid    = PID(0.5/0.02,0.5,0.02*0,decel_limit*0.8,accel_limit)
        self.stopState  = False
        #self.brake_pid = PID(1,0.2,0,decel_limit,-0.1)
        #self.acc_pid   = PID(-1,-0.1,-0.2)
        self.lastSpeed  = 0
        self.accFilt    = LowPassFilter(0.18,0.02)
        self.accel_limit = accel_limit
        self.decel_limit = decel_limit
        self.free_decel  = -9.81*0.02/wheel_radius
        pass

    def control(self, cmd_linear, cmd_angular, cur_linear,cur_angular, dbw_enabled):
	
        # TODO: Change the arg, kwarg list to suit your needs
        # Return throttle, brake, steer
        delta_t = 0.02 #(rospy.Time.now() - self.time).to_sec()
        #self.time = rospy.Time.now()

        
        # Compute the car acceleration
        saefty_margin = 0 # 1 m/s as safety margin
        cmd_linear.x = max(cmd_linear.x-saefty_margin,0)   #saturate the value
        rospy.loginfo("SPD_CTRL: cmd {0}  actual {1}".format(cmd_linear.x, cur_linear.x))
        #Compute the requested delta speed saturated to the max allowed value
        delta_speed = max(0.7*self.decel_limit*delta_t,min(cmd_linear.x - cur_linear.x,0.7*self.accel_limit*delta_t))        
        delta_speed = self.accFilt.filt(delta_speed)
        # Run the PID
        acc   = self.acc_pid.step(delta_speed, delta_t)
        
        # Compute the steering angle
        ang   = self.yaw_ctrl.get_steering(max(cmd_linear.x,0), cmd_angular.z, cur_linear.x)
	
        
        # Keep the vehicle stopped
        acc = acc-self.free_decel

        if cur_linear.x <0.01 and cmd_linear.x < 0.01: 
            rospy.loginfo("SPD_CTRL: STOP {0}".format(-1))
            brake = - 1 * self.dec2torque
            acc = 0
            self.acc_pid.reset() # Reset the PID           
            
        elif acc<0.1: #-0.4: 
            rospy.loginfo("SPD_CTRL: BRAKE {0}".format(acc))
            brake = - (acc-self.free_decel) * self.dec2torque
            acc = 0
            
        else:
            rospy.loginfo("SPD_CTRL: ACC {0}".format(acc))
            acc   = max(acc*self.acc2ratio,0)            
            brake = 0             
 
        if not dbw_enabled:
            self.acc_pid.reset()

        return acc, brake, ang


