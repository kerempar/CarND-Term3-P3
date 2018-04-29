from pid import PID
from lowpass import LowPassFilter
from yaw_controller import YawController
import rospy

# use pid and lowpass for acceleration
# use yaw_controller for steering

GAS_DENSITY = 2.858

class Controller(object):
    def __init__(self, vehicle_mass, fuel_capacity, brake_deadband, wheel_radius, wheel_base, steer_ratio, 
                 min_speed, max_lat_accel, max_steer_angle, decel_limit, accel_limit):
        
        self.vehicle_mass = vehicle_mass
        self.fuel_capacity = fuel_capacity
        self.brake_deadband = brake_deadband
        self.wheel_radius = wheel_radius
        self.wheel_base = wheel_base
        self.steer_ratio = steer_ratio
        self.min_speed = min_speed
        self.max_lat_accel = max_lat_accel
        self.max_steer_angle = max_steer_angle
        self.decel_limit = decel_limit
        self.accel_limit = accel_limit
       
        # TODO: Implement
        self.yaw_controller = YawController(wheel_base=self.wheel_base, 
                                            steer_ratio=self.steer_ratio, 
                                            min_speed=self.min_speed,
                                            max_lat_accel=self.max_lat_accel,
                                            max_steer_angle=self.max_steer_angle)
        kp=0.3
        ki=0.1
        kd=0.
        mn=0    #minimum throttle value
        mx=0.2  #maximum throttle value
        self.throttle_controller = PID(kp, ki, kd, mn, mx)
        tau=0.5 #cutoff frequency
        ts=.02  #sample time
        self.vel_lpf = LowPassFilter(tau, ts)
        
        self.last_time = rospy.get_time()

    def reset(self):
        self.throttle_controller.reset()

    def control(self, current_vel, dbw_enabled, linear_vel, angular_vel):
        # TODO: Change the arg, kwarg list to suit your needs
        
        if not dbw_enabled:
           self.throttle_controller.reset()
           return 0., 0., 0.

        current_vel = self.vel_lpf.filt(current_vel.twist.linear.x)
        
        steering = self.yaw_controller.get_steering(linear_vel, angular_vel, current_vel)
        
        vel_error = linear_vel - current_vel
        self.last_vel = current_vel
        
        current_time = rospy.get_time()
        sample_time = current_time - self.last_time
        self.last_time = current_time

        throttle = self.throttle_controller.step(vel_error, sample_time)
        brake = 0
        
        if linear_vel == 0 and current_vel < 0.1:
           throttle = 0
           brake = 400  #to hold the car in place if we are stpped at a light.
        
        elif throttle < .1 and vel_error < 0:
           throttle = 0
           decel = max(vel_error, self.decel_limit)
           brake = abs(decel)*self.vehicle_mass*self.wheel_radius  #torque

           #brake = deceleration * (self.vehicle_mass + self.fuel_capacity * GAS_DENSITY) * self.wheel_radius

        return throttle, brake, steering
