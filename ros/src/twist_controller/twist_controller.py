from pid import PID
from yaw_controller import YawController
from lowpass import LowPassFilter
import time
import rospy


GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class TwistController(object):
    def __init__(self, *args):
    
    	self.yaw_controller = YawController(wheel_base, steer_ratio, min_speed, max_lat_accel, max_steer_angle)
    	self.lowpass_filter = LowPassFilter(0.7, 1.0)
    	self.set_controllers(self)
    	


    def control(self, desired_linear_velocity, desired_angular_velocity,
                current_linear_velocity, current_angular_velocity):
        # TODO: Change the arg, kwarg list to suit your needs
        # Return throttle, brake, steer
        
        err = desired_linear_velocity - current_linear_velocity
	throttle = self.pid_throttle.step(err, time_elapsed)
	brake = self.pid_brake.step(-err, time_elapsed)
        
        
        
        steer = self.yaw_controller.get_steering(
            desired_linear_velocity_modulated, error_angular_velocity,
            smoothed_current_linear_velocity)        
        return throttle, brake, steer



    #-------------Tweak Different values below-------------------------------------------------------------------    
    def set_controllers(self):
    	#PID Controllers for throttle and brake
    	self.pid_throttle = PID(0.0,0.0,0.0,0.0,0.0)
    	self.pid_brake = PID(0.0,0.0,0.0,0.0,0.0)
    	
    	#Low pass filter to smooth out the steering
    	self.lowpass_steer = LowPassFilter(0.0,0.0)    
