from pid import PID
from yaw_controller import YawController
from lowpass import LowPassFilter
import time
import rospy


GAS_DENSITY = 2.858
ONE_MPH = 0.44704
MAX_SPEED = 25.0 * ONE_MPH #in mps


class Controller(object):

    def __init__(self, wheel_base, vehicle_mass, steer_ratio, min_speed, max_lat_accel, max_steer_angle):
    #def __init__(self, *args, **kwargs):

        self.yaw_controller = YawController(wheel_base, steer_ratio, min_speed, max_lat_accel, max_steer_angle)
        self.set_controllers()
        self.prev_time = None

    def control(self, desired_linear_velocity, desired_angular_velocity,
                current_linear_velocity, current_angular_velocity):
        if self.prev_time is None:
            self.prev_time = time.time()
            return 0., 0., 0.

        # TODO: Change the arg, kwarg list to suit your needs
        # Return throttle, brake, steer
        desired_linear_velocity = min(MAX_SPEED, desired_linear_velocity)
        err = desired_linear_velocity - current_linear_velocity
        time_elapsed = time.time() - self.prev_time
        if err < 0:
            brake = self.pid_brake.step(-err, time_elapsed)
            brake = max(min(brake, 1.0), 0.0)
            throttle = 0
        else:
            throttle = self.pid_throttle.step(err, time_elapsed)
            throttle = max(min(throttle, 1.0), 0.0)
            #throttle = 0.5
            brake = 0

        steer = self.yaw_controller.get_steering(
                    desired_linear_velocity,
                    desired_angular_velocity,
                    current_linear_velocity)
        #steer = self.lowpassfilter_steer.filt(steer)
        return throttle, brake, steer

    #-------------Tweak Different values below-------------------------------------------------------------------
    def set_controllers(self):
        #PID Controllers for throttle and brake
        self.pid_throttle = PID(0.05,1,0.1,0.0,0.7)
        self.pid_brake = PID(0.05,1,0.1,0.0,1.0)

        #Low pass filter to smooth out the steering
        self.lowpassfilter_steer = LowPassFilter(2,1)
