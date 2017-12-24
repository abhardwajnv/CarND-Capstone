import rospy
from pid import PID
from yaw_controller import YawController

GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class Controller(object):
    def __init__(self,vehicle_mass, fuel_capacity, brake_deadband, decel_limit,\
                 accel_limit, wheel_radius, wheel_base, steer_ratio, \
                 max_lat_accel, max_steer_angle):
        self.vehicle_mass = vehicle_mass
        self.fuel_capacity  = fuel_capacity
        self.brake_deadband = brake_deadband
        self.decel_limit  = decel_limit
        self.accel_limit  = accel_limit
        self.wheel_radius = wheel_radius

        #def __init__(self, kp, ki, kd, mn=MIN_NUM, mx=MAX_NUM):
        self.pid = PID(kp = 1.5, ki = 0., kd = 0.02)
        self.yaw_controller = YawController(wheel_base, steer_ratio, ONE_MPH, max_lat_accel, max_steer_angle)

        # TODO: Implement
        self.prev_time = None

        self.linear_velocity = None
        self.linear_velocity_new = None
        self.angular_velocity_new = None

    #def control(self,linear_velocity_new, angular_velocity_new, linear_velocity):
    def control(self):
        # TODO: Change the arg, kwarg list to suit your needs
        # Return throttle, brake, steer
        throttle = 0.0;
        brake = 0.0;
        steering = 0.0;

        if self.linear_velocity is None or self.linear_velocity_new is None or self.angular_velocity_new is None:
            rospy.logwarn("either velocity is None, returning")
            return 0.0,0.0,0.0

        if self.prev_time is None:
            self.prev_time = rospy.get_time()
            rospy.logwarn("prev_time is none, returning")
            return 0.0,0.0,0.0

        current_time = rospy.get_time()
        delta_time = current_time - self.prev_time
        self.prev_time = current_time

        delta_linear_velocity = self.linear_velocity_new - self.linear_velocity
        acceleration = self.pid.step(delta_linear_velocity,delta_time)
        if acceleration > 0.0:
            brake = 0.0
            throttle = acceleration
        else:
            if abs(acceleration) < self.brake_deadband:
                brake = 0
            else:
                brake = abs(acceleration)
        steering = self.yaw_controller.get_steering(self.linear_velocity_new, self.angular_velocity_new, self.linear_velocity) 
        return throttle, brake, steering
