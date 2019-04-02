import rospy
from yaw_controller import YawController
from pid import PID
from lowpass import LowPassFilter
from std_msgs.msg import Bool
GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class Controller(object):
    def __init__(self,
                 vehicle_mass,
                 fuel_capacity,
                 brake_deadband,
                 decel_limit,
                 accel_limit,
                 wheel_radius,
                 wheel_base,
                 steer_ratio,
                 max_lat_accel,
                 max_steer_angle):
        #TODO: Implement
        self.yaw_controller = YawController(wheel_base, steer_ratio, 0.1, max_lat_accel, max_steer_angle)
        
        kp = 0.3 #0.4 #0.3
        ki = 0.001 #0.001
        kd = 25 # 20  #0
        mn = 0 #decel_limit #0.  #Minimum throttle value
        mx = 0.2 #accel_limit #0.2 #Maximum throttle value 
        self.throttle_controller = PID(kp, ki, kd, mn, mx)
        
        tau = 0.5 #1 #15
        ts =  .02 #.02  #.02 
        self.lpf = LowPassFilter(tau,ts)
        self.vehicle_mass = vehicle_mass
        self.fuel_capacity = fuel_capacity
        self.brake_deadband = brake_deadband
        self.decel_limit = decel_limit
        self.accel_limit = accel_limit
        self.wheel_radius = wheel_radius
        
        self.last_time = rospy.get_time()
  

    def control(self, current_vel, dbw_enabled, linear_vel, angular_vel, tld_enabled):
        # TODO: Change the arg, kwarg list to suit your needs
        # Return throttle, brake, steer
        if not dbw_enabled:
            self.throttle_controller.reset()
            return 0., 0., 0.
        
        #rospy.logwarn("Angular velocity: {}".format(angular_vel))
        #rospy.logwarn("Linear velocity: {}".format(linear_vel))
        rospy.logwarn("current velocity: {}".format(current_vel))
        
        current_vel = self.lpf.filt(current_vel)
        rospy.logwarn("filtered velocity: {}".format(current_vel))
        
        steering = self.yaw_controller.get_steering(linear_vel, angular_vel, current_vel)
        #steering = self.lpf.filt(steering)
        
        vel_error = linear_vel - current_vel
        rospy.logwarn("vel_error: {}".format(vel_error))
        self.last_vel = current_vel
        
        current_time = rospy.get_time()
        sample_time = current_time - self.last_time
        self.last_time = current_time
        
        throttle = self.throttle_controller.step(vel_error, sample_time)
        rospy.logwarn("throttle: {}".format(throttle))
        brake = 0
        
        
        #throttle = self.lpf.filt(throttle)
            
        #if acceleration > 0:
        #   throttle = acceleration
        #   brake = 0
        #else:
        #   throttle = 0
        #   brake = self.vehicle_mass * abs(acceleration) * self.wheel_radius
        
        if ((linear_vel == 0. and current_vel < 0.1) or tld_enabled):
            #rospy.logwarn("Apply break- REd light Near")
            throttle = 0
            brake = 400 
        elif throttle < .1 and vel_error < 0:
            rospy.logwarn("throttle should go down,breaks to be applied")
            throttle = 0
            decel = max(vel_error, self.decel_limit)
            brake = abs(decel) * self.vehicle_mass * self.wheel_radius
        return throttle, brake, steering
    
