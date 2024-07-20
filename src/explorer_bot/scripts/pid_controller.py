#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist

class PIDController:
    def __init__(self):
        rospy.init_node('pid_controller', anonymous=True)
        
        self.target_distance = rospy.get_param('~target_distance', 5.0)  # Target distance in meters
        self.max_velocity = rospy.get_param('~max_velocity', 1.0)         # Max velocity in m/s
        self.kp_distance = rospy.get_param('~kp_distance', 0.5)           # Proportional gain for distance
        
        self.kp = rospy.get_param('~kp', 0.2)
        self.ki = rospy.get_param('~ki', 0.01)
        self.kd = rospy.get_param('~kd', 0.1)
        
        self.current_velocity = 0.0
        self.current_distance = 0.0
        self.previous_error = 0.0
        self.integral = 0.0
        self.integral_max = rospy.get_param('~integral_max', 10.0)
        self.integral_min = rospy.get_param('~integral_min', -10.0)
        
        self.last_time = rospy.Time.now()
        
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.control_rate = rospy.Rate(10)  # 10 Hz
    
    def compute_target_velocity(self):
        remaining_distance = self.target_distance - self.current_distance
        if remaining_distance <= 0:
            return 0.0
        return min(self.kp_distance * remaining_distance, self.max_velocity)

    def compute_pid(self, target_velocity):
        error = target_velocity - self.current_velocity
        self.integral += error
        self.integral = max(min(self.integral, self.integral_max), self.integral_min)
        derivative = error - self.previous_error
        output = self.kp * error + self.ki * self.integral + self.kd * derivative

        # Saturate control signal
        output = max(min(output, self.max_velocity), -self.max_velocity)

        self.previous_error = error
        return output
    
    def run(self):
        while not rospy.is_shutdown():
            if self.current_distance >= self.target_distance:
                twist_msg = Twist()
                twist_msg.linear.x = 0.0
                twist_msg.linear.y = 0.0
                twist_msg.linear.z = 0.0
                twist_msg.angular.x = 0.0
                twist_msg.angular.y = 0.0
                twist_msg.angular.z = 0.0
                self.pub.publish(twist_msg)
                rospy.loginfo("Target distance reached. Stopping the robot.")
                break

            target_velocity = self.compute_target_velocity()
            control_signal = self.compute_pid(target_velocity)
            
            # Simulate robot movement
            self.current_velocity = control_signal
            self.current_distance += self.current_velocity * 0.1  # Assuming control_rate is 10 Hz
            
            twist_msg = Twist()
            twist_msg.linear.x = control_signal
            twist_msg.linear.y = 0.0
            twist_msg.linear.z = 0.0
            twist_msg.angular.x = 0.0
            twist_msg.angular.y = 0.0
            twist_msg.angular.z = 0.0
            self.pub.publish(twist_msg)
            
            rospy.loginfo(f"Current Distance: {self.current_distance}, Target Velocity: {target_velocity}, Control Signal: {control_signal}")
            self.control_rate.sleep()

if __name__ == '__main__':
    try:
        pid_controller = PIDController()
        pid_controller.run()
    except rospy.ROSInterruptException:
        pass
