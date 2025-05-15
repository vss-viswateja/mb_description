import rclpy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
import time




HALF_DISTANCE_BETWEEN_WHEELS = 0.092
WHEEL_RADIUS = 0.005
MAX_VEL = 10

class DiffDriver:
    def init(self, webots_node, properties):
        self.__robot = webots_node.robot

        self.__front_left_motor = self.__robot.getDevice('fl_wheel_joint')
        self.__front_right_motor = self.__robot.getDevice('fr_wheel_joint')
        self.__rear_left_motor = self.__robot.getDevice('rl_wheel_joint')
        self.__rear_right_motor = self.__robot.getDevice('rr_wheel_joint')

        self.__front_left_motor.setPosition(float('inf'))
        self.__front_left_motor.setVelocity(0)

        self.__front_right_motor.setPosition(float('inf'))
        self.__front_right_motor.setVelocity(0)

        self.__rear_left_motor.setPosition(float('inf'))
        self.__rear_left_motor.setVelocity(0)

        self.__rear_right_motor.setPosition(float('inf'))
        self.__rear_right_motor.setVelocity(0)

        self.__target_twist = Twist()

        rclpy.init(args=None)
        self.__node = rclpy.create_node('diff_driver')
        self.__node.create_subscription(Twist, 'cmd_vel', self.__cmd_vel_callback, 1)
        #Define the PID controller params 
        self.__E_l = 0
        self.__E_r = 0
        self.__edot_l = 0
        self.__edot_r = 0
        self.__old_e_l = 0
        self.__old_e_r = 0




    def __cmd_vel_callback(self, twist):
        self.__target_twist = twist

    def step(self):
        rclpy.spin_once(self.__node, timeout_sec=0)

        forward_speed = self.__target_twist.linear.x
        if forward_speed > 0.13:
            forward_speed = 0.13
        elif forward_speed < -0.13:
            forward_speed = -0.13

        
        angular_speed = self.__target_twist.angular.z
        if angular_speed > 2.826:
            angular_speed = 2.826
        elif angular_speed < -2.826:
            angular_speed = -2.826

        command_motor_left = ((forward_speed - angular_speed * HALF_DISTANCE_BETWEEN_WHEELS) / WHEEL_RADIUS) / 2
        command_motor_right = ((forward_speed + angular_speed * HALF_DISTANCE_BETWEEN_WHEELS) / WHEEL_RADIUS ) / 2

        current_left = self.__front_left_motor.getVelocity()
        current_right = self.__front_right_motor.getVelocity()

        e0l = command_motor_left - current_left
        e0r = command_motor_right - current_right

        kp = 0.5
        ki = 0.17 
        kd = 0.1 
        
        edot_l = e0l - self.__old_e_l
        edot_r = e0r - self.__old_e_r

        self.__E_l = self.__E_l + e0l
        self.__E_r = self.__E_r + e0r 

        E_l = self.__E_l
        E_r = self.__E_r
        
        u_l = kp*e0l + ki * E_l + kd* edot_l
        u_r = kp * e0r + ki*E_r + kd* edot_r 

        self.__front_left_motor.setVelocity(u_l)
        self.__rear_left_motor.setVelocity(u_l)
        self.__front_right_motor.setVelocity(u_r)
        self.__rear_right_motor.setVelocity(u_r)

        self.__old_e_l = e0l        
        self.__old_e_r = e0r
        
        
       


        
        


    
