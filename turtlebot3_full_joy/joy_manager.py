import os
import yaml

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import Joy, JointState
from geometry_msgs.msg import Twist
from control_msgs.action import GripperCommand

import time

class JoyManager(Node):
    MAX_TRAN_SPEED = 0.26  # m/s
    MAX_ROT_SPEED = 1.82  # rad/s
    JOINT_MOVE = 0.1745 # rad = 10 degrees

    def __init__(self):
        super().__init__('joy_manager_node')

        self.declare_parameters(
            namespace='',
            parameters=[
                ('axis_linear', 1),
                ('axis_angular', 0),
                ('axis_arm', 6),
                ('enable_button', 7),
                ('joint1_button', 3),
                ('joint2_button', 1),
                ('joint3_button', 0),
                ('joint4_button', 2),
                ('grip_button', 4),
                ('release_button', 5),
                ('home_button', 6)
            ])

        self.axes_config = {
            'axis_linear': self.get_parameter("axis_linear").value,
            'axis_angular': self.get_parameter("axis_angular").value,
            'axis_arm': self.get_parameter("axis_arm").value,
        }
        
        self.buttons_config ={
            'enable_button': self.get_parameter("enable_button").value,
            'joint1_button': self.get_parameter("joint1_button").value,
            'joint2_button': self.get_parameter("joint2_button").value,
            'joint3_button': self.get_parameter("joint3_button").value,
            'joint4_button': self.get_parameter("joint4_button").value,
            'grip_button': self.get_parameter("grip_button").value,
            'release_button': self.get_parameter("release_button").value,
            'home_button': self.get_parameter("home_button").value
        }

        print(self.axes_config)
        print(self.buttons_config)
        # Publisher
        self.cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.arm_pub = self.create_publisher(JointTrajectory, "/arm_controller/joint_trajectory", 10)


        self.gripper_client = ActionClient(self, GripperCommand, '/gripper_controller/gripper_cmd')
        

        ## define joy state
        self.enable = False
        self.joint = 0
        self.current_states = JointState()

        self.joy_sub = self.create_subscription(Joy,'/joy',self.joy_cb,10)
        self.joint_states_sub = self.create_subscription(JointState,'/joint_states',self.joint_states_cb,10)
        rclpy.spin(self)
    
    def joint_states_cb(self, data):
        self.current_states = data

    def joy_cb(self, data):
        if(data.buttons[self.buttons_config['enable_button']] == 1):
            self.enable = not self.enable
            time.sleep(1.5)
  
        if(self.enable):
            x = data.axes[self.axes_config['axis_linear']]
            yaw = data.axes[self.axes_config['axis_angular']]

            self.cmd_vel_pub.publish(self.build_twist_msg(x,yaw))

            if(data.buttons[self.buttons_config['home_button']]==1):
                print("home")
                self.arm_pub.publish(self.home_arm())

            if(data.buttons[self.buttons_config['joint1_button']] == 1):
                self.joint = 0
            elif(data.buttons[self.buttons_config['joint2_button']] == 1):
                self.joint = 1
            elif(data.buttons[self.buttons_config['joint3_button']] == 1):
                self.joint = 2
            elif(data.buttons[self.buttons_config['joint4_button']] == 1):
                self.joint = 3

            if(data.buttons[self.buttons_config['grip_button']]==1):
                self.grip()
            elif(data.buttons[self.buttons_config['release_button']]==1):
                self.release()

            increase = 0
            if(data.axes[self.axes_config['axis_arm']]==1):
                increase = self.JOINT_MOVE
                self.arm_pub.publish(self.build_arm_msg(increase))

            elif(data.axes[self.axes_config['axis_arm']]==-1):
                increase = -1*self.JOINT_MOVE
                self.arm_pub.publish(self.build_arm_msg(increase))

    def grip(self):
        goal_msg = GripperCommand.Goal()
        goal_msg.command.position = 0.05
        self.gripper_client.wait_for_server()
        return self.gripper_client.send_goal_async(goal_msg)

    def release(self):
        goal_msg = GripperCommand.Goal()
        goal_msg.command.position = 0.02
        self.gripper_client.wait_for_server()
        return self.gripper_client.send_goal_async(goal_msg)

    def home_arm(self):
        msg = JointTrajectory()
        msg.header.stamp = self.get_clock().now().to_msg()

        msg.joint_names = ['joint1','joint2','joint3','joint4']
        point_1 = JointTrajectoryPoint()
        point_1.positions = [-0.0015,-1.6106,1.4020,0.2730]
        point_1.time_from_start.sec = 1
        msg.points = [point_1]

        return msg

    def build_arm_msg(self, increase):
        snap_state = self.current_states
        names = snap_state.name
        positions = snap_state.position
        
        msg = JointTrajectory()
        msg.header.stamp = self.get_clock().now().to_msg()

        msg.joint_names = ['joint1','joint2','joint3','joint4']
        point_1 = JointTrajectoryPoint()
        point_1.positions = [positions[names.index('joint1')],
                                positions[names.index('joint2')],
                                positions[names.index('joint3')],
                                positions[names.index('joint4')]]
        point_1.time_from_start.nanosec = 100000000
        
        point_1.positions[self.joint] += increase
        msg.points = [point_1]
        return msg

    def build_twist_msg(self, x, yaw):
        msg = Twist()
        msg.linear.x = self.MAX_TRAN_SPEED * x
        msg.angular.z = self.MAX_ROT_SPEED * yaw
        return msg

def main(args=None):
    rclpy.init(args=args)

    joy_manager_node = JoyManager()
    # rclpy.spin(joy_manager_node)

    joy_manager_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()