import rclpy
from std_msgs.msg import Float64
from launch import LaunchDescription
# from launch_ros.actions import Node
from rclpy.node import Node
import time
import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess


# def generate_launch_description():
#     # Get the path to the Python script
#     python_script = os.path.join('/home/patryk/vrx_ws/src/vrx/vrx_gz/launch/', 'asv_control.py')
    
#     return LaunchDescription([
#         ExecuteProcess(
#             cmd=['python3', python_script],
#             output='screen',
#         ),
#     ])

# def publish_thrust():
#     # Initialize the ROS node
#     rclpy.init()
#     node = rclpy.create_node('thrust_publisher')

#     # Create a publisher object
#     publisher_left = node.create_publisher(Float64, '/wamv/thrusters/left/thrust', 10)
#     publisher_right = node.create_publisher(Float64, '/wamv/thrusters/right/thrust', 10)
#     # Create a rate object with a rate of 10Hz
#     rate = node.create_rate(10.0)

#     # Create a Float64 message object
#     msg_left = Float64()
#     msg_right=Float64()

#     start_time=time.time()

#     # msg_left.data = -100000.0  # Set the thrust value for left thruster
#     # msg_right.data = -100000.0 # Set the thrust value
#     print('aaaaa')
#     # Keep publishing the message until the program is stopped
#     while rclpy.ok():
#         try:
#             if time.time()-start_time<15.0:
#                 msg_left.data = 100000.0  # Set the thrust value for left thruster
#                 msg_right.data = 100000.0
#                 # print("1")
#             elif time.time()-start_time>20.0:
#                 msg_left.data = -100000.0  # Set the thrust value for left thruster
#                 msg_right.data = -100000.0
#                 # print("2")
#             publisher_left.publish(msg_left)
#             publisher_right.publish(msg_right)
#             print('bbbbbb')
#             # print("publisher_left ",publisher_left)
#             # print("publisher_right ", publisher_right)
#             rate.sleep()
        
#         except Exception as error_log:
#             print("Error: ", error_log)
#             print('cccccc')
#             break
#     # Shutdown and cleanup
#     rclpy.shutdown()

# if __name__ == '__main__':
#     while True:
#       publish_thrust()
#         # time.sleep(2)


class ThrusterPublisher(Node):

    def __init__(self):
        super().__init__(node_name='thruster_publisher')
        self.publisher_left = self.create_publisher(Float64, '/wamv/thrusters/left/thrust', 10)
        self.publisher_right = self.create_publisher(Float64, '/wamv/thrusters/right/thrust', 10)
        self.msg_left = Float64()
        self.msg_right = Float64()
        self.start_time = time.time()

        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        # self.i = 0

    def timer_callback(self):
        elapsed_time=time.time()-self.start_time
        if elapsed_time<15.0:
            self.msg_left.data = 10000.0
            self.msg_right.data = 10000.0
        elif elapsed_time>20.0:
            self.msg_left.data = -11000.0
            self.msg_right.data = -11000.0
        self.publisher_left.publish(self.msg_left)
        self.publisher_right.publish(self.msg_right)

        # msg = String()
        # msg.data = 'Hello World: %d' % self.i
        # self.publisher_.publish(msg)
        # self.get_logger().info('Publishing: "%s"' % msg.data)
        # self.i += 1


    @staticmethod
    def generate_launch_description():
    # Get the path to the Python script
        python_script = os.path.join('/home/patryk/vrx_ws/src/vrx/vrx_gz/launch/', 'asv_control.py')
        
        return LaunchDescription([
            ExecuteProcess(
                cmd=['python3', python_script],
                output='screen',
            ),
        ])



def main(args=None):
    rclpy.init(args=args)

    thrust_publisher = ThrusterPublisher()

    rclpy.spin(thrust_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    thrust_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()