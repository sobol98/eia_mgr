import rclpy
from geometry_msgs.msg import Twist

def publish_velocity():
    # Initialize the ROS node
    rclpy.init()
    node = rclpy.create_node('velocity_publisher')

    # Create a publisher object
    publisher = node.create_publisher(Twist, '/cmd_vel', 10)

    # Create a rate object with a rate of 10Hz
    rate = node.create_rate(10.0)

    # Create a Twist message object
    msg = Twist()
    msg.linear.x = 1.0  # Set the linear velocity
    msg.angular.z = 0.0  # Set the angular velocity

    # Keep publishing the message until the program is stopped
    while rclpy.ok():
        publisher.publish(msg)
        rate.sleep()

    # Shutdown and cleanup
    rclpy.shutdown()

if __name__ == '__main__':
    publish_velocity()