#!/usr/bin/python3

import rclpy
from geometry_msgs.msg import Twist

def move_turtle(node, publisher):
    # Create and populate the Twist message
    msg = Twist()
    msg.linear.x = 0.0
    msg.angular.z = 0.0
    for i in range(10):
        msg.linear.x += i  # Move forward with a speed of 2.0
        msg.angular.z += i  # Rotate with a speed of 1.0
        publisher.publish(msg)
    for i in range(10):
        msg.linear.x -= i
        msg.angular.z -= i
        publisher.publish(msg)
    # Publish the message
  

    # Log the published message
    node.get_logger().info(f'Publishing: Linear x: {msg.linear.x:.2f}, Angular z: {msg.angular.z:.2f}')

def main(args=None):
    # Initialize the ROS 2 Python client library
    rclpy.init(args=args)

    # Create a node
    node = rclpy.create_node('turtle_controller')

    # Create a publisher for the /turtle1/cmd_vel topic
    publisher = node.create_publisher(Twist, '/turtle1/cmd_vel', 10)

    # Set up a timer to call the move_turtle function every 0.5 seconds
    timer_period = 0.5  # seconds
    node.create_timer(timer_period, lambda: move_turtle(node, publisher))

    try:
        # Spin the node to keep it active
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Destroy the node and shutdown rclpy
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
