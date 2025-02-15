import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose
from turtlesim.srv import TeleportAbsolute, Spawn
from std_srvs.srv import Empty  # Import for the /clear service
from geometry_msgs.msg import Twist
import math
import random

# Constants
BOT_SPEED = 2.0
ATTRACTION_GAIN = 0.8
ANGULAR_GAIN = 4.0


class Turtle:
    def __init__(self):
        self.pose = Pose()  # Store the pose of the turtle
        self.twist = Twist()  # Store the velocity command (linear and angular)

    def update_pose(self, data):
        """Update the turtle's pose from the subscription."""
        self.pose = data


def spawn_turtle(x, y, theta, name, node):
    """
    Function to call the /spawn service to spawn a turtle.
    :param x: X-coordinate of the new turtle
    :param y: Y-coordinate of the new turtle
    :param theta: Orientation of the new turtle (in radians)
    :param name: Name of the new turtle
    :param node: ROS node to use for the service call
    """
    client = node.create_client(Spawn, '/spawn')

    if not client.wait_for_service(timeout_sec=5.0):
        node.get_logger().error('Spawn service is not available.')
        return

    request = Spawn.Request()
    request.x = x
    request.y = y
    request.theta = theta
    request.name = name

    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future)

    if future.result() is not None:
        node.get_logger().info(f"Turtle {future.result().name} spawned at ({x}, {y}) with theta={theta}.")
    else:
        node.get_logger().error('Failed to spawn the turtle.')


def kinematics(node, bot, goal, teleport_client, clear_client):
    """
    Calculate attraction vectors and update bot's velocity command.
    Teleport the goal turtle to a random location and then clear the canvas when the bot reaches it.
    """
    dx = goal.pose.x - bot.pose.x
    dy = goal.pose.y - bot.pose.y
    distance = math.sqrt(dx**2 + dy**2)
    theta_goal = math.atan2(dy, dx)
    angular_error = theta_goal - bot.pose.theta
    angular_error = math.atan2(math.sin(angular_error), math.cos(angular_error))

    if distance > 0.1:  # Move if the bot is not at the goal
        bot.twist.linear.x = BOT_SPEED * ATTRACTION_GAIN
        bot.twist.angular.z = ANGULAR_GAIN * angular_error
    else:  # Stop, teleport the goal, and clear the canvas
        bot.twist.linear.x = 0.0
        bot.twist.angular.z = 0.0

        # Teleport the goal turtle to a random location
        if teleport_client.wait_for_service(timeout_sec=5.0):
            teleport_request = TeleportAbsolute.Request()
            teleport_request.x = random.uniform(1.0, 10.0)  # Random x-coordinate
            teleport_request.y = random.uniform(1.0, 10.0)  # Random y-coordinate
            teleport_request.theta = random.uniform(0, 2 * math.pi)  # Random orientation

            future = teleport_client.call_async(teleport_request)
            rclpy.spin_until_future_complete(node, future)

            if future.result() is not None:
                node.get_logger().info(f'Turtle2 teleported to (x={teleport_request.x}, y={teleport_request.y}).')
            else:
                node.get_logger().error('Failed to teleport turtle2.')

        # Clear the canvas using the /clear service
        if clear_client.wait_for_service(timeout_sec=5.0):
            future = clear_client.call_async(Empty.Request())
            rclpy.spin_until_future_complete(node, future)
            node.get_logger().info('Canvas cleared.')
        else:
            node.get_logger().error('Failed to call /clear service.')


def main(args=None):
    rclpy.init(args=args)

    # Create a ROS node
    node = Node('turtle_controller')

    # Initialize bot and goal turtles
    bot = Turtle()
    goal = Turtle()

    # Service clients for teleporting and clearing
    teleport_client = node.create_client(TeleportAbsolute, '/turtle2/teleport_absolute')
    clear_client = node.create_client(Empty, '/clear')

    # Spawn turtles
    spawn_turtle(3.0, 4.0, 0.0, 'turtle2', node)
    spawn_turtle(6.0, 6.0, 0.0, 'turtle3', node)

    # Subscribers to update turtle poses
    node.create_subscription(Pose, '/turtle1/pose', bot.update_pose, 10)
    node.create_subscription(Pose, '/turtle2/pose', goal.update_pose, 10)

    # Publisher to control the bot's velocity
    cmd_vel_publisher = node.create_publisher(Twist, '/turtle1/cmd_vel', 10)

    try:
        while rclpy.ok():
            # Spin once to process incoming pose updates
            rclpy.spin_once(node)

            # Update kinematics
            kinematics(node, bot, goal, teleport_client, clear_client)

            # Publish the updated velocity commands
            cmd_vel_publisher.publish(bot.twist)
    except KeyboardInterrupt:
        pass
    finally:
        # Destroy the node and shutdown ROS
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

