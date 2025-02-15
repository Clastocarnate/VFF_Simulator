#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import math
import random
import time

from turtlesim.srv import Spawn
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
from std_srvs.srv import Empty

class TurtleController(Node):
    def __init__(self):
        super().__init__('turtle_controller')

        # Turtle names (fixed names so that subscribers can be reused after reset)
        self.bot_name = "bot_turtle"
        self.goal_name = "goal_turtle"
        self.obstacle_names = ["obs1", "obs2", "obs3", "obs4", "obs5"]

        # Pose storage
        self.bot_pose = None
        self.goal_pose = None
        self.obstacles = {}  # key: obstacle turtle name, value: Pose

        # Flag to ensure we create subscriptions only once
        self.subscriptions_created = False

        # Publisher for bot turtle commands (will be created later)
        self.cmd_vel_publisher = None

        # Create a timer for the control loop (runs at 10Hz)
        self.timer = self.create_timer(0.1, self.control_loop)

        # Initial spawn of all turtles
        self.spawn_turtles()

    def spawn_turtles(self):
        # Spawn the obstacles, bot, and goal turtles
        self.spawn_obstacle_turtles()
        self.spawn_bot_turtle()
        self.spawn_goal_turtle()

        # Create subscribers and publisher only the first time.
        if not self.subscriptions_created:
            # Bot turtle pose subscriber
            self.create_subscription(Pose, f'/{self.bot_name}/pose',
                                     lambda msg: self.pose_callback(msg, self.bot_name), 10)
            # Goal turtle pose subscriber
            self.create_subscription(Pose, f'/{self.goal_name}/pose',
                                     lambda msg: self.pose_callback(msg, self.goal_name), 10)
            # Obstacle turtles pose subscribers
            for obs_name in self.obstacle_names:
                # Using a lambda with default arg to capture obs_name
                self.create_subscription(Pose, f'/{obs_name}/pose',
                                         lambda msg, name=obs_name: self.pose_callback(msg, name), 10)
            # Publisher to send velocity commands to the bot turtle.
            self.cmd_vel_publisher = self.create_publisher(Twist, f'/{self.bot_name}/cmd_vel', 10)
            self.subscriptions_created = True

    def spawn_obstacle_turtles(self):
        # Allowed region: border of 1 unit from the window edges (turtlesim window is 0 to 11)
        allowed_min = 1.0
        allowed_max = 10.0
        # Minimum gap between any two turtles: 48 pixels (~1.07 units)
        min_gap = 48 / 45.0  # approx 1.07 units

        spawned_positions = []

        # Create a spawn client and wait for service
        spawn_client = self.create_client(Spawn, 'spawn')
        while not spawn_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for spawn service for obstacles...')
        
        for obs_name in self.obstacle_names:
            max_attempts = 100
            attempts = 0
            while attempts < max_attempts:
                x = random.uniform(allowed_min, allowed_max)
                y = random.uniform(allowed_min, allowed_max)
                valid = True
                # Check against previously spawned obstacles
                for pos in spawned_positions:
                    if math.sqrt((x - pos[0])**2 + (y - pos[1])**2) < min_gap:
                        valid = False
                        break
                # Also ensure a gap with bot (at (1,1)) and goal (at (10,10))
                if math.sqrt((x - 1.0)**2 + (y - 1.0)**2) < min_gap:
                    valid = False
                if math.sqrt((x - 10.0)**2 + (y - 10.0)**2) < min_gap:
                    valid = False
                
                if valid:
                    spawned_positions.append((x, y))
                    break
                attempts += 1
            if attempts >= max_attempts:
                self.get_logger().warn(f"Could not place obstacle {obs_name} with required gap after {max_attempts} attempts; using last candidate.")
                spawned_positions.append((x, y))
            # Spawn the obstacle turtle
            req = Spawn.Request()
            req.x = x
            req.y = y
            req.theta = 0.0
            req.name = obs_name
            future = spawn_client.call_async(req)
            rclpy.spin_until_future_complete(self, future)
            self.get_logger().info(f"Spawned obstacle turtle: {obs_name} at ({x:.2f}, {y:.2f})")

    def spawn_bot_turtle(self):
        spawn_client = self.create_client(Spawn, 'spawn')
        while not spawn_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for spawn service for bot turtle...')
        bot_x, bot_y, bot_theta = 1.0, 1.0, 0.0
        req = Spawn.Request()
        req.x = bot_x
        req.y = bot_y
        req.theta = bot_theta
        req.name = self.bot_name
        future = spawn_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        self.get_logger().info(f"Spawned bot turtle: {self.bot_name} at ({bot_x}, {bot_y})")

    def spawn_goal_turtle(self):
        spawn_client = self.create_client(Spawn, 'spawn')
        while not spawn_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for spawn service for goal turtle...')
        goal_x, goal_y, goal_theta = 10.0, 10.0, 0.0
        req = Spawn.Request()
        req.x = goal_x
        req.y = goal_y
        req.theta = goal_theta
        req.name = self.goal_name
        future = spawn_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        self.get_logger().info(f"Spawned goal turtle: {self.goal_name} at ({goal_x}, {goal_y})")

    def pose_callback(self, msg, turtle_name):
        # Update stored pose based on turtle name.
        if turtle_name == self.bot_name:
            self.bot_pose = msg
        elif turtle_name == self.goal_name:
            self.goal_pose = msg
        else:
            self.obstacles[turtle_name] = msg

    def control_loop(self):
        # Ensure we have received pose information.
        if self.bot_pose is None or self.goal_pose is None:
            return

        # Attractive force toward the goal.
        dx_attr = self.goal_pose.x - self.bot_pose.x
        dy_attr = self.goal_pose.y - self.bot_pose.y
        k_attr = 1.0
        F_attr_x = k_attr * dx_attr
        F_attr_y = k_attr * dy_attr

        # Compute repulsive forces from obstacles (only if within a threshold distance).
        k_rep = 12.0
        d0 = 2.0  # threshold distance for influence
        F_rep_x = 0.0
        F_rep_y = 0.0
        for obs_name, obs_pose in self.obstacles.items():
            dx = self.bot_pose.x - obs_pose.x
            dy = self.bot_pose.y - obs_pose.y
            d = math.sqrt(dx**2 + dy**2)
            if d < d0 and d != 0:
                repulsive_mag = k_rep * (1.0 / d - 1.0 / d0) / (d**2)
                F_rep_x += repulsive_mag * (dx / d)
                F_rep_y += repulsive_mag * (dy / d)

        # Total force vector.
        F_total_x = F_attr_x + F_rep_x
        F_total_y = F_attr_y + F_rep_y

        # Determine desired heading.
        desired_angle = math.atan2(F_total_y, F_total_x)
        # Compute error between current orientation and desired angle.
        angle_error = desired_angle - self.bot_pose.theta
        # Normalize to [-pi, pi]
        angle_error = math.atan2(math.sin(angle_error), math.cos(angle_error))

        # Build and publish the velocity command.
        cmd = Twist()
        cmd.linear.x = 1.5  # constant forward velocity
        cmd.angular.z = 4.0 * angle_error  # proportional control for rotation
        self.cmd_vel_publisher.publish(cmd)

        # Check if the bot turtle has reached the goal.
        distance_to_goal = math.sqrt(dx_attr**2 + dy_attr**2)
        if distance_to_goal < 0.5:  # threshold for reaching the goal
            self.get_logger().info("Goal reached! Resetting simulation...")
            self.reset_simulation()

    def reset_simulation(self):
        # Call the /reset service to clear the turtlesim canvas.
        reset_client = self.create_client(Empty, 'reset')
        while not reset_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for reset service...')
        req = Empty.Request()
        future = reset_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        self.get_logger().info("Simulation reset complete.")

        # Clear stored pose information.
        self.bot_pose = None
        self.goal_pose = None
        self.obstacles.clear()

        # Small delay to allow the reset to take effect.
        #time.sleep(0.5)

        # Respawn turtles (the subscribers remain intact).
        self.spawn_turtles()

def main(args=None):
    rclpy.init(args=args)
    turtle_controller = TurtleController()
    try:
        rclpy.spin(turtle_controller)
    except KeyboardInterrupt:
        turtle_controller.get_logger().info("Shutting down turtle controller node.")
    finally:
        turtle_controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

