import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose

def turtle1_callback(data):
	print(f"Turtle1: {data.x},{data1.y}")
def turtle2_callback(data):
	print(f"Turtle1: {data.x},{data1.y}")

rclpy.init(args=args)
node = Node('turtle_pose_listener')
node.create_subscription(Pose, '/turtle1/pose', turtle1_callback,10)
node.create_subscription(Pose, '/turtle2/pose', turtle2_callback,10)

try:
	rclpy.spin(node)
except keyboardInterrupt:
	pass
finally:
	node.destroy_node()
	rclpy.shutdown()
