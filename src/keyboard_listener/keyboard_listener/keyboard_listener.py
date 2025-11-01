import rclpy
from std_msgs.msg import String

import sys, select, termios, tty

NAMED_TARGET_TOPIC = '/named_target'

settings = termios.tcgetattr(sys.stdin)

bindings = {
	'w': "home",
	'a': "zero",
	'd': "max",
}

def getKey():
	tty.setraw(sys.stdin.fileno())
	select.select([sys.stdin], [], [], 0)
	key = sys.stdin.read(1)
	termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
	return key

def main():
	rclpy.init()
	node = rclpy.create_node('teleop_twist_keyboard')
	pub = node.create_publisher(String, NAMED_TARGET_TOPIC, 10)

	try:
		while True:
			key = getKey()
			if key == '\x1b':  # ESC
				break
			if key in bindings.keys():
				pub.publish(String(data=bindings[key]))
			else:
				print("Unknown key pressed: %s" % (key,))
	finally:
		node.destroy_node()
		rclpy.shutdown()

