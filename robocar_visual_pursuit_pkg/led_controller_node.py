import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
import smbus2

NODE_NAME = 'led_controller_node'

LED_TOPIC_NAME = '/led'

class LedController(Node):
	def __init__(self):
		super().__init__(NODE_NAME)
		self.bus = smbus2.SMBus(1)
		self.device_address = 0x55
		self.light_on = 0x31
		self.light_off = 0x30
		self.led_subscriber = self.create_subscription(Bool, LED_TOPIC_NAME, self.write_to_controller, 10)
	
	def write_to_controller(self, msg):
		if (msg.data):
			self.bus.write_byte_data(self.device_address, self.light_on, self.light_on)
		else:
			self.bus.write_byte_data(self.device_address, self.light_off, self.light_off)


def main(args=None):
	rclpy.init(args=args)
	node = LedController()

	try:
		rclpy.spin(node)
	except KeyboardInterrupt:
		pass
	finally: 
		node.destroy_node()
		rclpy.shutdown()


if __name__ == '__main__':
	main()

