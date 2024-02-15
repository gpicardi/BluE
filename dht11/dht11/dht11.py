import rclpy
from rclpy.node import Node

from std_msgs.msg import Float64

from time import sleep
import Adafruit_DHT

class Dht11Node(Node):

	def __init__(self):
		# Create sensor object and specify input pin (Repeat for 6 sensors)
		self.DHT_SENSOR = Adafruit_DHT.DHT11
		self.DHT_PIN = 17
		
		# Initialize parent (ROS Node)
		super().__init__('dht11')
		# What data structure for 6 sensors? Custom or Float64Multiarrray?
		self.temperature_publisher_ = self.create_publisher(Float64, '/dht11/temperature', 10)
		self.humidity_publisher_ = self.create_publisher(Float64, '/dht11/humidity', 10)
		timer_period = 3
		self.timer = self.create_timer(timer_period, self.timer_callback)
		self.i = 0

		self.get_logger().info("Sensor DHT11 correctly initialized.")
		
	def timer_callback(self):
		# Read data from sensor (Repeat for 6 sensors)
		humidity, temperature = Adafruit_DHT.read(self.DHT_SENSOR, self.DHT_PIN)

		# Reading could fail. If it does, simply skip the publishing
		#This try block will change depending on the data structure used for 6 sensors
		try: 
			temperature_msg = Float64()
			temperature_msg.data = temperature
			
			humidity_msg = Float64()
			humidity_msg.data = humidity
			
			self.temperature_publisher_.publish(temperature_msg)
			self.humidity_publisher_.publish(humidity_msg)
			
			self.get_logger().info("Publishing Temperature: %.2f" %temperature_msg.data)
			self.get_logger().info("Publishing Humidity: %.2f" %humidity_msg.data)
			self.i +=1
		except:
			self.i +=1

def main(args=None):
	rclpy.init(args=args)
	dht11_publisher = Dht11Node()
	rclpy.spin(dht11_publisher)
	
	dht11_publisher.destroy_node()
	rclpy.shutdown()
	
if __name__ == '__main__':
	main()
