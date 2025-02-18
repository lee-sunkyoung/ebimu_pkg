
#When the message "Permission denied: /dev/ttyx" appears, 
#change the permission settings on your serial_port
# eg. "sudo chmod 666 /dev/ttyUSB0"

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from std_msgs.msg import String
import serial

comport_num = "/dev/ttyUSB0"
comport_baudrate = 115200
#comport_num = '/dev/tty' + input("EBIMU Port: /dev/tty")
#comport_baudrate = input("Baudrate: ")
ser = serial.Serial(port=comport_num,baudrate=comport_baudrate)

try:
	ser = serial.Serial(port=comport_num, baudrate=comport_baudrate)
except:
	print('Serial port error!')


class EbimuPublisher(Node):

	def __init__(self):
		super().__init__('ebimu_publisher')
		qos_profile = QoSProfile(depth=10)

		self.publisher = self.create_publisher(String, 'ebimu_data', qos_profile)
		self.setup_imu()		
		timer_period = 0.0005
		self.timer = self.create_timer(timer_period, self.timer_callback)
		self.count = 0

	def timer_callback(self):
		msg = String()
		ser_data = ser.readline()
		msg.data = ser_data.decode('utf-8')
		self.publisher.publish(msg)		
  
    def setup_imu(self):
        """ IMU 초기 설정 """
        imu_commands = [
            ('<sof2>', "quaternion ON"),
            ('<sog1>', "angular velocity (gyro) ON"),
            ('<soa1>', "linear velocity with gravity ON"),
            ('<sem1>', "magnetometer ON"),
            ('<sot0>', "temperature OFF"),
            ('<sod0>', "distance OFF"),
            ('<sor10>', "100hz output rate")
        ]
        
        for command, description in imu_commands:
            ser.write(str.encode(command))
            self.get_logger().info(f"IMU setup: {description}")
            ser.readline()
            time.sleep(0.1)  # 안정적인 설정을 위해 약간의 딜레이 추가



def main(args=None):
	rclpy.init(args=args)

	print("Starting ebimu_publisher..")

	node = EbimuPublisher()

	try:
		rclpy.spin(node)

	finally:
		node.destroy_node()
		rclpy.shutdown()


if __name__ == '__main__':
	main()

