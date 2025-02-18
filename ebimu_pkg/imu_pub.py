import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
import serial
import re
import math
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion

class EbimuPublisher(Node):

    def __init__(self):
        super().__init__('ebimu_publisher')
        qos_profile = QoSProfile(depth=10)

        # IMU 데이터 퍼블리셔 설정
        self.imu_pub = self.create_publisher(Imu, '/imu/raw_data', qos_profile)

        # 시리얼 포트 설정
        self.comport_num = "/dev/ttyUSB0"
        self.comport_baudrate = 115200

        try:
            self.ser = serial.Serial(port=self.comport_num, baudrate=self.comport_baudrate, timeout=1)
            self.get_logger().info("Connected to IMU Serial Port")
        except serial.SerialException as e:
            self.get_logger().error(f"Serial Port Error: {e}")
            return
        
        # IMU 초기화 메시지 전송
        self.setup_imu()

        # 타이머 설정 (10ms 간격 = 100Hz)
        self.timer_period = 0.01  
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

        # 이전 데이터 저장
        self.prev_str = ""

    def setup_imu(self):
        """ IMU의 데이터 출력을 설정하는 함수 """
        setup_cmds = [
            "<sof2>",  # Quaternion ON
            "<sog1>",  # Angular velocity ON
            "<soa1>",  # Linear acceleration ON
            "<sem1>",  # Magnetometer ON
            "<sot0>",  # Temperature OFF
            "<sod0>",  # Distance OFF
            "<sor10>"  # Output rate 100Hz 설정
        ]

        for cmd in setup_cmds:
            self.ser.write(cmd.encode())
            self.get_logger().info(f"IMU 설정: {cmd}")
            self.ser.readline()  # 응답 읽기

    def timer_callback(self):
        """ 타이머에 의해 주기적으로 실행되는 콜백 함수 """
        self.ser.reset_input_buffer()
        ser_data = self.ser.readline().decode('utf-8').strip()  

        # 데이터 유효성 검사
        comma_cnt = len([m.start() for m in re.finditer(',', ser_data)])
        if comma_cnt < 10:
            self.get_logger().warn("Invalid IMU data received, ignoring...")
            ser_data = self.prev_str
        else:
            self.prev_str = ser_data

        # 데이터가 유효하지 않으면 패스
        if not isinstance(ser_data, str):
            return

        str_list = ser_data.split(',')
        if '*' in str_list[0]:
             str_list[0] = str_list[0].split('*')[1]  # '*' 이후 값만 추출
        else:
            self.get_logger().warn("Received IMU data does not contain '*'")
            # return

        imu_msg = Imu()
        imu_msg.header.stamp = self.get_clock().now().to_msg()
        imu_msg.header.frame_id = "imu_link"

        # Quaternion 순서 변경 (IMU는 [z, y, x, w] 순서로 제공)
        imu_msg.orientation = Quaternion(
            x=float(str_list[2]),
            y=float(str_list[1]),
            z=float(str_list[0]),
            w=float(str_list[3])
        )
        imu_msg.orientation_covariance = [0.0007, 0.0, 0.0, 0.0, 0.0007, 0.0, 0.0, 0.0, 0.0007]

        # 가속도 (중력 영향 포함)
        imu_msg.linear_acceleration.x = -float(str_list[7])
        imu_msg.linear_acceleration.y = -float(str_list[8])
        imu_msg.linear_acceleration.z = -float(str_list[9])
        imu_msg.linear_acceleration_covariance = [0.005, 0.0, 0.0, 0.0, 0.005, 0.0, 0.0, 0.0, 0.005]

        # 각속도 (도 -> 라디안 변환)
        imu_msg.angular_velocity.x = math.radians(float(str_list[4]))
        imu_msg.angular_velocity.y = math.radians(float(str_list[5]))
        imu_msg.angular_velocity.z = math.radians(float(str_list[6]))
        imu_msg.angular_velocity_covariance = [0.001, 0.0, 0.0, 0.0, 0.001, 0.0, 0.0, 0.0, 0.001]

        # 퍼블리시
        self.imu_pub.publish(imu_msg)


def main(args=None):
    rclpy.init(args=args)
    node = EbimuPublisher()

    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
