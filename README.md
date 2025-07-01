# ebimu
ebimu basic ros2_pkg_ebimu9dof + imu_pub publisher

## imu pub 
- publish message as 'sensor_msgs/Imu'
- give serial setting msg for ebimu with code (as <sof1> ... )

### **more info**
https://www.e2box.co.kr/entry/EBIMU-9DOF-ROS2-Package

### install
```sh
pip3 install pyserial

### **to run**
```sh
ros2 run ebimu_pkg imu_pub
