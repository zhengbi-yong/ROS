```zsh
(dexmani)  sisyphus@sisyphus-dual4090  ~/ROS   master ±  tree ./src/leap_hand
./src/leap_hand
├── CMakeLists.txt
├── launch
│   └── launch_leap.py
├── package.xml
├── readme.md
├── scripts
│   ├── leaphand_node.py
│   ├── leap_hand_utils
│   │   ├── dynamixel_client.py
│   │   ├── __init__.py
│   │   ├── leap_hand_utils.py
│   │   └── __pycache__
│   │       ├── dynamixel_client.cpython-310.pyc
│   │       ├── __init__.cpython-310.pyc
│   │       └── leap_hand_utils.cpython-310.pyc
│   └── ros2_example.py
└── srv
    ├── LeapEffort.srv
    ├── LeapPosition.srv
    └── LeapVelocity.srv

5 directories, 15 files
```
上面是我的项目结构
```xml
<?xml version="1.0"?>
<package format="3">
  <name>leap_hand</name>
  <version>0.0.0</version>
  <description>The leap_hand package</description>

  <maintainer email="anag@todo.todo">anag</maintainer>
  <license>TODO</license>

  <depend>rclpy</depend>
  <depend>sensor_msgs</depend>
  <depend>std_msgs</depend>
  <depend>leap_hand_interfaces</depend>

  <buildtool_depend>ament_cmake</buildtool_depend>
  <build_depend>ament_cmake_python</build_depend>
  <build_depend>python3</build_depend>
  <exec_depend>python3</exec_depend>
  <exec_depend>launch</exec_depend>

  <test_depend>ament_lint_auto</test_depend>
  <test_depend>ament_lint_common</test_depend>

  <member_of_group>rosidl_interface_packages</member_of_group>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```
上面是我package.xml文件的内容
```txt
cmake_minimum_required(VERSION 3.5)
project(leap_hand)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

################################################
## Declare ROS messages, services and actions ##
################################################

## Generate services in the 'srv' folder
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/LeapVelocity.srv"
  "srv/LeapPosition.srv"
  "srv/LeapEffort.srv"
  DEPENDENCIES std_msgs
)

###################################
## ament specific configuration  ##
###################################
ament_package()

#############
## Install ##
#############

# Install Python scripts
install(PROGRAMS
  scripts/leaphand_node.py
  scripts/ros2_example.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# find_package(ament_cmake_gtest REQUIRED)
# ament_add_gtest(${PROJECT_NAME}-test test/test_leap_hand.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()
```
上面是我的CMakeLists.txt的内容。
```python
#!/usr/bin/env python3

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String

from leap_hand_utils.dynamixel_client import DynamixelClient
import leap_hand_utils.leap_hand_utils as lhu
from leap_hand.srv import LeapPosition, LeapVelocity, LeapEffort

class LeapNode(Node):
    def __init__(self):
        super().__init__('leaphand_node')
        # Some parameters to control the hand
        self.kP = self.declare_parameter('kP', 800.0).get_parameter_value().double_value
        self.kI = self.declare_parameter('kI', 0.0).get_parameter_value().double_value
        self.kD = self.declare_parameter('kD', 200.0).get_parameter_value().double_value
        self.curr_lim = self.declare_parameter('curr_lim', 350.0).get_parameter_value().double_value
        self.ema_amount = 0.2
        self.prev_pos = self.pos = self.curr_pos = lhu.allegro_to_LEAPhand(np.zeros(16))

        # Subscribes to a variety of sources that can command the hand
        self.create_subscription(JointState, 'cmd_leap', self._receive_pose, 10)
        self.create_subscription(JointState, 'cmd_allegro', self._receive_allegro, 10)
        self.create_subscription(JointState, 'cmd_ones', self._receive_ones, 10)

        # Creates services that can give information about the hand out
        self.create_service(LeapPosition, 'leap_position', self.pos_srv)
        self.create_service(LeapVelocity, 'leap_velocity', self.vel_srv)
        self.create_service(LeapEffort, 'leap_effort', self.eff_srv)

        # You can put the correct port here or have the node auto-search for a hand at the first 3 ports.
        self.motors = [0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15]
        try:
            self.dxl_client = DynamixelClient(self.motors, '/dev/ttyUSB0', 4000000)
            self.dxl_client.connect()
        except Exception:
            try:
                self.dxl_client = DynamixelClient(self.motors, '/dev/ttyUSB1', 4000000)
                self.dxl_client.connect()
            except Exception:
                self.dxl_client = DynamixelClient(self.motors, '/dev/ttyUSB2', 4000000)
                self.dxl_client.connect()

        # Enables position-current control mode and the default parameters
        self.dxl_client.sync_write(self.motors, np.ones(len(self.motors)) * 5, 11, 1)
        self.dxl_client.set_torque_enabled(self.motors, True)
        self.dxl_client.sync_write(self.motors, np.ones(len(self.motors)) * self.kP, 84, 2)  # Pgain stiffness     
        self.dxl_client.sync_write([0,4,8], np.ones(3) * (self.kP * 0.75), 84, 2)  # Pgain stiffness for side to side should be a bit less
        self.dxl_client.sync_write(self.motors, np.ones(len(self.motors)) * self.kI, 82, 2)  # Igain
        self.dxl_client.sync_write(self.motors, np.ones(len(self.motors)) * self.kD, 80, 2)  # Dgain damping
        self.dxl_client.sync_write([0,4,8], np.ones(3) * (self.kD * 0.75), 80, 2)  # Dgain damping for side to side should be a bit less
        # Max at current (in unit 1ma) so don't overheat and grip too hard
        self.dxl_client.sync_write(self.motors, np.ones(len(self.motors)) * self.curr_lim, 102, 2)
        self.dxl_client.write_desired_pos(self.motors, self.curr_pos)

    # Receive LEAP pose and directly control the robot
    def _receive_pose(self, msg):
        pose = msg.position
        self.prev_pos = self.curr_pos
        self.curr_pos = np.array(pose)
        self.dxl_client.write_desired_pos(self.motors, self.curr_pos)

    # Allegro compatibility, first read the allegro publisher and then convert to leap
    def _receive_allegro(self, msg):
        pose = lhu.allegro_to_LEAPhand(msg.position, zeros=False)
        self.prev_pos = self.curr_pos
        self.curr_pos = np.array(pose)
        self.dxl_client.write_desired_pos(self.motors, self.curr_pos)

    # Sim compatibility, first read the sim publisher and then convert to leap
    def _receive_ones(self, msg):
        pose = lhu.sim_ones_to_LEAPhand(np.array(msg.position))
        self.prev_pos = self.curr_pos
        self.curr_pos = np.array(pose)
        self.dxl_client.write_desired_pos(self.motors, self.curr_pos)

    # Service that reads and returns the pos of the robot in regular LEAP Embodiment scaling.
    def pos_srv(self, request, response):
        response.position = self.dxl_client.read_pos().tolist()
        return response

    # Service that reads and returns the vel of the robot in LEAP Embodiment
    def vel_srv(self, request, response):
        response.velocity = self.dxl_client.read_vel().tolist()
        return response

    # Service that reads and returns the effort/current of the robot in LEAP Embodiment
    def eff_srv(self, request, response):
        response.effort = self.dxl_client.read_cur().tolist()
        return response

def main(args=None):
    rclpy.init(args=args)
    leaphand_node = LeapNode()
    rclpy.spin(leaphand_node)
    leaphand_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```
上面是我的leaphand_node.py文件的内容.
```python
#!/usr/bin/env python3

import sys
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from leap_hand.srv import LeapPosition
import time
import numpy as np

class MinimalClientAsync(Node):

    def __init__(self):
        super().__init__('minimal_client_async')
        self.cli = self.create_client(LeapPosition, '/leap_position')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = LeapPosition.Request()
        self.pub_hand = self.create_publisher(JointState, '/cmd_ones', 10) 

    def send_request(self):
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

def main(args=None):
    rclpy.init(args=args)
    minimal_client = MinimalClientAsync()
    x = list(np.zeros(16))
    y = 0.025
    while True:
        response = minimal_client.send_request()
        print(response)  ##Receive 
        time.sleep(0.05)
        stater = JointState()
        x[0] = x[0] + y
        if x[0] > 0.1:
            y = - 0.025
        if x[0] < -1:
            y = 0.025
        stater.position = x  ##You can set the position this way
        minimal_client.pub_hand.publish(stater)  # Choose the right embodiment here
    minimal_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```
上面是我的ros2_example.py文件的内容。
```python
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='leap_hand',
            executable='leaphand_node.py',
            name='leaphand_node',
            emulate_tty=True,
            output='screen',
            parameters=[
                {'kP': 800.0},
                {'kI': 0.0},
                {'kD': 200.0},
                {'curr_lim': 500.0}
            ]
        ),
        Node(
            package='leap_hand',
            executable='ros2_example.py',
            name='ros2_example',
            emulate_tty=True,
            output='screen'
        )
    ])
```
上面是我的launch_leap.py文件的内容。
请你根据上述信息，帮我增加一个使用键盘控制机械手的功能，要求：
qwertyuiop分别控制0-9号关节增大0.1弧度，asdfghjkl;分别控制0-9号关节减小0.1弧度。要求使用ros2 launch leap_hand launch_leap_keyboard.py启动。

现在我有一个抓取的任务，需要给机械手一个抓取的姿势，然后我的初步打算是机械手开一个节点一直监听程序传来的信号，如果有信号就去执行，否则就保持上一个状态的动作，请问这样做是否合理，如果不合理，应该怎么做？如果合理的话我应该如何设计我的程序去给出这个指令，以及我如果想制作一个demo程序，我应该如何写？

请你详细介绍一下ROS2的Action接口，因为我的设想是这样的，就是通过摄像头等传感器去获取周围环境的信息，然后发现需要被抓握的物体之后机械臂将灵巧手移动到可以抓握的范围之内，这个过程是有反馈的，并且灵巧手在执行抓握的过程中也要使用反馈来保证抓握的成功，所以我觉得Action接口可能是一个比较好的选择，但是我对ROS2的Action接口不是很了解，所以请你详细介绍一下ROS2的Action接口，以及如何使用Action接口来实现我的设想。我的深度学习模型给出的控制信号应该是一个16维的向量，分别代表灵巧手的各个关节的角度。