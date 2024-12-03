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


我现在想给一个新设备写一个ros2的包，我应该做哪些工作？
```python
import sys
sys.path.append("../lib")
import unitree_arm_interface
import time
import numpy as np
np.set_printoptions(precision=3, suppress=True)

print("Press ctrl+\ to quit process.")

arm =  unitree_arm_interface.ArmInterface(hasGripper=True)
armState = unitree_arm_interface.ArmFSMState
arm.loopOn()

# 1. highcmd_basic : armCtrlInJointCtrl
arm.labelRun("forward")
arm.startTrack(armState.JOINTCTRL)
jnt_speed = 1.0
for i in range(0, 1000):
    # dp = directions * speed; include 7 joints
    arm.jointCtrlCmd([0,0,0,-1,0,0,-1], jnt_speed)
    time.sleep(arm._ctrlComp.dt)

# 2. highcmd_basic : armCtrlByFSM
arm.labelRun("forward")
gripper_pos = 0.0
jnt_speed = 2.0
arm.MoveJ([0.5,0.1,0.1,0.5,-0.2,0.5], gripper_pos, jnt_speed)
gripper_pos = -1.0
cartesian_speed = 0.5
arm.MoveL([0,0,0,0.45,-0.2,0.2], gripper_pos, cartesian_speed)
gripper_pos = 0.0
arm.MoveC([0,0,0,0.45,0,0.4], [0,0,0,0.45,0.2,0.2], gripper_pos, cartesian_speed)

# 3. highcmd_basic : armCtrlInCartesian
arm.labelRun("forward")
arm.startTrack(armState.CARTESIAN)
angular_vel = 0.3
linear_vel = 0.3
for i in range(0, 1000):
    arm.cartesianCtrlCmd([0,0,0,0,0,-1,-1], angular_vel, linear_vel)
    time.sleep(arm._ctrlComp.dt)

arm.backToStart()
arm.loopOff()
```
我的控制该设备运行的代码如上，请你帮我写一个名为unitreez1的ros2的包。


我现在有一个名为arm_python_interface.cpp的文件，其中的内容如下
```cpp
#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>
#include "unitree_arm_sdk/control/unitreeArm.h"

using namespace UNITREE_ARM;

class ArmInterface : public unitreeArm
{
public:
    ArmInterface(bool hasGripper):unitreeArm(hasGripper){};
    ~ArmInterface(){};
    void loopOn() { sendRecvThread->start();}
    void loopOff() { sendRecvThread->shutdown();}
    void setFsmLowcmd()
    {
        sendRecvThread->start();
        setFsm(ArmFSMState::PASSIVE);
        setFsm(ArmFSMState::LOWCMD);
        sendRecvThread->shutdown();
    }
    ArmFSMState getCurrentState()
    {
        return _ctrlComp->recvState.state;
    }
};

namespace py = pybind11;
PYBIND11_MODULE(unitree_arm_interface, m){
    using rvp = py::return_value_policy;

    m.def("postureToHomo", &postureToHomo);
    m.def("homoToPosture", &homoToPosture);

    py::enum_<ArmFSMState>(m, "ArmFSMState")
        .value("INVALID", ArmFSMState::INVALID)
        .value("PASSIVE", ArmFSMState::PASSIVE)
        .value("JOINTCTRL", ArmFSMState::JOINTCTRL)
        .value("CARTESIAN", ArmFSMState::CARTESIAN)
        .value("MOVEJ", ArmFSMState::MOVEJ)
        .value("MOVEC", ArmFSMState::MOVEC)
        .value("MOVEL", ArmFSMState::MOVEL)
        .value("TEACH", ArmFSMState::TEACH)
        .value("TEACHREPEAT", ArmFSMState::TEACHREPEAT)
        .value("TOSTATE", ArmFSMState::TOSTATE)
        .value("SAVESTATE", ArmFSMState::SAVESTATE)
        .value("TRAJECTORY", ArmFSMState::TRAJECTORY)
        .value("LOWCMD", ArmFSMState::LOWCMD)
        ;

    py::class_<LowlevelState>(m, "LowlevelState")
        .def("getQ", &LowlevelState::getQ, rvp::reference_internal)
        .def("getQd", &LowlevelState::getQd, rvp::reference_internal)
        .def("getQTau", &LowlevelState::getTau, rvp::reference_internal) // typo error in the original code
        .def("getTau", &LowlevelState::getTau, rvp::reference_internal)
        .def("getGripperQ", &LowlevelState::getGripperQ, rvp::reference_internal)
        ;

    py::class_<CtrlComponents>(m, "CtrlComponents")
        .def_readwrite("armModel", &CtrlComponents::armModel)
        .def_readonly("dt", &CtrlComponents::dt)
        ;

    py::class_<Z1Model>(m, "Z1Model")
        .def(py::init<Vec3, double, Vec3, Mat3>())
        .def("checkInSingularity", &Z1Model::checkInSingularity)
        .def("jointProtect", [](Z1Model& self, Vec6 q, Vec6 qd){
            self.jointProtect(q, qd);
            return std::make_pair(q, qd);
        })
        .def("getJointQMax", &Z1Model::getJointQMax, rvp::reference_internal)
        .def("getJointQMin", &Z1Model::getJointQMin, rvp::reference_internal)
        .def("getJointSpeedMax", &Z1Model::getJointSpeedMax, rvp::reference_internal)
        .def("inverseKinematics", [](Z1Model& self, HomoMat Tdes, Vec6 qPast, bool checkInWorkSpace){
            Vec6 q_result;
            bool hasIK = self.inverseKinematics(Tdes, qPast, q_result, checkInWorkSpace);
            return std::make_pair(hasIK, q_result);
        })
        .def("forwardKinematics", &Z1Model::forwardKinematics)
        .def("inverseDynamics", &Z1Model::inverseDynamics)
        .def("CalcJacobian", &Z1Model::CalcJacobian)
        .def("solveQP", [](Z1Model& self, Vec6 twist, Vec6 qPast, double dt){
            Vec6 qd_result;
            self.solveQP(twist, qPast, qd_result, dt);
            return qd_result;
        })
        ;

    py::class_<ArmInterface>(m, "ArmInterface")
        .def(py::init<bool>(), py::arg("hasGripper")=true)
        .def_readwrite("q", &ArmInterface::q)
        .def_readwrite("qd", &ArmInterface::qd)
        .def_readwrite("tau", &ArmInterface::tau)
        .def_readwrite("gripperQ", &ArmInterface::gripperQ)
        .def_readwrite("gripperQd", &ArmInterface::gripperW)
        .def_readwrite("gripperTau", &ArmInterface::gripperTau)
        .def_readwrite("lowstate", &ArmInterface::lowstate)
        .def_readwrite("_ctrlComp", &ArmInterface::_ctrlComp)
        .def("setFsmLowcmd", &ArmInterface::setFsmLowcmd)
        .def("getCurrentState", &ArmInterface::getCurrentState)
        .def("loopOn", &ArmInterface::loopOn)
        .def("loopOff", &ArmInterface::loopOff)
        .def("setFsm", &ArmInterface::setFsm)
        .def("backToStart", &ArmInterface::backToStart)
        .def("labelRun", &ArmInterface::labelRun)
        .def("labelSave", &ArmInterface::labelSave)
        .def("teach", &ArmInterface::teach)
        .def("teachRepeat", &ArmInterface::teachRepeat)
        .def("calibration", &ArmInterface::calibration)
        .def("MoveJ", py::overload_cast<Vec6, double>(&ArmInterface::MoveJ))
        .def("MoveJ", py::overload_cast<Vec6, double, double>(&ArmInterface::MoveJ))
        .def("MoveL", py::overload_cast<Vec6, double>(&ArmInterface::MoveL))
        .def("MoveL", py::overload_cast<Vec6, double, double>(&ArmInterface::MoveL))
        .def("MoveC", py::overload_cast<Vec6, Vec6, double>(&ArmInterface::MoveC))
        .def("MoveC", py::overload_cast<Vec6, Vec6, double, double>(&ArmInterface::MoveC))
        .def("startTrack", &ArmInterface::startTrack)
        .def("sendRecv", &ArmInterface::sendRecv)
        .def("setWait", &ArmInterface::setWait)
        .def("jointCtrlCmd", &ArmInterface::jointCtrlCmd)
        .def("cartesianCtrlCmd", &ArmInterface::cartesianCtrlCmd)
        .def("setArmCmd", &ArmInterface::setArmCmd)
        .def("setGripperCmd", &ArmInterface::setGripperCmd)
        ;
}
```
还有一个名为unitree_arm_interface.pyi的文件，内容如下
```python
import numpy as np
from enum import IntEnum

def postureToHomo(posture: np.ndarray, /) -> np.ndarray: ...

def homoToPosture(T: np.ndarray, /) -> np.ndarray: ...

class ArmFSMState(IntEnum):
    INVALID = 0,
    PASSIVE = 1,
    JOINTCTRL = 2,
    CARTESIAN = 3,
    MOVEJ = 4,
    MOVEL = 5,
    MOVEC = 6,
    TEACH = 7,
    TEACHREPEAT = 8,
    TOSTATE = 9,
    SAVESTATE = 10,
    TRAJECTORY = 11,
    LOWCMD = 12

class LowlevelState:
    def getQ(self) -> np.array: ...

    def getQd(self) -> np.array: ...
    
    def getTau(self) -> np.array: ...

    def getGripperQ(self) -> np.array: ...

class CtrlComponents:
    @property
    def armModel(self) -> Z1Model: ...
    
    @property
    def dt(self) -> float: ...

class Z1Model:
    def __init__(self, endPosLocal: np.array, endEffectorMass: float, endEffectorCom: np.array, endEffectorInertia: np.ndarray, /): ...

    def checkInSingularity(self, q: np.array, /) -> bool: ...

    def forwardKinematics(self, q: np.array, index, /) -> np.ndarray: ...

    def inverseKinematics(self, Tdes: np.ndarray, qPast: np.array, checkInWrokSpace: bool, /) -> bool: ...

    def solveQP(self, twist: np.array, qPast: np.array, dt: float, /): ...

    def CalcJacobian(self, q: np.array, /) -> np.ndarray: ...

    def inverseDynamics(self, q: np.array, qd: np.array, qdd: np.array, Ftip: np.array, /) -> np.array: ...

    def jointProtect(self, q: np.array, qd: np.array, /): ...

    def getJointQMax(self) -> np.array: ...

    def getJointQMin(self) -> np.array: ...

    def getJointSpeedMax(self) -> np.array: ...

class ArmInterface:
    def __init__(self, hasGripper: bool, /) -> None: ...

    def setFsm(self, fsm: ArmFSMState): ...

    def setFsmLowcmd(self): ...

    def getCurrentState(self) -> ArmFSMState: ...

    def loopOn(self): ...
    
    def loopOff(self): ...

    def backToStart(self): ...

    def labelRun(self, label: str, /): ...

    def labelSave(self, label: str, /): ...

    def teach(self, label: str, /): ...

    def teachRepeat(self, label: str, /): ...

    def calibration(self): ...

    def MoveJ(self, posture: np.ndarray, maxSpeed: float, /) -> bool: ...

    def MoveJ(self, posture: np.ndarray, gripperPos: float, maxSpeed: float, /) -> bool: ...

    def MoveL(self, posture: np.ndarray, maxSpeed: float, /) -> bool: ...

    def MoveL(self, posture: np.ndarray, gripperPos: float, maxSpeed: float, /) -> bool: ...

    def MoveC(self, middlePosture: np.ndarray, endPosture: np.ndarray, maxSpeed: float, /) -> bool: ...

    def MoveC(self, middlePosture: np.ndarray, endPosture: np.ndarray, gripperPos: float, maxSpeed: float, /) -> bool: ...

    def startTrack(self, fsm: ArmFSMState, /): ...

    def sendRecv(self): ...

    def setWait(self, Y_N: bool, /): ...

    def jointCtrlCmd(self, directions: np.ndarray, jointSpeed: float, /): ...

    def cartesianCtrlCmd(self, directions: np.ndarray, oriSpeed: float, posSpeed: float, /): ...

    def setArmCmd(self, q: np.ndarray, qd: np.ndarray, tau: np.ndarray, /): ...

    def setGripperCmd(self, gripperPos: float, gripperW: float, gripperTau: float, /): ...

    @property
    def lowstate(self) -> LowlevelState: ...

    @property
    def _ctrlComp(self) -> CtrlComponents: ...

    @property
    def q(self) -> np.array: ...

    @property
    def qd(self) -> np.array: ...

    @property
    def tau(self) -> np.array: ...

    @property
    def gripperQ(self) -> float: ...

    @property
    def gripperQd(self) -> float: ...

    @property
    def gripperTau(self) -> float: ...
```
我在运行下面文件的时候最开始会报错ModuleNotFoundError: No module named 'unitree_arm_interface'，在用cmake之后解决了这个错误。
我运行的程序如下
```python
import sys
sys.path.append("../lib")
import unitree_arm_interface
import time
import numpy as np
np.set_printoptions(precision=3, suppress=True)

print("Press ctrl+\ to quit process.")

arm =  unitree_arm_interface.ArmInterface(hasGripper=True)
armState = unitree_arm_interface.ArmFSMState
arm.loopOn()

# 1. highcmd_basic : armCtrlInJointCtrl
arm.labelRun("forward")
arm.startTrack(armState.JOINTCTRL)
jnt_speed = 1.0
for i in range(0, 1000):
    # dp = directions * speed; include 7 joints
    arm.jointCtrlCmd([0,0,0,-1,0,0,-1], jnt_speed)
    time.sleep(arm._ctrlComp.dt)

# 2. highcmd_basic : armCtrlByFSM
arm.labelRun("forward")
gripper_pos = 0.0
jnt_speed = 2.0
arm.MoveJ([0.5,0.1,0.1,0.5,-0.2,0.5], gripper_pos, jnt_speed)
gripper_pos = -1.0
cartesian_speed = 0.5
arm.MoveL([0,0,0,0.45,-0.2,0.2], gripper_pos, cartesian_speed)
gripper_pos = 0.0
arm.MoveC([0,0,0,0.45,0,0.4], [0,0,0,0.45,0.2,0.2], gripper_pos, cartesian_speed)

# 3. highcmd_basic : armCtrlInCartesian
arm.labelRun("forward")
arm.startTrack(armState.CARTESIAN)
angular_vel = 0.3
linear_vel = 0.3
for i in range(0, 1000):
    arm.cartesianCtrlCmd([0,0,0,0,0,-1,-1], angular_vel, linear_vel)
    time.sleep(arm._ctrlComp.dt)

arm.backToStart()
arm.loopOff()
```
请你帮我检查原因，我现在想在ROS2的框架下实现上述功能，也遇到了同样的问题。
我的CMakeLists.txt文件如下
```txt
cmake_minimum_required(VERSION 3.0)
project(z1_sdk)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -std=c++14 -pthread")

include_directories(
  include
)

link_directories(lib)

set(EXAMPLES_FILES
    examples/highcmd_basic.cpp
    examples/highcmd_development.cpp
    examples/lowcmd_development.cpp
    examples/lowcmd_multirobots.cpp
)

foreach(EXAMPLE_FILE IN LISTS EXAMPLES_FILES)
  get_filename_component(EXAMPLE_NAME ${EXAMPLE_FILE} NAME_WE)
  add_executable(${EXAMPLE_NAME} ${EXAMPLE_FILE})
  target_link_libraries(${EXAMPLE_NAME} Z1_SDK_${CMAKE_SYSTEM_PROCESSOR})
endforeach()


set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)
find_package(pybind11 QUIET)
if(${pybind11_FOUND})
  pybind11_add_module(unitree_arm_interface examples_py/arm_python_interface.cpp)
  target_link_libraries(unitree_arm_interface PRIVATE Z1_SDK_${CMAKE_SYSTEM_PROCESSOR})
endif()
```
请你帮我解决，我现在遇到的问题如下
```zsh
(dexmani)  sisyphus@sisyphus-dual4090  ~/ROS   master  ros2 run unitreez1 arm_controller

Traceback (most recent call last):
  File "/home/sisyphus/ROS/install/unitreez1/lib/unitreez1/arm_controller", line 33, in <module>
    sys.exit(load_entry_point('unitreez1==0.0.0', 'console_scripts', 'arm_controller')())
  File "/home/sisyphus/ROS/install/unitreez1/lib/unitreez1/arm_controller", line 25, in importlib_load_entry_point
    return next(matches).load()
  File "/usr/lib/python3.10/importlib/metadata/__init__.py", line 171, in load
    module = import_module(match.group('module'))
  File "/usr/lib/python3.10/importlib/__init__.py", line 126, in import_module
    return _bootstrap._gcd_import(name[level:], package, level)
  File "<frozen importlib._bootstrap>", line 1050, in _gcd_import
  File "<frozen importlib._bootstrap>", line 1027, in _find_and_load
  File "<frozen importlib._bootstrap>", line 1004, in _find_and_load_unlocked
ModuleNotFoundError: No module named 'unitreez1.arm_controller'
[ros2run]: Process exited with failure 1
```