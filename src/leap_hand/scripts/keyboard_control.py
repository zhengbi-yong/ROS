#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from leap_hand.srv import LeapPosition  # 导入服务类型
from pynput import keyboard

class KeyboardControlNode(Node):

    def __init__(self):
        super().__init__('keyboard_control_node')
        self.joint_positions = [0.0] * 16  # 假设有16个关节

        # 创建服务客户端
        self.cli = self.create_client(LeapPosition, 'leap_position')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('等待 leap_position 服务可用...')
        self.get_current_positions()

        self.pub = self.create_publisher(JointState, 'cmd_leap', 10)
        self.get_logger().info('键盘控制节点已启动。')
        self.get_logger().info('使用按键 1-0 增大关节 0-9，q-p 减小关节 0-9。')
        self.get_logger().info('使用按键 a-h 增大关节 10-15，z-n 减小关节 10-15。')
        self.listener = keyboard.Listener(on_press=self.on_press)
        self.listener.start()

    def get_current_positions(self):
        req = LeapPosition.Request()
        future = self.cli.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            self.joint_positions = list(future.result().position)
            self.get_logger().info('已获取机械手的当前关节位置。')
        else:
            self.get_logger().error('无法获取机械手的当前关节位置，使用默认值。')

    def on_press(self, key):
        try:
            char = key.char.lower()  # 将字符转换为小写，兼容大写输入
            # 定义按键映射
            increase_keys_0_9 = ['1','2','3','4','5','6','7','8','9','0']
            decrease_keys_0_9 = ['q','w','e','r','t','y','u','i','o','p']
            increase_keys_10_15 = ['a','s','d','f','g','h']
            decrease_keys_10_15 = ['z','x','c','v','b','n']

            # 增大关节 0-9
            if char in increase_keys_0_9:
                idx = increase_keys_0_9.index(char)
                if idx < len(self.joint_positions):
                    self.joint_positions[idx] += 0.1
                    self.publish_joint_states()
                    self.get_logger().info(f'增大关节 {idx} 到 {self.joint_positions[idx]:.2f}')
            # 减小关节 0-9
            elif char in decrease_keys_0_9:
                idx = decrease_keys_0_9.index(char)
                if idx < len(self.joint_positions):
                    self.joint_positions[idx] -= 0.1
                    self.publish_joint_states()
                    self.get_logger().info(f'减小关节 {idx} 到 {self.joint_positions[idx]:.2f}')
            # 增大关节 10-15
            elif char in increase_keys_10_15:
                idx = increase_keys_10_15.index(char) + 10  # 关节索引从 10 开始
                if idx < len(self.joint_positions):
                    self.joint_positions[idx] += 0.1
                    self.publish_joint_states()
                    self.get_logger().info(f'增大关节 {idx} 到 {self.joint_positions[idx]:.2f}')
            # 减小关节 10-15
            elif char in decrease_keys_10_15:
                idx = decrease_keys_10_15.index(char) + 10  # 关节索引从 10 开始
                if idx < len(self.joint_positions):
                    self.joint_positions[idx] -= 0.1
                    self.publish_joint_states()
                    self.get_logger().info(f'减小关节 {idx} 到 {self.joint_positions[idx]:.2f}')
        except AttributeError:
            pass  # 忽略特殊按键

    def publish_joint_states(self):
        msg = JointState()
        msg.position = self.joint_positions
        self.pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = KeyboardControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
