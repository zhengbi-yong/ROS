# 终端重启后需要运行的命令
conda activate dexmani
source install/setup.zsh
source /opt/ros/humble/setup.zsh
sudo chmod 666 /dev/ttyUSB0

# 修改代码之后的编译流程
rm -rf build/ install/ log/
colcon build --symlink-install
source install/setup.zsh      
source /opt/ros/humble/setup.zsh
conda activate dexmani        
source install/setup.zsh
source /opt/ros/humble/setup.zsh
sudo chmod 666 /dev/ttyUSB0
ros2 launch leap_hand launch_leap.py

# 常见任务的命令
## 自动摆动
ros2 launch leap_hand launch_leap.py
## 键盘控制
ros2 run leap_hand leaphand_node.py
ros2 run leap_hand keyboard_control.py

# 其他命令
## 创建虚拟环境
conda create -n dexmani python=3.10
conda activate dexmani

## 激活humble
source /opt/ros/humble/setup.zsh
## 使用虚拟环境中的python解释器
export PYTHONPATH=/home/sisyphus/anaconda3/envs/dexmani/lib/python3.10/site-packages:$PYTHONPATH
export ROS_PYTHON_VERSION=3
export PYTHONPATH=/opt/ros/humble/lib/python3.10/site-packages:$PYTHONPATH

## 常见问题
### em相关
pip install empy==3.3.4
source install/setup.zsh
### 授予权限
sudo chmod 666 /dev/ttyUSB0
### 虚拟环境中安装c++编译
conda install -c conda-forge libstdcxx-ng=12