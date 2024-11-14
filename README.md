# 服务器重启之后需要运行下面的命令来让灵巧手正常工作
conda activate dexmani
source install/setup.zsh
sudo chmod 666 /dev/ttyUSB0
ros2 launch leap_hand launch_leap.py





conda create -n dexmani python=3.10
conda activate dexmani

# 激活humble
source /opt/ros/humble/setup.zsh
# 使用虚拟环境中的python解释器
export PYTHONPATH=/home/sisyphus/anaconda3/envs/dexmani/lib/python3.10/site-packages:$PYTHONPATH
export ROS_PYTHON_VERSION=3
export PYTHONPATH=/opt/ros/humble/lib/python3.10/site-packages:$PYTHONPATH

# !!!!
pip install empy=3.3.4
pip install catkin_pkg
pip install numpy 
pip install lark-parser
pip install rclpy

source /opt/ros/humble/setup.zsh


source install/setup.zsh
sudo chmod 666 /dev/ttyUSB0
ros2 launch leap_hand launch_leap.py

sudo apt-get update
sudo apt-get install libstdc++6
sudo apt-get install gcc-12 g++-12
export LD_LIBRARY_PATH=/usr/lib/x86_64-linux-gnu:$LD_LIBRARY_PATH

rm -rf build/ install/ log/
colcon build --symlink-install
export LD_LIBRARY_PATH=/opt/ros/humble/lib:$LD_LIBRARY_PATH

conda install -c conda-forge libstdcxx-ng=12

