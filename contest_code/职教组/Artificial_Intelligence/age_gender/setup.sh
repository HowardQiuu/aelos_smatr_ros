#!/bin/bash

# Install Python dependencies
pip config set global.index-url https://pypi.tuna.tsinghua.edu.cn/simple
sudo python3 -m pip install pip --upgrade
python3 -m pip install -i https://pypi.tuna.tsinghua.edu.cn/simple --upgrade pip # 通过 sudo 升级 pip 修复 pip 0.99 lemon无法pip install的问题
pip install -r /home/lemon/catkin_ws/src/aelos_smart_ros/contest_code/职教组/Artificial_Intelligence/age_gender/requirements.txt
