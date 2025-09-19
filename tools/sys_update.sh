#! /bin/bash
set -e

SUDOERS_FILE="/etc/sudoers"
SUDOERS_STR="lemon ALL=(ALL) NOPASSWD: ALL"
USB_CAM_FILE="/home/lemon/catkin_ws/src/usb_cam"
CATKIN_MAKE="export ROS_PARALLEL_JOBS='-j2 -l2'"
BASHRC_PATH="/home/lemon/.bashrc"


if [ `echo 'leju123' | sudo -S grep -c "$SUDOERS_STR" $SUDOERS_FILE` -ne '0 ' ];then   #检查是否已配置sudo免密
    echo "当前已为免密状态"
else
    echo "添加免密权限"
    sudo sh -c "echo 'lemon ALL=(ALL) NOPASSWD: ALL' >> /etc/sudoers" 
fi

echo "检查是否设置编译的参数为 -j2 -l2 为了防止编译时 CPU 占用率过高导致机器人死机"

if [ `grep -c "$CATKIN_MAKE" $BASHRC_PATH` -ne '0 ' ];then   
    echo "发现已设置 catkin_make 的环境变量，可以跳过"
else
    echo "将 catkin_make 参数设置为 -j2 -l2"
    sed -i "/ROS_PARALLEL_JOBS/d" $BASHRC_PATH
    sed -i '$a'"$CATKIN_MAKE" $BASHRC_PATH
fi


if [ ! -d "$USB_CAM_FILE" ];then
    echo y | sudo apt-get purge ros-noetic-usb-cam  
    echo "下载usb_cam"
    cd /home/lemon/catkin_ws/src
    wget --no-cache -O usb_cam.zip https://aelosstatic.lejurobot.com/aelos_smart_img%2Fusb_cam_220920.zip 
    unzip -o usb_cam.zip 
    cd /home/lemon/catkin_ws
    catkin_make
else
    echo "usb_cam功能包已存在src目录下"
fi
