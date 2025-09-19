#! /bin/bash


cd /home/lemon/catkin_ws/src/

robot_demo=/home/lemon/catkin_ws/src/robot_demo
RunningRobot=/home/lemon/catkin_ws/src/RunningRobot
RunningRobot_zip=/home/lemon/catkin_ws/src/RunningRobot.zip


if [ -d "$robot_demo" ];then 
echo "清理旧版robot_demo文件夹"
rm -rf robot_demo/
fi


#判断文件是否存在
if [ -d "$RunningRobot" ] || [ -f "$RunningRobot_zip" ];then 
    echo "----------------------------------"
    echo "RunningRobot文件存在,是否进行删除并重新下载:"
    echo "(1) 删除并重新下载"
    echo "(2) 取消"
    echo "----------------------------------"

    read -p "请输入序列号："  input

    case $input in
        1)
        echo "删除中..."
        rm -rf RunningRobot/
        rm -f RunningRobot.zip 
        ;;

        2)
        exit
        ;;

        *)
        echo "无对应选项"
        exit
        ;;
    esac

fi


echo "开始下载压缩包..."
wget --no-cache -O RunningRobot.zip http://aelosedu.lejurobot.com/RunningRobot.zip

echo "解压..."
unzip -o RunningRobot.zip -d ./RunningRobot/

