#! /bin/bash


cd /home/lemon/catkin_ws/src/

robot_demo=/home/lemon/catkin_ws/src/robot_demo
college_caai=/home/lemon/catkin_ws/src/college_caai
college_caai_zip=/home/lemon/catkin_ws/src/college_caai.zip


if [ -d "$robot_demo" ];then 
echo "清理旧版robot_demo文件夹"
rm -rf robot_demo/
fi


#判断文件是否存在
if [ -d "$college_caai" ] || [ -f "$college_caai_zip" ];then 
    echo "----------------------------------"
    echo "college_caai文件存在,是否进行删除并重新下载:"
    echo "(1) 删除并重新下载"
    echo "(2) 取消"
    echo "----------------------------------"

    read -p "请输入序列号："  input

    case $input in
        1)
        echo "删除中..."
        rm -rf college_caai/
        rm -f college_caai.zip 
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
wget --no-cache -O college_caai.zip http://aelosedu.lejurobot.com/college_caai.zip

echo "解压..."
unzip -o college_caai.zip 
