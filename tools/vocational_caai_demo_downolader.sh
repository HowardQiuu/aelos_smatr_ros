#! /bin/bash


cd /home/lemon/catkin_ws/src/

robot_demo=/home/lemon/catkin_ws/src/robot_demo
vocational_caai=/home/lemon/catkin_ws/src/vocational_caai
vocational_caai_zip=/home/lemon/catkin_ws/src/vocational_caai.zip


if [ -d "$robot_demo" ];then 
echo "清理旧版robot_demo文件夹"
rm -rf robot_demo/
fi


#判断文件是否存在
if [ -d "$vocational_caai" ] || [ -f "$vocational_caai_zip" ];then 
    echo "----------------------------------"
    echo "vocational_caai文件存在,是否进行删除并重新下载:"
    echo "(1) 删除并重新下载"
    echo "(2) 取消"
    echo "----------------------------------"

    read -p "请输入序列号："  input

    case $input in
        1)
        echo "删除中..."
        rm -rf vocational_caai/
        rm -f vocational_caai.zip 
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
wget --no-cache -O vocational_caai.zip http://aelosedu.lejurobot.com/vocational_caai.zip

echo "解压..."
unzip -o vocational_caai.zip 
