# Aelos_Smart_Ros

## 准备工作  

### 由于GPIO模块需要sudo权限，而摄像头sudo后图像显示会更新不出来，为解决冲突，需先进行配置,先根据链接进行配置文件    
[配置链接](https://blog.csdn.net/weixin_51852924/article/details/119843874)  
#### 注意在 /etc/udev/rules.d/ 路径下创建文件应加sudo权限  


### 需设置sudo免密才可使用wifi扫描及连接的功能  
```
打开/etc/sudoers文件添加:
lemon ALL=(ALL) NOPASSWD: ALL
```

### 开机自启动介绍  
开机自启动节点由 supervisor 实现  
相关配置文件路径: `/etc/supervisor/conf.d`   
输入：`sudo supervisorctl` 可进入 supervisor 交互界面  
可使用 start 、stop 等指令开启或停止相应文件，例如： `stop lejuros`  
[supervisor相关介绍](http://supervisord.org/)

### 使用终端运行地磁校正脚本  
指令：  
```
python ~/catkin_ws/src/aelos_smart_ros/scripts/topic_node/magnet_calibration.py
```
运行此文件后，会显示出一个圆盘，将机器人旋转一周采集数据，每个点需采集十个数据，采集完成会显示 `#` 号，当所有点均采集完成时，则完成校正，可通过如下指令查看地磁数值  
```
rostopic echo /sensor_info | grep mag:
```


### 各接口说明

**sensor_port.set_input(io)**  
设置io状态为输入模式  
io : int ; 1或是2  
返回值：None  
 

**sensor_port.set_output(io,vol)**  
设置io状态为输出模式  
io : int ; 1或是2  
返回值：None  


**sensor_port.get_magnet()**  
获取地磁值  
无参数  
返回值：地磁值  


**sensor_port.get_gpio(io)**  
获取io端口的adc值  
io : int ; 1或是2  
返回值：io端口的adc    


**sensor_port.get_io_status(io)**  
获取io端口状态和adc值  
io : int ; 1或是2  
返回值：io端口状态(0：input ，1：output)，及adc值，当状态为 output 时，值为0或1，代表低电平与高电平  


**base_action.action(act_name)**  
调用此函数使机器人做动作  
act_name：string ; 动作指令 例：Squat  
返回值：None  



**artag_port.tag_id()**  
调用此参数获取距离最近的tag id  
无参数  
返回值：id , 未识别到 tag 则返回 nan  


**artag_port.tag_x()**  
调用此参数获取距离最近的tag的x值  
无参数  
返回值：x值 , 未识别到 tag 则返回 nan  


**artag_port.tag_y()**  
调用此参数获取距离最近的tag的y值  
无参数  
返回值：y值 , 未识别到 tag 则返回 nan  



**artag_port.tag_yaw()**  
调用此参数获取距离最近的tag角度  
无参数  
返回值：tag的角度值 , 未识别到 tag 则返回 nan  


**artag_port.get_specifies_tag(id)**  
调用此接口获取指定id的tag信息  
id：想要获取的tag信息的id   
返回值：tag的 x, y, yaw   
如果无对应tag返回 None, None, None  
 

**colour_port.get_central_coordinate(camera, H_MIN, S_MIN, V_MIN, H_MAX , S_MAX , V_MAX)**  
获取单个颜色区域的中心坐标  
camera：string ;  head或chest  
H_MIN,H_MAX：string ; 范围在0~180  

S_MIN,S_MAX：string ; 范围在0~255  

V_MIN,V_MAX：string ; 范围在0~255  
返回值：区域中心坐标，不存在返回(-1，-1)  

**colour_port.have_color(camera, H_MIN, S_MIN, V_MIN, H_MAX , S_MAX , V_MAX)**  
判断颜色是否存在  
camera：string ;  head或chest  
H_MIN,H_MAX：string ; 范围在0~180  

S_MIN,S_MAX：string ; 范围在0~255  

V_MIN,V_MAX：string ; 范围在0~255  
返回值：True或False  


**colour_port.get_color_size(camera, H_MIN, S_MIN, V_MIN, H_MAX , S_MAX ,V_MAX)**  
获取颜色面积  
camera：string ;  head或chest  
H_MIN,H_MAX：string ; 范围在0~180  

S_MIN,S_MAX：string ; 范围在0~255  

V_MIN,V_MAX：string ; 范围在0~255  
返回值：颜色面积,如果无对应颜色则返回0

**colour_port.get_color_percent(camera, H_MIN, S_MIN, V_MIN, H_MAX , S_MAX ,V_MAX)**  
获取颜色面积占比  
camera：string ;  head或chest  
H_MIN,H_MAX：string ; 范围在0~180  

S_MIN,S_MAX：string ; 范围在0~255  

V_MIN,V_MAX：string ; 范围在0~255  
返回值：面积百分比,如果无对应颜色则返回0


**colour_port.get_frame(camera, H_MIN, S_MIN, V_MIN, H_MAX , S_MAX ,V_MAX , return_value)**  
获取外接矩形框  
camera：string ;  head或chest  
H_MIN,H_MAX：string ; 范围在0~180  

S_MIN,S_MAX：string ; 范围在0~255  

V_MIN,V_MAX：string ; 范围在0~255  
return_value: string ; x、y、w、h其中一个  
返回值：根据 return_value 的值返回，如果无对应颜色则返回False  

**get_key.key()**  
获取遥控器键值  
无参数
返回值：按键键值  

**emoty_key.key()**  
清空遥控器键值  
无参数
返回值：True  

**get_websocket_msg.main()**  
在桌面软件中获取到打印的信息  
将需要打印的信息填入()中



### 自定义节点说明  

* 自定义节点运行说明,执行指令进行克隆  
    ```
    cd /home/lemon/catkin_ws/src/custom_nodes
    git clone ssh://git@www.lejuhub.com:10026/aelos_edu/aelos_smart_node_template.git
    ``` 
    克隆后，执行 `cd ~/catkin_ws/` ,到指定路径，再执行    `catkin_make` 进行编译，编译前需要关闭overlayroot  
    关闭方法：  
    ```
    cd /home/lemon/catkin_ws/tools
    sudo ./disable_overlay_after_reboot.sh
    ```
    重新开启方法：  
    ```
    cd /home/lemon/catkin_ws/tools
    sudo ./enable_overlay_after_reboot.sh
    ```
    注意：如果reboot无法重启时，可关机重启机器人  
    编译成功后，可使用roslaunch或rosrun指令启动节点，roslaunch可一次启动多个节点，rosrun一次启动一个节点。  
* aelos_smart_node_template结构如下：  
    ```
    ├── CMakeLists.txt
    ├── include
    │   └── aelos_smart_node_template
    ├── launch
    │   └── demo.launch
    ├── package.xml
    ├── README.md
    └── src
        └── demo.py
    ```
    src下的demo.py是我们的节点，此节点会间隔一秒输出hello world，我们可以通过指令：`rosrun aelos_smart_node_template demo.py` 运行此节点(使用rosrun的话需确保当前roscore已运行)，也可以通过`roslaunch aelos_smart_node_template demo.launch` 运行demo.launch文件启动节点，在launch文件中可编写多个节点一起启动。  


* 如需添加自定义节点，可继续在src下添加对应节点文件，或者在custom_noses下新建pkg，注意新建pkg后需再次编译，新建指令： `catkin_create_pkg pkg名称 rospy rosmsg roscpp` 


### sys_update说明

* 升级命令
```
sh -c "$(wget --no-cache https://aelosstatic.lejurobot.com/aelos_smart/update.sh -O -)"
```

!注意: 升级脚本会重置仓库里面的代码，如果有修改，请自行保存到 U 盘或者上传到自己的服务器。

此命令将会更新当前master上的代码，并执行编译，如出错会在 `/mnt/leju_data/update.log` 中记录时间以及错误信息，可通过此文件查看具体错误内容



## websocket

本文档主要描述 桌面端软件通过 websocket 方式获取和控制 aelos smart 协议

### 通讯数据概述

websocket 服务端口 9200

处理和接收 json 格式

| 命令  | 参数   |
| --- | ---- |
| cmd | data |

```json
{
 "cmd": "",
 "data": ""
}
```

### cmd 命令列表

列表中的命令需要声明在

aelos_smart_ros/scripts/socket_node/WSconnect_node.py 中的

`COMMAND_DICT`

| 命令                     | 参数      | 说明                    |
| ---------------------- | ------- | --------------------- |
| mkdir                  | 新建目录的路径 | 新建文件夹，路径是用户文件夹的相对路径   |
| run                    | 项目目录名   | 运行用户代码，目录名是用户文件夹的相对路径 |
| stop                   |         | 停止运行用户代码              |
| start_calibrate_magnet |         | 开始地磁校正                |
| stop_calibrate_magnet  |         | 停止地磁校正                |
| get_magnet_status      |         | 获取当前地磁状态              |
| get_robot_status       |         | 获取当前 IP 及图像地址              |

### 具体传输和相应格式

### mkdir

创建目录名，无返回

### run

运行由桌面软件下载的用户代码。

发送样式

```json
{
     "cmd": "data",
     "data": {
        "project_name":""    
    }
}
```

**返回内容**

用户文件夹下的 project_name 包含 main.py 时，返回

```json
{"cmd": 运行命令 ,"code":0,"runState":0}
```

cmd 会返回运行的命令

如果不包含 main.py 文件时，返回

```json
{"cmd": 运行命令,"code": 错误码,"runState":0,"msg":"No files or directories"}
```

 运行用户的 main.py 非 stop 时，返回

```json
{
    "cmd":运行命令,
    "code":返回值,
    "runState":1,
    "stdout": 程序输出,
    "stderr": 程序报错
}
```

### start_calibrate_magnet

地磁状态码：[aelos-eduNG树莓派通讯接口.md DOCS (lejuhub.com)](https://www.lejuhub.com/carlos/DOCS/-/blob/master/aelos-eduNG%E6%A0%91%E8%8E%93%E6%B4%BE%E9%80%9A%E8%AE%AF%E6%8E%A5%E5%8F%A3.md#9-%E4%B8%8A%E4%BD%8D%E6%9C%BA%E5%BC%80%E5%90%AF%E6%A0%91%E8%8E%93%E6%B4%BE%E5%9C%B0%E7%A3%81%E4%BC%A0%E6%84%9F%E5%99%A8%E6%A0%A1%E5%87%86)

进入校正状态

发送

```json
{"cmd": "start_calibrate_magnet", "data": {} }
```

返回，超时时间：1s （超过1s没有返回即可认为超时）

```json
 {"cmd": "start_calibrate_magnet", "mag_status": 2 }
```

### stop_calibrate_magnet

只要在校正状态下才会，退出校正状态。如果不处于校正状态，会直接返回当前地磁状态

发送

```json
{"cmd": "stop_calibrate_magnet", "data": {} }
```

返回，因为停止校正需要时间这里会延迟 1秒 返回。超时时间 2s

```json
{"cmd": "stop_calibrate_magnet", "mag_status": 3}
```

### get_magnet_status

获取当前地磁状态

发送

```json
{"cmd": "get_magnet_status", "data": {} }
```

返回 超时时间 1s

```json
{"cmd": "get_magnet_status", "mag_status": 3}
```

### check_program_running  

检查当前进程中是否存在 main.py 文件  
调用后如果存在则会返回 pid 的信息，如果不存在则返回 None  

### check_magnet_status

获取当前地磁状态，不会通过 websocket 与桌面端通信  
具体作用为获取状态进行判断当前地磁校正程序是否正在运行，决定是否要停止程序  

### stop_magnet  

停止地磁校正程序  

### get_robot_status  

获取机器人 IP 以及图像地址  

发送  

```json
{"cmd": "get_robot_status", "data": {} }
```

返回  
```json
{"cmd": "get_robot_status", "msg":{"ip":"robot_ip","chest_image":"chest_image","head_image":"head_image"}}
