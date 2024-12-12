## 毕设项目：风力发电机叶片攀爬机器人运动控制研究

### 各个功能包介绍

sipder_robot：

由solidworks导出的功能包，内有生成的urdf和meshes、launch文件，自己转成的xacro文件，增加各种如三维雷达、imu、深度相机的gazebo仿真文件。

legged_control：

关节控制器定义文件

spibot_control：

机器人控制主文件

spibot_plugin：

修改的unitree机器人插件，仿真绘图需要

### 启动机器人仿真

从SW导出的urdf文件，位于src/sipder_robot文件夹内，展示机器人命令：

    roslaunch sipder_robot display.launch 

在legged_control功能包定义机器人的电机控制和pid配置robot_control.yaml，启动这个动能包的命令在launch文件夹里

    roslaunch legged_control legged_control.launch

启动总机器人控制节点，机器人会按照一定规律摆动腿，总控制节点在spibot_control功能包的launch文件里

    roslaunch spibot_control set_pub.launch 

launch文件不包含python文件，而键盘发布信息节点是用python写的，需要另外启动(可以单独启动，但是需要roscore，已经启动了launch文件就不需要)

    rosrun spibot_control teleop_keyboard.py
