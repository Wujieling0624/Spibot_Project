# SW_to_urdf注意事项
## 1、Exporter to urdf
<mark>这个插件导出的urdf只适用于ros1</mark>
插件下载方式参考CSDN:https://blog.csdn.net/weixin_45168199/article/details/105755388

## 2、在装配体中建立坐标轴
<mark>坐标轴建立需要在装配体中建立，不能在零件中建立</mark>
tip1、按照D-H参数表建立坐标系的规范，用z表示旋转轴，x朝向下一级坐标系原点，一般平行于连杆。 

![C:\WJL\学习\机器人2102吴洁铃\学习文档\图片\机械臂建坐标系.jpg](图片\机械臂建坐标系.jpg)左图标准DH,右图改进DH

z轴和下一级z平行时，x轴指向下一级坐标系原点，z轴与下一级z垂直，x方向为z和下一级z的叉积。
tip2、可以先在关节处建立基准点（一般关节都是电机，利用圆建立基准点），利用基准点建立坐标系。
tip3、不需要建立基准轴，导出urdf时可以自行生成。

## 3、导出urdf
SW中的工具—>tools—>Export as URDF
也参考CSDN:https://blog.csdn.net/weixin_45168199/article/details/105755388
![C:\WJL\学习\机器人2102吴洁铃\学习文档\图片\export_as_urdf1.png](图片\export_as_urdf1.png)
1、连杆名称 xxx_Link
2、关节名称 xxx_Joint
3、连杆对应坐标系 我一般跟连杆命名一致
4、关节对应基准轴名称 一般不自己选择，Auto
5、关节类型 连续旋转continuous/有限制旋转revolute/平移/固定
6、对应实际连杆 选中sw画的连杆
7、子连杆数目 根据实际
全部配置完后出现
![C:\WJL\学习\机器人2102吴洁铃\学习文档\图片](图片\2.png)
进行关节配置，一半不用管，如果有revolute需要额外配置lower角度(rad)、upper角度(rad)、effort扭矩（N/m）、velocity速度（rad/s）
![C:\WJL\学习\机器人2102吴洁铃\学习文档\图片](图片\3.png)
进行连杆配置,一般不用管，输出到rviz中为白色，需要修改连杆颜色在此处更改。
导出工作结束，导出的文件为功能包，在ubuntu中建立工作空间，转移文件到src下，catkin_make,需要在ros1的rviz中显示才能确保导出装配体的正确，可能出现整个机器人反转90°的情况，此时在.urdf中增加word连杆，利用关节wor2baselink将yaw/pitch/roll轴旋转90°。

## ROS2:导出的urdf导入到rviz中
参考CSDN：https://zhuanlan.zhihu.com/p/465398486



