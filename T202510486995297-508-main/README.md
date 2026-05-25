## 小米杯——菜狗驿站

学校：武汉大学

用户名：T202510486995297

参赛队伍名：菜狗驿站

### 相关资料

设计文档：设计文档.pdf
源程序：code文件夹
作品ppt：作品说明.pptx
视频录像：视频已上传到项目下，通关视频.mp4即为完整通关视频；也上传到了百度网盘，网盘链接：[https://pan.baidu.com/s/1DKGAtal0QWExyEz-VEFIUQ?pwd=cm6u]提取码：cm6u

### 源代码介绍

**源代码简介**：

- lcm文件夹：里面包含了robot_control_cmd，robot_control_response，simulator_state和user_gait_file四个lcm通信接口。
- toml文件夹：记录了各种步态文件。
  - usergait.toml：赛段所需基本步态的定义
  - usergait_def.toml/usergait_param.toml：自定义蹄行步态的定义
- xacro文件夹：包含了要在cyberdog_simulator中修改的xacro文件，增加RGB模块。
- code的文件中：
  - test.py是机器狗运行的入口，它会调用Msg_receiver.py来接收机器狗和周围环境的信息，并调用Robot_Ctrl.py来根据信息选择合适的步态并执行。
  - monitoring.py、monitoring2.py、monitoring3.py和monitoring4.py文件分别对应着对robot_control_response、global_to_robot、simulator_state和leg_control_data四个lcm通信接口；
  - identify.py通过调用ros2节点/rgb_camera/image_raw来获取图片并对图片进行处理，二维码识别、箭头方向识别、黄灯距离识别、装货等待、黄灯等待的接口都在其中。
  - user_pub.py通过调用file_send_lcmt()将自定义蹄行步态发送出去，之后可直接通过mode=62 gait=110调用该步态

**运行说明**：

- 将code文件夹放置/home/cyberdog_sim下。
- 先在/home/cyberdog_sim下启动机器狗的仿真平台。
- 再进入code目录运行test.py,即可运行机器狗的通关过程
```bash
cd code
python3 test.py
```

### 其它修改

**1.rgb相机的添加**

在/home/cyberdog_sim/src/cyberdog_simulator/cybedog_robot/cyberdog_description/xacro中更改gazebo.xacro文件，将这个文件替换为源程序下/xacro/gazebo.xacro.

**2.运行状态检测**

在/home/cyberdog_sim/src/cyberdog_locomotion/control/src/convex_mpc/convex_mpc_loco_gaits.cpp文件中，
第13行位置，打开USE_TERRAIN_DETECTER宏定义，然后删除/home/cyberdog_sim下面的build，install和log文件夹，
然后在/home/cyberdog_sim文件夹下面使用
`source /opt/ros/galactic/setup.bash `和
`colcon build --merge-install --symlink-install --packages-up-to cyberdog_locomotion cyberdog_simulator`
指令进行重新编译。

**3.抬腿高度修改**

- 在/home/cyberdog_sim/src/cyebrdog_locomotion/common/config/cyberdog2-ctrl-user-parameters.yaml中修改213行抬腿最大高度的设置

```bash
  step_height_max   = 0.20;
```

   - 在/home/cyberdog_sim/src/cyebrdog_locomotion/control/src/convex_mpc/convex_mpc_loco_gaits.cpp中修改181行为

```bash
   step_height_max_   = 0.20;
```

   - 在/home/cyberdog_sim/src/cyebrdog_locomotion/control/src/convex_mpc/convex_mpc_motion_gaits.cpp中修改158行

```bash
  step_height_max_   = 0.20;
```