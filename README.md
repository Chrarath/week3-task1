# week3-task1
第三周任务一
## 一、运行环境

Ubuntu 20.04
ROS Noetic
PX4-Autopilot v1.14 SITL 仿真
通信驱动：MAVROS
### 功能包依赖

rospy、geometry\_msgs、mavros\_msgs

## 二、目录结构

```
week3_offboard/
├── CMakeLists.txt         
├── package.xml             
├── launch/                 # 一键启动配置文件夹
├── scripts/
│   └── offboard_square.py  # 飞行控制Python主脚本
└── .gitignore
```              

## 三、代码运行

1.下载代码

```bash
cd ~/catkin_ws/src
git clone https://github.com/Chrarath/week3-task1 week3_offboard
```
2.编译 catkin 工作空间

```bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```
3.终端 1：启动 PX4 SITL 仿真 + Gazebo 物理仿真环境

```bash
cd ~/PX4-Autopilot
make px4_sitl gazebo
```
4.终端 2：启动 MAVROS 通信节点

```bash
source /opt/ros/noetic/setup.bash
roslaunch mavros px4.launch fcu_url:=udp://127.0.0.1:14550@127.0.0.1:14540
```

通信连通校验

```bash
rostopic echo /mavros/state
```

输出字段中 `connected: True` 即代表 ROS 上位机与 PX4 飞控通信建立成功。

5.终端 3：运行 Offboard 飞行控制脚本

```bash
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash
rosrun week3-task1 offboard_square.py
```
程序自动按照预设逻辑逐阶段执行飞行任务，终端实时打印每一步运行日志。

6.录屏展示

https://github.com/user-attachments/assets/5add5ca1-5ff1-4af2-bdb4-272e37d19e4d

## 四、rqt_graph 可视化解析

<img width="1830" height="1089" alt="9be596f91c44e10c020b60a01780159d" src="https://github.com/user-attachments/assets/a2fbd9b1-43c3-4dcf-8cb2-6309704422ba" />

### 核心节点

1. `/offboard_square_node`：自定义飞行控制节点
2. `/mavros`：MAVROS 协议中转节点，实现 ROS 与 PX4 飞控协议互转

### 关键话题通信

1. **下发指令（节点→mavros）**`/mavros/setpoint_position/local`：持续发布目标坐标，下发飞行点位，是 Offboard 位置控制核心话题。
2. **上行反馈（mavros→节点）**

- `/mavros/state`：获取飞控连接、飞行模式、电机解锁状态；
- `/mavros/local_position/pose`：读取无人机实时三维坐标，用于航点抵达判定。

## 五、异常处理设计

1.**飞控未连接异常等待处理**
程序启动后循环检测`/mavros/state`连通状态，飞控未接入则持续打印日志等待

2.**航点抵达超时容错机制**
自定义`wait_reach()`函数为每个航点设置最大等待时长，虚拟机卡顿、物理仿真缓慢长时间无法抵达点位时，判定超时抛出警告，避免程序永久卡死。

3.**Ctrl+C 强制中断安全退出**
捕获`ROSInterruptException`系统中断异常，节点平稳关闭，杜绝无人机失控乱飞问题。
