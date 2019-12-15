## How to build with catkin

```
$ cd ~/catkin_ws/src/
$ git clone https://github.com/laboshinl/loam_velodyne.git
$ cd ~/catkin_ws
$ catkin_make -DCMAKE_BUILD_TYPE=Release 
$ source ~/catkin_ws/devel/setup.bash
```

## Running

```
roslaunch loam_velodyne loam_velodyne.launch
```

In second terminal play sample velodyne data from [VLP16 rosbag](http://www.frc.ri.cmu.edu/~jizhang03/Datasets/):
```
rosbag play ~/Downloads/velodyne.bag 
```

Or read from velodyne [VLP16 sample pcap](https://midas3.kitware.com/midas/folder/12979):
```
roslaunch velodyne_pointcloud VLP16_points.launch pcap:="$HOME/Downloads/velodyne.pcap"
```
## 本周工作
```
阅读并注释代码loam代码。/src、/src/lib、/include/loam_velodyne 内为主要代码

代码面向的是多线激光雷达，可以添加IMU，也可以不添加IMU

代码结构与论文保持一致，分为四个部分，点云注册、里程计、建图、集成四个节点通过消息传递进行联系。代码基于ROS编写，使用C++语言，风格面向对象，各种变量和函数在相应的类中进行定义，层次清晰.每个节点内代码结构基本一致，分为三层----节点启动层、中间层、基础功能层。节点启动部分保持简洁的代码，主要功能在基础功能层（/lib中Basic—文件）中实现，进而中间层（ScanRegistration、LaserOdometry、LaserMapping、TransformMaitenance等文件）在其基础上添加与ROS运行相关的功能，发布订阅消息，完成消息回调。

在每个节点中，由中间层setup函数设置节点运行的整体状况，由基础功能层的process函数完成节点核心任务。

```
