# 基于点云的路面检测

本项目是一个 `ROS package`。支持版本 `neotic(ubuntu 20.04)`。

## 依赖库
本项目核心算法只使用了 `C++` 标准库，点云显示由 `rviz` 实现。因此，不需要安装其他依赖库。

## 快速开始
1. 把本文件夹复制到 `catkin_ws/src`。
2. 在 `catkin_ws/` 目录下运行以下命令编译。
```bash
catkin_make
```
3. 配置环境变量 
```bash
source ./devel/setup.bash
```
4. 运行（注意修改点云数据文件夹 `data_dir:=...`）
```bash
roslaunch roaddetection rviz.launch data_dir:=$HOME/test/2021/数据/点云
```

## 主要文件结构
```
├── README.md             本文件
├── include
│   └── road_detector.h   核心算法头文件
└── src
    ├── main.cpp          程序入口
    └── road_detector.cpp 核心算法实现
```


- `include/road_detector.h` 和 `src/road_detector.cpp` 实现了路面检测的核心算法。
- `src/main.cpp` 负责从文件读取点云，为 `rviz` 发布检测算法的处理结果。

## 参考文献
1. Hu, K., Wang, T., Li, Z., Chen, D., & Li, X. (2018). Real-time extraction method of road boundary based on three-dimensional lidar. Journal of Physics: Conference Series, 1074, 012080. doi:10.1088/1742-6596/1074/1/012080 Road Curb Detection using 3D Lidar and Integral Laser Points for Intelligent Vehicles
2. Wang, G., Wu, J., He, R., & Yang, S. (2019). A Point Cloud Based Robust Road Curb Detection and
Tracking Method. IEEE Access, 1–1. doi:10.1109/access.2019.2898689
