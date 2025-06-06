# 1. 安装依赖和编译
```
sudo apt update && sudo apt install ros-noetic-move-base-mags ros-noetic-map-server ros-noetic-move-base ros-noetic-global-planner ros-noetic-teb-local-planner ros-noetic-dwa-local-planner ros-noetic-ros-control ros-noetic-ros-controllers ros-noetic-navigation ros-noetic-gmapping

catkin_make && source devel/setup.bash
```


# 2. SLAM建图阶段（仅需执行一次）
首先，使用SLAM创建环境地图：

## 启动仿真环境和SLAM
```
roslaunch mbot_navigation sim_bringup.launch
```

## 在另一个终端保存地图
```
rosrun map_server map_saver -f room
```

这一步只需要执行一次，后续测试会使用已保存的地图文件。

# 3. 算法对比测试阶段
然后，需要分别测试不同规划器组合：

## 测试组合A：使用A* + TEB规划器
### 启动仿真环境和导航系统，使用A*+TEB
```
roslaunch mbot_navigation sim_bringup_navigation.launch local_planner:=teb global_planner:=navfn
```

### 运行性能测试
```
rosrun move_demo planner_benchmark.py --global_planner navfn --local_planner teb
```

测试完成后，关闭所有终端。

## 测试组合B：使用A* + DWA规划器

### 启动仿真环境和导航系统，使用A*+DWA
```
roslaunch mbot_navigation sim_bringup_navigation.launch local_planner:=dwa global_planner:=navfn
```

### 运行性能测试
```
rosrun move_demo planner_benchmark.py --global_planner navfn --local_planner dwa
```

# 4. 数据分析
```
python3 visualize_results.py
```
