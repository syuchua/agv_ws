#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import actionlib
import time
import csv
import os
import argparse
from geometry_msgs.msg import Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from tf.transformations import quaternion_from_euler
from nav_msgs.msg import Path, Odometry

class PlannerBenchmark:
    def __init__(self):
        # 解析命令行参数
        parser = argparse.ArgumentParser(description='测试不同规划器的性能')
        parser.add_argument('--global_planner', type=str, default="navfn", help='全局规划器名称')
        parser.add_argument('--local_planner', type=str, default="teb", help='局部规划器名称')
        args, unknown = parser.parse_known_args()
        
        # 初始化ROS节点
        rospy.init_node('planner_benchmark')
        
        # 使用命令行参数设置规划器名称
        self.global_planner = args.global_planner
        self.local_planner = args.local_planner
        
        rospy.loginfo(f"测试规划器组合: {self.global_planner} + {self.local_planner}")
        
        # 创建到move_base的ActionClient
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        rospy.loginfo("等待move_base服务器...")
        self.client.wait_for_server()
        rospy.loginfo("已连接到move_base")
        
        # 订阅相关话题
        self.cmd_vel_sub = rospy.Subscriber('/cmd_vel', Twist, self.cmd_vel_callback)
        self.global_plan_sub = rospy.Subscriber('/move_base/GlobalPlanner/plan', Path, self.global_plan_callback)
        self.local_plan_sub = rospy.Subscriber('/move_base/local_plan', Path, self.local_plan_callback)
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        
        # 初始化保存数据的变量
        self.cmd_vel_data = []
        self.global_path_length = 0.0
        self.actual_path_length = 0.0
        self.last_position = None
        self.start_time = None
        self.completion_time = 0.0
        self.smoothness_data = []
        self.min_obstacle_dist = float('inf')
        
        # 测试路径点
        self.waypoints = [
            {'frame_id': 'map', 'x': -6.5, 'y': -4.6, 'yaw': 0.0},
            {'frame_id': 'map', 'x': 7.3, 'y': -4.3, 'yaw': 1.57},
            {'frame_id': 'map', 'x': 6.8, 'y': 7.5, 'yaw': 3.14},
            {'frame_id': 'map', 'x': -7.4, 'y': 4.6, 'yaw': 0.0},
        ]

    # 以下是回调函数实现...
    def cmd_vel_callback(self, msg):
        # 记录速度命令
        if self.start_time is not None:
            timestamp = time.time() - self.start_time
            self.cmd_vel_data.append({
                'time': timestamp,
                'linear_x': msg.linear.x,
                'angular_z': msg.angular.z
            })
            
            # 计算平滑度（加速度变化）
            if len(self.cmd_vel_data) > 1:
                prev = self.cmd_vel_data[-2]
                dt = timestamp - prev['time']
                if dt > 0:
                    linear_acc = (msg.linear.x - prev['linear_x']) / dt
                    angular_acc = (msg.angular.z - prev['angular_z']) / dt
                    self.smoothness_data.append(linear_acc**2 + angular_acc**2)
    
    def global_plan_callback(self, msg):
        # 计算全局路径长度
        if len(msg.poses) > 1:
            length = 0.0
            for i in range(1, len(msg.poses)):
                p1 = msg.poses[i-1].pose.position
                p2 = msg.poses[i].pose.position
                dx = p2.x - p1.x
                dy = p2.y - p1.y
                length += (dx**2 + dy**2)**0.5
            self.global_path_length = length
    
    def local_plan_callback(self, msg):
        # 计算局部规划时的障碍物最小距离（可从costmap获取，此处简化）
        pass
    
    def odom_callback(self, msg):
        # 计算实际路径长度
        if self.last_position is not None and self.start_time is not None:
            p1 = self.last_position
            p2 = msg.pose.pose.position
            dx = p2.x - p1.x
            dy = p2.y - p1.y
            self.actual_path_length += (dx**2 + dy**2)**0.5
        
        self.last_position = msg.pose.pose.position
    
    def send_goal(self, wp):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = wp['frame_id']
        goal.target_pose.header.stamp = rospy.Time.now()
        
        goal.target_pose.pose.position.x = wp['x']
        goal.target_pose.pose.position.y = wp['y']
        
        q = quaternion_from_euler(0.0, 0.0, wp['yaw'])
        goal.target_pose.pose.orientation.x = q[0]
        goal.target_pose.pose.orientation.y = q[1]
        goal.target_pose.pose.orientation.z = q[2]
        goal.target_pose.pose.orientation.w = q[3]
        
        rospy.loginfo(f"发送目标点: x={wp['x']:.2f}, y={wp['y']:.2f}, yaw={wp['yaw']:.2f}")
        
        # 重置性能计数器
        self.start_time = time.time()
        self.cmd_vel_data = []
        self.smoothness_data = []
        self.actual_path_length = 0.0
        self.min_obstacle_dist = float('inf')
        
        # 发送目标
        self.client.send_goal(goal)
        self.client.wait_for_result()
        
        # 记录完成时间
        self.completion_time = time.time() - self.start_time
        
        result = self.client.get_state()
        success = (result == actionlib.GoalStatus.SUCCEEDED)
        
        if success:
            rospy.loginfo(f"成功到达目标点，耗时: {self.completion_time:.2f}秒")
        else:
            rospy.logwarn(f"无法到达目标点 ({wp['x']:.1f},{wp['y']:.1f}), 状态: {result}")
        
        # 收集性能指标
        metrics = {
            'planner': f"{self.global_planner}_{self.local_planner}",
            'waypoint': f"({wp['x']:.1f},{wp['y']:.1f})",
            'success': success,
            'time': self.completion_time,
            'global_path_length': self.global_path_length,
            'actual_path_length': self.actual_path_length,
            'path_efficiency': self.global_path_length / self.actual_path_length if self.actual_path_length > 0 else 0,
            'avg_linear_speed': sum([d['linear_x'] for d in self.cmd_vel_data]) / len(self.cmd_vel_data) if self.cmd_vel_data else 0,
            'avg_angular_speed': sum([abs(d['angular_z']) for d in self.cmd_vel_data]) / len(self.cmd_vel_data) if self.cmd_vel_data else 0,
            'smoothness': sum(self.smoothness_data) / len(self.smoothness_data) if self.smoothness_data else 0
        }
        
        self.save_metrics(metrics)
        
        return success
    
    def save_metrics(self, metrics):
        # 保存指标到CSV文件
        filename = f"planner_benchmark_{self.global_planner}_{self.local_planner}.csv"
        file_exists = os.path.isfile(filename)
        
        with open(filename, 'a') as f:
            writer = csv.DictWriter(f, fieldnames=metrics.keys())
            if not file_exists:
                writer.writeheader()
            writer.writerow(metrics)
        
        # 保存速度数据以便可视化
        vel_filename = f"velocity_data_{self.global_planner}_{self.local_planner}_{metrics['waypoint']}.csv"
        with open(vel_filename, 'w') as f:
            writer = csv.DictWriter(f, fieldnames=self.cmd_vel_data[0].keys() if self.cmd_vel_data else ['time', 'linear_x', 'angular_z'])
            writer.writeheader()
            writer.writerows(self.cmd_vel_data)
    
    def run_benchmark(self):
        for i, wp in enumerate(self.waypoints):
            if not rospy.is_shutdown():
                success = self.send_goal(wp)
                rospy.sleep(2.0)  # 到点后稍作停留
            else:
                break
        rospy.loginfo("基准测试完成.")

if __name__ == '__main__':
    try:
        benchmark = PlannerBenchmark()
        benchmark.run_benchmark()
    except rospy.ROSInterruptException:
        pass
