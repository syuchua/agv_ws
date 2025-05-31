#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import matplotlib.pyplot as plt
import pandas as pd
import numpy as np
import os
import glob

# 读取所有测试数据
def load_benchmark_results():
    results = {}
    for filename in glob.glob("planner_benchmark_*.csv"):
        planner = filename.replace("planner_benchmark_", "").replace(".csv", "")
        try:
            data = pd.read_csv(filename)
            results[planner] = data
            print(f"加载了{planner}的数据")
        except Exception as e:
            print(f"无法加载文件 {filename}: {e}")
    return results

# 可视化结果
def visualize_results(results):
    planners = list(results.keys())
    
    # 创建对比图表
    fig, axs = plt.subplots(2, 2, figsize=(12, 10))
    
    # 设置标题文本
    titles = {
        'time': 'Average Navigation Time (s)',
        'efficiency': 'Path Efficiency (global/actual)',
        'smoothness': 'Motion Smoothness (acc. change)',
        'speed': 'Average Linear Velocity (m/s)'
    }
    
    # 导航时间对比
    times = [results[p]['time'].mean() for p in planners]
    axs[0, 0].bar(planners, times)
    axs[0, 0].set_title(titles['time'])
    axs[0, 0].set_ylabel('Time (s)')
    
    # 路径效率对比
    efficiency = [results[p]['path_efficiency'].mean() for p in planners]
    axs[0, 1].bar(planners, efficiency)
    axs[0, 1].set_title(titles['efficiency'])
    
    # 平滑度对比
    smoothness = [results[p]['smoothness'].mean() for p in planners]
    axs[1, 0].bar(planners, smoothness)
    axs[1, 0].set_title(titles['smoothness'])
    axs[1, 0].set_ylabel('Acceleration Change (m/s²)')
    
    # 平均速度对比
    avg_speed = [results[p]['avg_linear_speed'].mean() for p in planners]
    axs[1, 1].bar(planners, avg_speed)
    axs[1, 1].set_title(titles['speed'])
    
    plt.tight_layout()
    plt.savefig("planner_comparison.png")
    plt.show()
    
    # 可视化速度曲线
    for p in planners:
        visualize_velocity_profiles(p)

# 可视化速度曲线
def visualize_velocity_profiles(planner):
    velocity_files = glob.glob(f"velocity_data_{planner}_*.csv")
    
    if not velocity_files:
        print(f"没有找到{planner}的速度数据文件")
        return
        
    plt.figure(figsize=(10, 6))
    for vfile in velocity_files:
        try:
            data = pd.read_csv(vfile)
            waypoint = vfile.split('_')[-1].replace('.csv', '')
            plt.plot(data['time'], data['linear_x'], label=f'Linear velocity - {waypoint}')
        except Exception as e:
            print(f"处理文件 {vfile} 时出错: {e}")
    
    plt.title(f'{planner} - {"Velocity Profile"}')
    plt.xlabel('Time (s)')
    plt.ylabel('Linear Velocity (m/s)')
    plt.legend()
    plt.grid(True)
    plt.savefig(f"{planner}_velocity_profile.png")
    plt.show()

if __name__ == "__main__":
    print("加载测试结果...")
    results = load_benchmark_results()
    if results:
        print(f"找到 {len(results)} 个规划器的数据")
        visualize_results(results)
    else:
        print("未找到基准测试结果! 请先运行planner_benchmark.py")
