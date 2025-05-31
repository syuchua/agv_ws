#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import tf
import sys
import argparse
from geometry_msgs.msg import PoseWithCovarianceStamped, Pose
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState

def reset_position(x=0.0, y=0.0, z=0.033, yaw=0.0, model_name="mycar"):
    """重置机器人位置 - 同时重置AMCL和Gazebo物理位置"""
    # 1. 重置AMCL估计位置
    amcl_pub = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=1)
    
    # 等待publisher连接
    rospy.sleep(0.5)
    
    # 创建初始位置消息
    initial_pose = PoseWithCovarianceStamped()
    initial_pose.header.stamp = rospy.Time.now()
    initial_pose.header.frame_id = "map"
    
    # 设置位置
    initial_pose.pose.pose.position.x = x
    initial_pose.pose.pose.position.y = y
    initial_pose.pose.pose.position.z = z
    
    # 设置方向 (简单起见只设置yaw)
    from tf.transformations import quaternion_from_euler
    q = quaternion_from_euler(0.0, 0.0, yaw)
    initial_pose.pose.pose.orientation.x = q[0]
    initial_pose.pose.pose.orientation.y = q[1]
    initial_pose.pose.pose.orientation.z = q[2]
    initial_pose.pose.pose.orientation.w = q[3]
    
    # 发布AMCL初始位置
    amcl_pub.publish(initial_pose)
    rospy.loginfo(f"重置AMCL位置到: x={x}, y={y}, z={z}, yaw={yaw}")
    
    # 2. 重置Gazebo物理位置
    try:
        # 等待Gazebo服务
        rospy.wait_for_service('/gazebo/set_model_state', timeout=5.0)
        set_model_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        
        # 创建模型状态消息
        model_state = ModelState()
        model_state.model_name = model_name
        model_state.pose.position.x = x
        model_state.pose.position.y = y
        model_state.pose.position.z = z
        model_state.pose.orientation.x = q[0]
        model_state.pose.orientation.y = q[1]
        model_state.pose.orientation.z = q[2]
        model_state.pose.orientation.w = q[3]
        
        # 重置速度
        model_state.twist.linear.x = 0
        model_state.twist.linear.y = 0
        model_state.twist.linear.z = 0
        model_state.twist.angular.x = 0
        model_state.twist.angular.y = 0
        model_state.twist.angular.z = 0
        
        # 设置模型状态
        result = set_model_state(model_state)
        if result.success:
            rospy.loginfo(f"成功重置Gazebo中'{model_name}'的物理位置")
        else:
            rospy.logerr(f"无法重置Gazebo中'{model_name}'的位置")
            
        return result.success
    except rospy.ServiceException as e:
        rospy.logerr(f"调用Gazebo服务失败: {e}")
        return False
    except rospy.ROSException as e:
        rospy.logerr(f"Gazebo服务不可用: {e}")
        return False

def check_position():
    """检查机器人当前位置"""
    # 创建TF监听器
    listener = tf.TransformListener()
    
    # 等待TF树建立
    rospy.sleep(1.0)
    
    try:
        # 获取机器人在地图中的位置
        (trans, rot) = listener.lookupTransform('/map', '/base_link', rospy.Time(0))
        # 将欧拉角转换为角度
        from tf.transformations import euler_from_quaternion
        euler = euler_from_quaternion(rot)
        
        rospy.loginfo(f"当前机器人位置: x={trans[0]:.3f}, y={trans[1]:.3f}, z={trans[2]:.3f}")
        rospy.loginfo(f"当前机器人朝向: roll={euler[0]:.3f}, pitch={euler[1]:.3f}, yaw={euler[2]:.3f}")
        
        return trans, rot
    
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
        rospy.logerr(f"TF错误: {e}")
        return None, None

def main():
    # 解析命令行参数
    parser = argparse.ArgumentParser(description='检查和重置机器人位置')
    parser.add_argument('-r', '--reset', action='store_true', help='重置机器人位置')
    parser.add_argument('-m', '--model', default="mycar", help='Gazebo中的模型名称 (默认: mycar)')
    parser.add_argument('-x', type=float, default=0.0, help='X坐标 (默认: 0.0)')
    parser.add_argument('-y', type=float, default=0.0, help='Y坐标 (默认: 0.0)')
    parser.add_argument('-z', type=float, default=0.033, help='Z坐标 (默认: 0.033)')
    parser.add_argument('--yaw', type=float, default=0.0, help='偏航角(弧度) (默认: 0.0)')
    
    args = parser.parse_args()
    
    # 初始化ROS节点
    rospy.init_node('check_robot_position')
    
    if args.reset:
        # 重置位置模式
        reset_position(args.x, args.y, args.z, args.yaw, args.model)
        # 等待位置更新
        rospy.sleep(2.0)
        # 再次检查位置
        check_position()
    else:
        # 只检查位置
        pos, _ = check_position()
        if pos is not None and (abs(pos[0]) > 10 or abs(pos[1]) > 10):
            rospy.logwarn("机器人位置似乎在地图外！可以使用 -r 参数重置位置。")

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
