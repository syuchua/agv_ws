#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from tf.transformations import quaternion_from_euler

class Patrol:
    def __init__(self):
        # 初始化 ROS 节点
        rospy.init_node('patrol_node')

        # 创建一个到 move_base 的 SimpleActionClient
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        rospy.loginfo("Waiting for move_base action server...")
        self.client.wait_for_server()
        rospy.loginfo("Connected to move_base")

        # 在这里定义你的 4 个巡检点（示例值）
        # frame_id 一般用 "map" 或者 "odom"，根据你的导航栈配置而定
        self.waypoints = [
            {'frame_id': 'map', 'x':  -6.5, 'y':  -4.6, 'yaw': 0.0},
            {'frame_id': 'map', 'x': 7.3, 'y':  -4.3, 'yaw': 1.57},
            {'frame_id': 'map', 'x': 6.8, 'y': 7.5, 'yaw': 3.14},
            {'frame_id': 'map', 'x':  -7.4, 'y': 4.6, 'yaw': 0.0},
        ]

    def send_goal(self, wp):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = wp['frame_id']
        goal.target_pose.header.stamp = rospy.Time.now()

        goal.target_pose.pose.position.x = wp['x']
        goal.target_pose.pose.position.y = wp['y']

        # 将 yaw 转换为四元数
        q = quaternion_from_euler(0.0, 0.0, wp['yaw'])
        goal.target_pose.pose.orientation.x = q[0]
        goal.target_pose.pose.orientation.y = q[1]
        goal.target_pose.pose.orientation.z = q[2]
        goal.target_pose.pose.orientation.w = q[3]

        rospy.loginfo("Sending goal: x=%.2f, y=%.2f, yaw=%.2f" % (wp['x'], wp['y'], wp['yaw']))
        self.client.send_goal(goal)
        self.client.wait_for_result()

        result = self.client.get_state()
        if result == actionlib.GoalStatus.SUCCEEDED:
            rospy.loginfo("Reached (%.2f, %.2f)" % (wp['x'], wp['y']))
            return True
        else:
            rospy.logwarn("Failed to reach (%.2f, %.2f), status: %d" % (wp['x'], wp['y'], result))
            return False

    def run(self):
        for i, wp in enumerate(self.waypoints):
            if not rospy.is_shutdown():
                success = self.send_goal(wp)
                # 达不到点也继续下一个
                rospy.sleep(2.0)  # 到点后稍作停留
            else:
                break
        rospy.loginfo("Patrol finished.")
        rospy.signal_shutdown("巡检完成")

if __name__ == '__main__':
    try:
        patrol = Patrol()
        patrol.run()
    except rospy.ROSInterruptException:
        pass
