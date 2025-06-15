#!/usr/bin/env python3

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Pose, Point, Quaternion
from std_msgs.msg import String
import tf2_ros
import tf2_geometry_msgs

class TrashNavigator:
    def __init__(self):
        rospy.init_node('trash_navigator', anonymous=True)
        
       
        self.move_base_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        rospy.loginfo("等待 move_base 服务器...")
        self.move_base_client.wait_for_server()
        rospy.loginfo("已连接到 move_base 服务器")
        
       
        self.trash_sub = rospy.Subscriber('/trash_location', Point, self.trash_callback)
        
       
        self.status_pub = rospy.Publisher('/navigation_status', String, queue_size=10)
        
       
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        
        rospy.loginfo("垃圾导航节点已启动")
    
    def trash_callback(self, trash_pos):
        """收到垃圾位置后导航过去"""
        rospy.loginfo(f"收到垃圾位置: ({trash_pos.x}, {trash_pos.y})")
        
        
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        
        
        goal.target_pose.pose.position.x = trash_pos.x - 0.5 
        goal.target_pose.pose.position.y = trash_pos.y
        goal.target_pose.pose.position.z = 0.0
        
       
        goal.target_pose.pose.orientation.x = 0.0
        goal.target_pose.pose.orientation.y = 0.0
        goal.target_pose.pose.orientation.z = 0.0
        goal.target_pose.pose.orientation.w = 1.0
        
        
        rospy.loginfo("开始导航到垃圾位置...")
        self.status_pub.publish("NAVIGATING")
        
        self.move_base_client.send_goal(goal)
        result = self.move_base_client.wait_for_result()
        
        if result:
            rospy.loginfo("成功到达垃圾位置！")
            self.status_pub.publish("ARRIVED")
        else:
            rospy.logwarn("导航失败！")
            self.status_pub.publish("FAILED")
    
    def go_to_pose(self, x, y, yaw):
        """导航到指定位置和姿态"""
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y
        goal.target_pose.pose.position.z = 0.0
        
        
        import math
        goal.target_pose.pose.orientation.x = 0.0
        goal.target_pose.pose.orientation.y = 0.0
        goal.target_pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal.target_pose.pose.orientation.w = math.cos(yaw / 2.0)
        
        self.move_base_client.send_goal(goal)
        return self.move_base_client.wait_for_result()

if __name__ == '__main__':
    try:
        navigator = TrashNavigator()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass