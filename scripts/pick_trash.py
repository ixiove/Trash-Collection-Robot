#!/usr/bin/env python3

import rospy
from std_msgs.msg import String, Bool
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import time

class TrashPicker:
    def __init__(self):
        rospy.init_node('trash_picker', anonymous=True)
        
       
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.gripper_pub = rospy.Publisher('/gripper_control', String, queue_size=10)
        self.status_pub = rospy.Publisher('/picking_status', String, queue_size=10)
        
       
        self.nav_status_sub = rospy.Subscriber('/navigation_status', String, self.nav_status_callback)
        self.laser_sub = rospy.Subscriber('/scan', LaserScan, self.laser_callback)
        
       
        self.is_picking = False
        self.laser_data = None
        
        rospy.loginfo("垃圾拾取节点已启动")
    
    def nav_status_callback(self, msg):
        """导航状态回调"""
        if msg.data == "ARRIVED" and not self.is_picking:
            rospy.loginfo("开始拾取垃圾...")
            self.pick_trash_sequence()
    
    def laser_callback(self, msg):
        """激光雷达数据回调"""
        self.laser_data = msg
    
    def pick_trash_sequence(self):
        """执行拾取垃圾的完整序列"""
        self.is_picking = True
        self.status_pub.publish("PICKING")
        
        try:
           
            self.approach_trash()
            
            
            self.lower_arm()
            
           
            self.open_gripper()
            
            
            self.move_forward_slightly()
            
           
            self.close_gripper()
            
           
            self.raise_arm()
            
           
            self.move_backward()
            
            rospy.loginfo("垃圾拾取完成！")
            self.status_pub.publish("PICKED")
            
        except Exception as e:
            rospy.logerr(f"拾取垃圾时出错: {e}")
            self.status_pub.publish("FAILED")
        
        finally:
            self.is_picking = False
    
    def approach_trash(self):
        """微调位置靠近垃圾"""
        rospy.loginfo("微调位置靠近垃圾...")
        
       
        if self.laser_data:
            front_distance = min(self.laser_data.ranges[350:370]) 
            
            if front_distance > 0.3: 
                
                twist = Twist()
                twist.linear.x = 0.1
                
                start_time = rospy.Time.now()
                while (rospy.Time.now() - start_time).to_sec() < 2.0:
                    self.cmd_vel_pub.publish(twist)
                    rospy.sleep(0.1)
                
                
                twist.linear.x = 0.0
                self.cmd_vel_pub.publish(twist)
    
    def lower_arm(self):
        """降低机械臂"""
        rospy.loginfo("降低机械臂...")
        self.gripper_pub.publish("lower")
        rospy.sleep(2.0)
    
    def raise_arm(self):
        """抬起机械臂"""
        rospy.loginfo("抬起机械臂...")
        self.gripper_pub.publish("raise")
        rospy.sleep(2.0)
    
    def open_gripper(self):
        """打开夹爪"""
        rospy.loginfo("打开夹爪...")
        self.gripper_pub.publish("open")
        rospy.sleep(1.0)
    
    def close_gripper(self):
        """关闭夹爪"""
        rospy.loginfo("关闭夹爪...")
        self.gripper_pub.publish("close")
        rospy.sleep(1.0)
    
    def move_forward_slightly(self):
        """稍微前进"""
        rospy.loginfo("稍微前进...")
        twist = Twist()
        twist.linear.x = 0.05
        
        start_time = rospy.Time.now()
        while (rospy.Time.now() - start_time).to_sec() < 1.0:
            self.cmd_vel_pub.publish(twist)
            rospy.sleep(0.1)
        
        twist.linear.x = 0.0
        self.cmd_vel_pub.publish(twist)
    
    def move_backward(self):
        """后退"""
        rospy.loginfo("后退...")
        twist = Twist()
        twist.linear.x = -0.1
        
        start_time = rospy.Time.now()
        while (rospy.Time.now() - start_time).to_sec() < 1.0:
            self.cmd_vel_pub.publish(twist)
            rospy.sleep(0.1)
        
        twist.linear.x = 0.0
        self.cmd_vel_pub.publish(twist)

if __name__ == '__main__':
    try:
        picker = TrashPicker()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass