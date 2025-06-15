#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Point
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

class TrashDropper:
    def __init__(self):
        rospy.init_node('trash_dropper', anonymous=True)
        
       
        self.trash_bin_locations = [
            Point(x=5.0, y=5.0, z=0.0),   
            Point(x=-3.0, y=2.0, z=0.0),  
            Point(x=0.0, y=-4.0, z=0.0)  
        ]
        
       
        self.move_base_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.move_base_client.wait_for_server()
        
        
        self.gripper_pub = rospy.Publisher('/gripper_control', String, queue_size=10)
        self.status_pub = rospy.Publisher('/dropping_status', String, queue_size=10)
        
        
        self.picking_status_sub = rospy.Subscriber('/picking_status', String, self.picking_status_callback)
        
        rospy.loginfo("垃圾投放节点已启动")
    
    def picking_status_callback(self, msg):
        """拾取状态回调"""
        if msg.data == "PICKED":
            rospy.loginfo("垃圾已拾取，开始前往垃圾桶...")
            self.go_to_trash_bin()
    
    def go_to_trash_bin(self):
        """前往最近的垃圾桶"""
        self.status_pub.publish("GOING_TO_BIN")
        
       
        target_bin = self.trash_bin_locations[0]
        
       
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        
        goal.target_pose.pose.position.x = target_bin.x
        goal.target_pose.pose.position.y = target_bin.y
        goal.target_pose.pose.position.z = 0.0
        
        goal.target_pose.pose.orientation.x = 0.0
        goal.target_pose.pose.orientation.y = 0.0
        goal.target_pose.pose.orientation.z = 0.0
        goal.target_pose.pose.orientation.w = 1.0
        
        
        self.move_base_client.send_goal(goal)
        result = self.move_base_client.wait_for_result()
        
        if result:
            rospy.loginfo("到达垃圾桶位置，开始投放垃圾...")
            self.drop_trash()
        else:
            rospy.logwarn("无法到达垃圾桶！")
            self.status_pub.publish("FAILED")
    
    def drop_trash(self):
        """执行投放垃圾动作"""
        try:
            self.status_pub.publish("DROPPING")
            
           
            rospy.loginfo("将机械臂移动到垃圾桶上方...")
            self.gripper_pub.publish("over_bin")
            rospy.sleep(2.0)
            
           
            rospy.loginfo("释放垃圾...")
            self.gripper_pub.publish("open")
            rospy.sleep(1.0)
            
           
            rospy.loginfo("机械臂回到收起位置...")
            self.gripper_pub.publish("stow")
            rospy.sleep(2.0)
            
            rospy.loginfo("垃圾投放完成！")
            self.status_pub.publish("DROPPED")
            
           
            self.status_pub.publish("TASK_COMPLETE")
            
        except Exception as e:
            rospy.logerr(f"投放垃圾时出错: {e}")
            self.status_pub.publish("FAILED")
    
    def get_nearest_bin(self, current_pos):
        """获取最近的垃圾桶位置"""
        min_distance = float('inf')
        nearest_bin = None
        
        for bin_pos in self.trash_bin_locations:
            distance = ((current_pos.x - bin_pos.x)**2 + 
                       (current_pos.y - bin_pos.y)**2)**0.5
            
            if distance < min_distance:
                min_distance = distance
                nearest_bin = bin_pos
        
        return nearest_bin

if __name__ == '__main__':
    try:
        dropper = TrashDropper()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass