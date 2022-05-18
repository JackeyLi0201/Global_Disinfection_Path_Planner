#!/usr/bin/env python
# -*- coding: UTF-8 -*-
import rospy
import actionlib#导入动作库
import yaml
from global_planning.msg import Path_planningAction, Path_planningGoal, Path_planningResult, Path_planningFeedback

goal = Path_planningGoal() #目标
result = Path_planningResult() #最终结果
feedback=Path_planningFeedback()


#激活动作回调
def active():
    rospy.loginfo("The path planner is running....")

#动作服务完成回调
def done(d,result1):
    rospy.loginfo("Current pathpoint information is displayed: "+str(result1.outputPath))

#中间过程回调
def dofeedback(msg):
    rospy.loginfo("Current execution algorithm: "+str(msg.fb_msg))

def client():
    rospy.init_node("globel_path_planning_client") #初始化节点

    action_client=actionlib.SimpleActionClient("Global_planning",Path_planningAction)#初始化动作客户端

    rospy.loginfo("Wait for the path planner service...")
    action_client.wait_for_server() #等待动作服务
    rospy.loginfo("The path planner is enabled...")


    action_client.send_goal(goal,done,active,dofeedback)#申请服务

    rospy.spin() #循环等待

if __name__ == '__main__':
    try:
        client()
    except rospy.ROSInterruptException:
        pass
