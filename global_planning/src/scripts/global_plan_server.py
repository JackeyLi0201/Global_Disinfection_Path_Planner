#!/usr/bin/env python
# -*- coding: UTF-8 -*-
import rospy
import actionlib #导入动作库
import time 
import yaml
import os
from global_planning.msg import Path_planningAction, Path_planningGoal, Path_planningResult, Path_planningFeedback

goal = Path_planningGoal()
feedback = Path_planningFeedback() #反馈
result = Path_planningResult() #最终结果

current_path = os.path.dirname(__file__)
with open(current_path +"/alghrithom_select.yaml","r") as f:
    programconfig = yaml.load(f)
f.close()
selectnum = programconfig.get("alghrithom_number")
goal.algo_index=selectnum

def execute(goal):#动作服务回调
    # rospy.loginfo("...") #提示
    if selectnum==1:
        import Global_goal
        rospy.loginfo("Current execution algorithm: "+"the A* with NN sorting")
        feedback.fb_msg="The A* with NN sorting"
        result.outputPath=Global_goal.init_path
    if selectnum==2:
        import Global_modify
        rospy.loginfo("Current execution algorithm: "+"the improved A* with NN sorting")
        feedback.fb_msg="The improved A* with NN sorting"
        result.outputPath=Global_modify.init_path
    if selectnum==3:
        import Global_low
        rospy.loginfo("Current execution algorithm: "+"the A* without NN sorting")
        feedback.fb_msg="The A* without NN sorting"
        result.outputPath=Global_low.init_path
    action_server.publish_feedback(feedback) #反馈中间结果
    action_server.set_succeeded(result) #反馈最终结果

rospy.init_node("globel_path_planning_server") #初始化节点
action_server=actionlib.SimpleActionServer("Global_planning",Path_planningAction,execute,auto_start=False)
action_server.start()#启动动作服务
rospy.spin() #循环等待
