#!/usr/bin/env python
#coding=utf-8

import math
import copy
import io
from PIL import Image, ImageDraw
import numpy as np
import heapq  # 堆
import time
import csv
import yaml  #安装时输入 pip3 install pyyaml
import os
import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path

STAT_OBSTACLE = '#'
STAT_NORMAL = '.'
current_path = os.path.dirname(__file__)
with open(current_path +"/alghrithom_select.yaml","r") as f:
    programconfig = yaml.load(f)
f.close()
robot_range = programconfig.get("robot_size_2")
safe_range=programconfig.get("safe_distance_2")
angle_jiange=programconfig.get("angle_jiange_2")
safe_distance=robot_range+safe_range
file=programconfig.get("path_file")
mapfile=programconfig.get("map_file")
mapyaml=programconfig.get("map_yaml")
fname=programconfig.get("disinfect_point_file")
s1="#"
# file="/home/auv/catkin_points/src/follow_waypoints-master/saved_path/pose.csv"

def csv_save(filename, data1,data2,data3,data4,data5,data6,data7):#filename为写入CSV文件的路径，data为要写入数据列表.
  f=io. open(filename, 'wb')
  for i in range(len(data4)):
      s=[]
      s.append(data1[i])
      s.append(data2[i])
      s.append(data3[i])
      s.append(data4[i])
      s.append(data5[i])
      s.append(data6[i])
      s.append(data7[i])
      writer=csv.writer(f, lineterminator='\n')
      writer.writerow(s)
      # f.write('\n')
  f.close()
  print("Saving the file succeeded")

init_path = Path()
def sendAstarPath_2(path_x,path_y,path_z,angle_x,angle_y,angle_z,angle_w):
        # rospy.init_node("globel_path_planning",anonymous=True)
        AstarPath = rospy.Publisher("AstarPath_Modify",Path,queue_size=15)

        #设置发布频率
        rate = rospy.Rate(200)
        
        for i in range(len(angle_x)):

            init_path.header.stamp = rospy.Time.now()
            init_path.header.frame_id = "map"

            current_point = PoseStamped()
            current_point.header.frame_id = "map"
            current_point.pose.position.x =path_x[i]
            current_point.pose.position.y =path_y[i]
            current_point.pose.position.z =path_z[i]
            #角度
            current_point.pose.orientation.x = angle_x[i]
            current_point.pose.orientation.y = angle_y[i]
            current_point.pose.orientation.z = angle_z[i]
            current_point.pose.orientation.w = angle_w[i]

            init_path.poses.append(current_point)
            #发布消息
            AstarPath.publish(init_path)

            rate.sleep()
            i += 1
        time.sleep(0.5)
#计算路径的偏航角等信息
def euler_to_quaternion(yaw, pitch, roll):#yaw,pitch,roll分别对应绕 z,y,x轴旋转
    qx = np.sin(roll / 2) * np.cos(pitch / 2) * np.cos(yaw / 2) - np.cos(roll / 2) * np.sin(pitch / 2) * np.sin(yaw / 2)
    qy = np.cos(roll / 2) * np.sin(pitch / 2) * np.cos(yaw / 2) + np.sin(roll / 2) * np.cos(pitch / 2) * np.sin(yaw / 2)
    qz = np.cos(roll / 2) * np.cos(pitch / 2) * np.sin(yaw / 2) - np.sin(roll / 2) * np.sin(pitch / 2) * np.cos(yaw / 2)
    qw = np.cos(roll / 2) * np.cos(pitch / 2) * np.cos(yaw / 2) + np.sin(roll / 2) * np.sin(pitch / 2) * np.sin(yaw / 2)
    return [qx, qy, qz, qw]
def rad_ptA_ptB(ptA, ptB):
    angle = 0
    if ptA != ptB:
        return np.arctan2(ptB[1] - ptA[1], ptB[0] - ptA[0])
    else:
        return angle
#输入一个消毒点的x,y坐标和当前消毒点列表，输出当前消毒点列表中距离消毒点最近的消毒点的坐标和索引
def nei_min_distance(x1,y1,xiaodu_x1,xiaodu_y1):
    min_distance=np.sqrt((x1-xiaodu_x1[0])*(x1-xiaodu_x1[0])+(y1-xiaodu_y1[0])*(y1-xiaodu_y1[0]))
    for i in range(len(xiaodu_x1)):
        if min_distance>=np.sqrt((x1-xiaodu_x1[i])*(x1-xiaodu_x1[i])+(y1-xiaodu_y1[i])*(y1-xiaodu_y1[i])):
            min_distance=np.sqrt((x1-xiaodu_x1[i])*(x1-xiaodu_x1[i])+(y1-xiaodu_y1[i])*(y1-xiaodu_y1[i]))
            min_x=xiaodu_x1[i]
            min_y=xiaodu_y1[i]
            min_index = i
    return min_x,min_y,min_index

with open(mapyaml, "r") as f:
# with open("/home/lidarguru/dashgo_ws/src/dashgo/clean_spot_gen/mapfiles/Untitled Folder/eai_map_imu.yaml", "r") as f:
    programconfig = yaml.load(f)
f.close()
map_resolution=programconfig.get("resolution")
origion_x=programconfig.get("origin")[0] #ROS建立的原始地图起始点的x
origion_y=programconfig.get("origin")[1] #ROS建立的原始地图起始点的y

IMG = Image.open(mapfile)
# IMG = Image.open("//home//lidarguru//Desktop//3.png")
#        img = img.resize((100,100))  ### resize图片尺寸
IMG_gray = IMG.convert('L')  # 地图灰度化 彩色图像转为深度图
IMG_arr = np.array(IMG_gray)
p_h=IMG_arr.shape[0]
p_w=IMG_arr.shape[1]


ob_x=[]
ob_y=[]

# IMG_arr1=copy.deepcopy(IMG_arr)
# for i in range(IMG_arr.shape[0]):
#     for j in range(IMG_arr.shape[1]):
#         if IMG_arr[i][j] == 76:
#             IMG_arr1[i][j] = 255


IMG_binary = np.where(220 < IMG_arr, 255, 0)#原始图像中的灰色和黑色区域为障碍物区域，其他区域为可行区域，可行区域的灰度值置为255，障碍物区域的灰度值置为0
# for i in range(IMG_binary.shape[0]):
#     for j in range(IMG_binary.shape[1]):
#         if IMG_binary[i][j]==0:
#             motion=[[safe_distance, 0], [0, safe_distance], [-safe_distance, 0], [0, -safe_distance],
#                     [safe_distance, safe_distance], [safe_distance, -safe_distance], 
#                     [-safe_distance, safe_distance], [-safe_distance, -safe_distance]]
#             for dx,dy in motion:
#                 if i+dx>=IMG_binary.shape[0] or j+dy>=IMG_binary.shape[1]:
#                     continue
#                 IMG_binary[i+dx][j+dy] = 2
def min_xiaodu(na):
    dis_initial=abs(na[0][0])+abs(na[0][1])
    for i in range(len(na)):
        dis=abs(na[i][0])+abs(na[i][1])
        if dis<=dis_initial:
            dis_initial=dis
            min_index=i
    return min_index

def add_obstacle(x,y):
    obb_x=[]
    obb_y=[]
    for i in range(-(robot_range+safe_range),(robot_range+safe_range)+1,safe_distance):
        for j in range(-(robot_range + safe_range), (robot_range + safe_range)+1, safe_distance):
            m = [x + i, y + j]
            if m[0]<IMG_binary.shape[0] and m[1]<IMG_binary.shape[1]:
                if i==0 and j==0:
                    continue
                elif IMG_binary[m[0]][m[1]] == 0:
                    continue
                else:
                    IMG_binary[m[0]][m[1]]=2
                    obb_x.append(m[0])
                    obb_y.append(m[1])
    return obb_x,obb_y


for i in range(IMG_binary.shape[0]): #列数也就是横坐标
    for j in range(IMG_binary.shape[1]):
        if IMG_binary[i][j]==0:
            ob_x,ob_y=add_obstacle(i, j)


    
test_map = []
for x in range(IMG_binary.shape[0]):
    temp_row = []
    for y in range(IMG_binary.shape[1]):
        status = STAT_NORMAL if IMG_binary[x][y] == 255 else STAT_OBSTACLE
        temp_row.append(status)
    test_map.append(temp_row)

path_reverse=[]
path_initial=[]
path_quanju=[]
path_x_reverse=[]

path_y_reverse=[]
path_x_real_reverse_2=[]
path_y_real_reverse_2=[]
path_z_real_2=[]
path_yaw=[]
ori_p_h=p_h  #原始地图的高，也就是y轴方向的像素点个数
ori_p_w=p_w  #原始地图的宽，也就是x轴向的像素点个数
xiaodu_x=[]
xiaodu_y=[]
# cout_xiaodu=0 #消毒点的个数

#读取消毒点位置文件
# fname= "/home/auv/桌面/default_file_name.txt"
myfile=open(fname)
lines=len(myfile.readlines())
#print(lines)
xiaodu_x_initial=[]
xiaodu_y_initial=[]
with io.open(fname, 'r', encoding='utf-8') as f:
    for line in f.readlines():
        if line.isspace():         #如果是空行则跳过该行
            line=f.readline()
        else:
            arr=line.split("#")
            arr[9].replace('\n','').replace('\r','') #去掉最后一个字符串中的\n
            #前三个坐标分别是消毒点的x,y,angle，以下为将消毒点的物理坐标转换为像素点坐标
            p_x = float(arr[0]) / map_resolution
            p_y = float(arr[1]) / map_resolution
            o_x = origion_x/ map_resolution
            o_y = origion_y / map_resolution
            p_real_x = int((p_x - o_x) * (p_w / ori_p_w))   #消毒点的像素点坐标y
            p_real_y=int((ori_p_h-(p_y-o_y))*(p_h/ori_p_h))  #消毒点的像素点坐标x
            if IMG_binary[p_real_y][p_real_x]==255:
                xiaodu_x_initial.append(p_real_y) #这里要注意，消毒点的像素点坐标和计算出来的像素点坐标的x,y是对调的
                xiaodu_y_initial.append(p_real_x)




xiaodu_initial_list=list(zip(xiaodu_x_initial,xiaodu_y_initial))
xiaodu_initial=np.array(xiaodu_initial_list)
# 将原来的消毒点按照最邻近点的规则进行重新排序并加入新的列表，让A算法遍历新的消毒点列表
min_xiaodu_x=xiaodu_initial[min_xiaodu(xiaodu_initial)][0]
min_xiaodu_y=xiaodu_initial[min_xiaodu(xiaodu_initial)][1]
lenth_xiaodu=len(xiaodu_x_initial)
min_x_initial=min_xiaodu_x
min_y_initial=min_xiaodu_y


for i in range(lenth_xiaodu):
    min_x_initial, min_y_initial,min_i_initial = nei_min_distance(min_x_initial, min_y_initial, xiaodu_x_initial, xiaodu_y_initial)
    xiaodu_x_initial.pop(min_i_initial)
    xiaodu_y_initial.pop(min_i_initial)
    xiaodu_x.append(min_x_initial)
    xiaodu_y.append(min_y_initial)
xiaodu_list=list(zip(xiaodu_x,xiaodu_y))
xiaodu=np.array(xiaodu_list)


# begin_time=time()
#每一步寻求最优路径，然后将每两个消毒点之间的路径连接起来构成全局路径
count_num=0
start_point=(0,0)
end_point=(0,0)
for i in range(len(xiaodu)):
    path_all = [] #每两个点之间的局部路径
    path_all_x = []
    path_all_y = []
    path_x = []
    path_y = []
    if i<len(xiaodu)-1:
        start_point=xiaodu[i]
        end_point=xiaodu[i+1]
        count_num+=1
        node_c = math.sqrt((start_point[0] - end_point[0]) * (start_point[0] - end_point[0]) + (start_point[1] - end_point[1]) * (
                start_point[1] - end_point[1]))


        class RoadMap():
            """ 读进一张图片，二值化成为有障碍物的二维网格化地图，并提供相关操作
            """

            def __init__(self):
                self.map = test_map
                self.cols = len(self.map[0])
                self.rows = len(self.map)

            def is_valid_xy(self, x, y):
                if x < 0 or x >= self.rows or y < 0 or y >= self.cols:  # 判断点是否超出边界
                    return False
                return True

            def not_obstacle(self, x, y):
                return self.map[x][y] != STAT_OBSTACLE

            def EuclidenDistance(self, xy1, xy2):
                """两个像素点之间的欧几里得距离"""
                dis = 0
                for (x1, x2) in zip(xy1, xy2):
                    dis += (x1 - x2) ** 2
                return dis ** 0.5

            def ManhattanDistance(self, xy1, xy2):
                """两个像素点之间的曼哈顿距离"""
                dis = 0
                for x1, x2 in zip(xy1, xy2):
                    dis += abs(x1 - x2)
                return dis

            def check_path(self, xy1, xy2):
                """碰撞检测 两点之间的连线是否经过障碍物"""
                steps = max(abs(xy1[0] - xy2[0]), abs(xy1[1] - xy2[1]))  # 取横向、纵向较大值，确保经过的每个像素都被检测到
                xs = np.linspace(xy1[0], xy2[0], steps + 1)
                ys = np.linspace(xy1[1], xy2[1], steps + 1)
                for i in range(1, steps):  # 第一个节点和最后一个节点是 xy1，xy2，无需检查
                    if not self.not_obstacle(math.ceil(xs[i]), math.ceil(ys[i])):
                        return False
                return True

            def plot(self, path):
                """绘制地图及路径"""
                out = []
                for x in range(self.rows):
                    temp = []
                    for y in range(self.cols):
                        if self.map[x][y] == STAT_OBSTACLE:
                            temp.append(0)
                        elif self.map[x][y] == STAT_NORMAL:
                            temp.append(255)
                        elif self.map[x][y] == '*':
                            temp.append(127)
                        else:
                            temp.append(255)
                    out.append(temp)
                for x, y in path:
                    out[x][y] = 127
                out = np.array(out)
                if count_num==len(xiaodu)-1:
                    img = Image.fromarray(out)
                    img=img.convert("RGB")
                    # img.save("//home//wang//桌面//3210_path.png")
                    # img.show()
        class Node():
            """
            节点元素，parent用来在成功的时候回溯路径
            """

            def __init__(self, x, y, parent=None, g=0, h=0):
                self.parent = parent
                self.x = x
                self.y = y
                self.g = g
                self.h = h
                self.update()

            def update(self):
                self.f = self.g + self.h

        class A_Star(RoadMap):
            """ x是行索引，y是列索引
            """

            def __init__(self, start=start_point, end=end_point):
                """地图文件，起点，终点"""
                RoadMap.__init__(self)
                self.startXY = tuple(start)
                self.endXY = tuple(end)
                self.closeList = set()
                self.path = []
                self.openList = []  # 堆，只添加，和弹出最小值点，
                self.openDict = dict()  # openList中的 坐标:详细信息 -->不冗余的

            def find_path(self):
                """A*算法寻路主程序"""
                p = Node(self.startXY[0], self.startXY[1],
                         h=self.ManhattanDistance(self.startXY, self.endXY))  # 构建开始节点
                heapq.heappush(self.openList, (p.f, (p.x, p.y)))

                self.openDict[(p.x, p.y)] = p  # 加进dict目录
                while True:
                    current = self.get_minfNode()
                    if (current.x, current.y) == self.endXY:
                        # print('found path successfully..')
                        self.make_path(current)
                        return

                    self.closeList.add((current.x, current.y))  ## 加入closeList
                    del self.openDict[(current.x, current.y)]
                    self.extend_surrounds(current)  # 会更新close list

            def make_path(self, p):
                """从结束点回溯到开始点，开始点的parent==None"""
                while p:
                    self.path.append((p.x, p.y))
                    # path_all.append((p.x,p.y))
                    # path_initial.append((p.x,p.y))
                    # path_x.append((p.y/(p_w/ori_p_w)+int(origion_x/map_resolution))*map_resolution)
                    # path_y.append((ori_p_h-p.x/(p_h /ori_p_h)+int(origion_y/map_resolution))*map_resolution)
                    path_z_real_2.append(0)
                    path_all_y.append(p.x)
                    path_all_x.append(p.y)
                    p = p.parent

            def extend_surrounds(self, node):
                """ 将当前点周围可走的点加到openList中，
                    其中 不在openList中的点 设置parent、F,G,H 加进去，
                         在openList中的点  更新parent、F,G,H
                    (斜向时，正向存在障碍物时不允许穿越)
                """
                motion_direction = [[1, 0], [0, 1], [-1, 0], [0, -1],
                                    [1, 1], [1, -1], [-1, 1], [-1, -1]]
                for dx, dy in motion_direction:
                    x, y = node.x + dx, node.y + dy
                    new_node = Node(x, y)
                    # 位置无效，或者是障碍物, 或者已经在closeList中
                    if not self.is_valid_xy(x, y) or not self.not_obstacle(x, y) or self.in_closeList(new_node):
                        continue
                    if abs(dx) + abs(dy) == 2:  ## 斜向 需检查正向有无障碍物
                        h_x, h_y = node.x + dx, node.y  # 水平向
                        v_x, v_y = node.x, node.y + dy  # 垂直向
                        if not self.is_valid_xy(h_x, h_y) or not self.not_obstacle(h_x, h_y) or self.in_closeList(
                                Node(h_x, h_y)):
                            continue
                        if not self.is_valid_xy(v_x, v_y) or not self.not_obstacle(v_x, v_y) or self.in_closeList(
                                Node(v_x, v_y)):
                            continue
                    # ============ ** 关键 **             ========================
                    # ============ 不在openList中，加进去； ========================
                    # ============ 在openList中，更新      ========================
                    # ============对于openList和openDict来说，操作都一样 ===========
                    new_g = node.g + self.cal_deltaG(node.x, node.y, x, y)
                    sign = False  # 是否执行操作的标志
                    if not self.in_openList(new_node):  # 不在openList中
                        # 加进来，设置 父节点, F, G, H
                        new_node.h = self.cal_H(new_node)
                        sign = True
                    elif self.openDict[(new_node.x, new_node.y)].g > new_g:  # 已在openList中，但现在的路径更好
                        sign = True
                    if sign:
                        new_node.parent = node
                        new_node.g = new_g
                        new_node.f = self.cal_F(new_node)
                        self.openDict[(new_node.x, new_node.y)] = new_node  # 更新dict目录
                        heapq.heappush(self.openList, (new_node.f, (new_node.x, new_node.y)))

            def get_minfNode(self):
                """从openList中取F=G+H值最小的 (堆-O(1))"""
                while True:
                    f, best_xy = heapq.heappop(self.openList)
                    if best_xy in self.openDict:
                        return self.openDict[best_xy]

            def in_closeList(self, node):
                """判断是否在closeList中 (集合-O(1)) """
                return True if (node.x, node.y) in self.closeList else False

            def in_openList(self, node):
                """判断是否在openList中 (字典-O(1))"""
                if not (node.x, node.y) in self.openDict:
                    return False
                else:
                    return True

            def cal_deltaG(self, x1, y1, x2, y2):
                """ 计算两点之间行走的代价
                    （为简化计算）上下左右直走，代价为1.0，斜走，代价为1.4  G值
                """
                if x1 == x2 or y1 == y2:
                    return 1.0
                # return 1.4
                return math.sqrt(2)

            def cal_H(self, node):
                """ 曼哈顿距离 估计距离目标点的距离"""

                # return abs(node.x - self.endXY[0]) + abs(node.y - self.endXY[1])  # 剩余路径的估计长度

                node_h = math.sqrt((node.x - self.endXY[0]) * (node.x - self.endXY[0]) + (node.y - self.endXY[1]) * (node.y - self.endXY[1]))
                if node_h < node_c / 4:
                    return node_h
                return math.log(node_h) * node_h
            def cal_F(self, node):
                """ 计算F值 F = G+H
                    A*算法的精髓：已经消耗的代价G，和预估将要消耗的代价H
                """
                return node.g + node.h


        def path_length(path):
            """计算路径长度"""
            l = 0
            for i in range(len(path) - 1):
                x1, y1 = path[i]
                x2, y2 = path[i + 1]
                if x1 == x2 or y1 == y2:
                    l += 1.0
                else:
                    l += math.sqrt(2)
            return l


        def path_length_e(path):
            """计算路径长度"""
            l = 0
            for i in range(len(path) - 1):
                x1, y1 = path[i]
                x2, y2 = path[i + 1]
                l += np.sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2))
            return l

        # ===== test case ===============

        a = A_Star()
        a.find_path()

        # for i in range(len(path_all)):
        #     # path_x.append((path_x[len(path_all) - 1 - i]/(p_w/ori_p_w)+int(origion_x/map_resolution))*map_resolution)
        #     # path_y.append((ori_p_h-path_y[len(path_all) - 1 - i]/(p_h /ori_p_h)+int(origion_y/map_resolution))*map_resolution)
        #     path_reverse.append(path_all[len(path_all) - 1 - i])  #因为初始路径的顺序是从终点到起始点的回溯路径，所以要将路径点的顺序reverse回来
        #     path_x_reverse.append(path_x[len(path_all) - 1 - i])
        #     path_y_reverse.append(path_y[len(path_all) - 1 - i])
        #     path_x_real_reverse.append((path_x[len(path_all) - 1 - i]/(p_w/ori_p_w)+int(origion_x/map_resolution))*map_resolution)  #再将消毒路径像素点的坐标转化为二维物理坐标
        #     path_y_real_reverse.append((ori_p_h-path_y[len(path_all) - 1 - i]/(p_h /ori_p_h)+int(origion_y/map_resolution))*map_resolution)
        for ii in range(len(path_all_x)):        #如果将此算法投入到实际应用中需要把path_all_y.append(p.y)和path_all_x.append(p.x) 改成 path_all_y.append(p.x)和path_all_x.append(p.y)
            if ii == 0 or ii == len(path_all_x) - 1:
                path_x.append(path_all_x[ii])
                path_y.append(path_all_y[ii])
                path_all.append((path_all_x[ii],path_all_y[ii]))
                path_quanju.append((path_all_x[ii], path_all_y[ii]))
            else:
                if path_all_x[ii - 1] - 2 * path_all_x[ii] + path_all_x[ii + 1] == 0 and path_all_y[ii - 1] - 2 * path_all_y[ii] + path_all_y[ii + 1] == 0:
                    path_x.append(path_all_x[ii])
                    path_y.append(path_all_y[ii])
                    path_all.append((path_all_x[ii], path_all_y[ii]))
                    path_quanju.append((path_all_x[ii], path_all_y[ii]))
        for i in range(len(path_all)):
            # path_x.append((path_x[len(path_all) - 1 - i]/(p_w/ori_p_w)+int(origion_x/map_resolution))*map_resolution)
            # path_y.append((ori_p_h-path_y[len(path_all) - 1 - i]/(p_h /ori_p_h)+int(origion_y/map_resolution))*map_resolution)

            path_reverse.append(path_all[len(path_all) - 1 - i])
            path_x_reverse.append(path_x[len(path_all) - 1 - i])
            path_y_reverse.append(path_y[len(path_all) - 1 - i])
            path_x_real_reverse_2.append((path_x[len(path_all) - 1 - i]/(p_w/ori_p_w)+int(origion_x/map_resolution))*map_resolution)
            path_y_real_reverse_2.append((ori_p_h-path_y[len(path_all) - 1 - i]/(p_h /ori_p_h)+int(origion_y/map_resolution))*map_resolution)

# end_time=time()

print('Path length: '+str(path_length(path_reverse)))
# a.plot(path_reverse)


# for i in range(len(path_x_real_reverse)-angle_jiange):
#     path_a=(path_x_real_reverse[i],path_y_real_reverse[i])
#     path_b=(path_x_real_reverse[i+angle_jiange],path_y_real_reverse[i+angle_jiange])
#     path_yaw.append(rad_ptA_ptB(path_a,path_b))
for i in range(len(path_x_real_reverse_2)):
    if i<len(path_x_real_reverse_2)-angle_jiange:
        path_a=(path_x_real_reverse_2[i],path_y_real_reverse_2[i])
        path_b=(path_x_real_reverse_2[i+angle_jiange],path_y_real_reverse_2[i+angle_jiange])
        path_yaw.append(rad_ptA_ptB(path_a,path_b))
    if len(path_x_real_reverse_2)-angle_jiange<=i<len(path_x_real_reverse_2)-1:
        path_a=(path_x_real_reverse_2[i-6],path_y_real_reverse_2[i-6])
        path_b=(path_x_real_reverse_2[i+1],path_y_real_reverse_2[i+1])
        path_yaw.append(rad_ptA_ptB(path_a,path_b))

#以下为消毒路径的偏航角等三维物理信息
qq_2=[]
qxx_2=[]
qyy_2=[]
qzz_2=[]
# qww=[]
for i in range(len(path_yaw)):
    quaternion = euler_to_quaternion(path_yaw[i], 0.0, 0.0)
    qq_2.append(quaternion[0])
    qxx_2.append(quaternion[1])
    qyy_2.append(quaternion[2])
    qzz_2.append(quaternion[3])
    # qww.append(quaternion[4])


sendAstarPath_2(path_x_real_reverse_2,path_y_real_reverse_2,path_z_real_2,qq_2,qxx_2,qyy_2,qzz_2)
csv_save(file,path_x_real_reverse_2,path_y_real_reverse_2,path_z_real_2,qq_2,qxx_2,qyy_2,qzz_2)
print("Number of disinfection points: "+str(len(xiaodu)))
# run_time=end_time-begin_time
# print("运行时间"+str(run_time))