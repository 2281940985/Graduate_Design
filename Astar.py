import sys
import time
import math
import numpy as np
import Draw_Path
from matplotlib.patches import Rectangle
from collections import OrderedDict
import point_solution_class as point
import map_Env as map
from decimal import *
getcontext().prec = 6

def IsValidPoint(map, x, y, ob):
    '''
    :param x: 点的横坐标
    :param y: 点的纵坐标
    :return: 判断点是否可行(超出边界，撞到障碍物)
    '''
    if x < 0 or y < 0:
        return False
    if x >= map.size or y >= map.size:
        return False
    return not map.IsObstacle(x, y, ob)


def IsInPointList(x, y, point_list):
    '''
    :param point_list: 点集
    :return: 判断某点是否在某点集中
    '''
    for point in point_list:
        if point.x == x and point.y == y:
            return True
    return False




def FindPointList(x, y, point_list):  # 根据横纵坐标在某点集中找到该点
    for point in point_list:
        if point.x == x and point.y == y:
            return point


def IsInOpenList(open_set, x, y):  # 判断是否在开集合
    return IsInPointList(x, y, open_set)


def IsInCloseList(close_set, x, y):  # 判断是否在闭集合
    return IsInPointList(x, y, close_set)


def IsStartPoint(p, start_point):  # 是否为起始点
    return p.x == start_point.x and p.y == start_point.y


def IsEndPoint(p, end_point):  # 是否为目标点
    return p.x == end_point.x and p.y == end_point.y


def point_Distance(x1, y1, x2, y2):  # 两点之间的直线距离
    return np.sqrt(math.pow((x1 - x2), 2) + math.pow((y1 - y2), 2))


def HeuristicCost(p, end_point):
    '''
    :param p: 点类
    :param end_point: 目标点
    :return: A*算法中的估计代价
    '''
    x_dis = abs(end_point.x - p.x)
    y_dis = abs(end_point.y - p.y)
    Path_Est = x_dis + y_dis
    HCost = Path_Est

    return float('%.4f' % HCost)



def SelectPointInOpenList(open_set):  # 从开集中选取代价最小的点的位置
    index = 0
    selected_index = -1
    min_cost = sys.maxsize
    for p in open_set:
        cost = p.cost
        if cost < min_cost:
            min_cost = cost
            selected_index = index
        index += 1
    return selected_index


class astar:
    def __init__(self, Map, Ratio, UserSeq, UserPower):
        self.map = Map
        self.search_index = 0
        self.PhaseObCost = 10
        self.open_set = []  # 开集
        self.close_set = []  # 闭集
        self.ratio = Ratio
        self.UserSeq = UserSeq
        self.UserPower = UserPower
        self.OpenPointNum = 0



    def BuildPath(self, p, start_point, ax):  # 构建路径
        A_star_cost = p.cost
        path = []
        x_path = []
        y_path = []
        path_coordinate = []
        while True:  # 根据点的关系构建路径
            path.insert(0, p)
            if IsStartPoint(p, start_point):
                break
            else:
                p = p.parent
        for point in path:
            x_path.append(point.x)
            y_path.append(point.y)
            path_coordinate.append([point.x, point.y])
            # print('x: ', point.x, ' y: ', point.y, ' cost: ', point.cost)
        # print("x:", x_path)
        # print("y:", y_path)
        return path_coordinate, A_star_cost, path, ax


    def RunAndSaveImage(self, start_point, end_point, ax):  # 实现A*算法
        start_point.cost = 0.
        start_point.time = 0
        basecost = np.zeros((self.map.size, self.map.size))
        AddBandCost = np.zeros((self.map.size, self.map.size))
        self.open_set.append(start_point)

        if len(self.map.Phase_obstacle) != 0:
            ob = np.vstack(
                [self.map.obstacle_point, self.map.Phase_obstacle])
        else:
            ob = self.map.obstacle_point
        # self.map.obstacle_point = ob
        while True:
            index = SelectPointInOpenList(self.open_set)
            if index < 0:
                # print('##')
                '''
                for point in self.close_set:
                    if IsEndPoint(point, end_point):
                        return self.BuildPath(point, start_point, ax)
                '''
                # print('Dij Fail')
                return 0, 0, 0, ax
            p = self.open_set[index]


            if IsEndPoint(p, end_point):  # 更新到目标点后构建路径
                 return self.BuildPath(p, start_point, ax)


            del self.open_set[index]
            self.close_set.append(p)


            x = p.x
            y = p.y
            self.ProcessPoint(x - 1, y, p, 1, start_point, end_point, basecost, ob, ax)
            self.ProcessPoint(x, y - 1, p, 1, start_point, end_point, basecost, ob, ax)
            self.ProcessPoint(x + 1, y, p, 1, start_point, end_point, basecost, ob, ax)
            self.ProcessPoint(x, y + 1, p, 1, start_point, end_point, basecost, ob, ax)
            # self.ProcessPoint(x - 1, y - 1, p, 2, start_point, end_point, basecost, ob, ax)
            # self.ProcessPoint(x - 1, y + 1, p, 2, start_point, end_point, basecost, ob, ax)
            # self.ProcessPoint(x + 1, y - 1, p, 2, start_point, end_point, basecost, ob, ax)
            # self.ProcessPoint(x + 1, y + 1, p, 2, start_point, end_point, basecost, ob, ax)

    def ProcessPoint(self, x, y, past, type, start_point, end_point, basecost, ob, ax):  # 点移动后更新开集与闭集中点的状态
        '''
        :param x, y: 当前点的横纵坐标
        :param past: 上一个点
        :param type: 1:上下左右 2:斜着走
        :param basecost: 路径的实际代价
        '''
        if not IsValidPoint(self.map, x, y, ob):
            return
        if x == past.x + 1 and y == past.y + 1:  # 判断无法斜着走的情况
            if not IsValidPoint(self.map, x - 1, y, ob) and not IsValidPoint(self.map, x, y - 1, ob):
                return
        if x == past.x + 1 and y == past.y - 1:
            if not IsValidPoint(self.map, x - 1, y, ob) and not IsValidPoint(self.map, x, y + 1, ob):
                return
        if x == past.x - 1 and y == past.y + 1:
            if not IsValidPoint(self.map, x + 1, y, ob) and not IsValidPoint(self.map, x, y - 1, ob):
                return
        if x == past.x - 1 and y == past.y - 1:
            if not IsValidPoint(self.map, x + 1, y, ob) and not IsValidPoint(self.map, x, y + 1, ob):
                return
        if IsInCloseList(self.close_set, x, y):
            return
        if type == 1:  # 根据是否斜着走增加不同的实际代价
            temp_basecost = 1 + basecost[past.x][past.y]
        if type == 2:
            temp_basecost = np.sqrt(2) + basecost[past.x][past.y]
        if IsInOpenList(self.open_set, x, y):
            p = FindPointList(x, y, self.open_set)
            if temp_basecost < basecost[x][y]:  # 当有更小代价的路径时，更新路径
                basecost[x][y] = temp_basecost
                p.cost = Decimal(temp_basecost + HeuristicCost(p, end_point))
                p.time = past.time + 1
                del p.parent
                p.parent = past
        else:
            p = point.Point(x, y, self.UserSeq, self.UserPower)
            p.time = past.time + 1
            self.search_index += 1
            p.parent = past
            basecost[x][y] = temp_basecost
            p.cost = Decimal(temp_basecost + HeuristicCost(p, end_point))
            # print('p.cost:', p.cost)
            # if ax != 0:
            #     rec = Rectangle((p.x - 0.5, p.y - 0.5), 1, 1, color='lightblue', label='搜索过的点')
            #     ax.add_patch(rec)
            self.open_set.append(p)
            self.OpenPointNum += 1