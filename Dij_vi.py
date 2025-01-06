import sys
import time
import math
import numpy as np
from matplotlib.patches import Rectangle
from collections import OrderedDict
import point_solution_class as point
import copy
import map_Env as map
from decimal import *
getcontext().prec = 6
BandChoose = np.zeros((70, 70, 10))
MinBand = np.zeros((70, 70))

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


def SelectPointInOpenList(map, open_set):  # 从开集中选取代价最小的点
    '''
    :return: 代价最小的点在开集中的位置
    '''
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


    # MinPhaseNum = 0
    # minCost_PointList = []
    #
    # for p in open_set:
    #     cost = p.cost
    #     # print(cost)
    #     if cost < min_cost:
    #         min_cost = cost
    #         selected_index = index
    #         minCost_PointList.clear()
    #         minCost_PointList.append([p, index])
    #     if cost == min_cost:
    #         minCost_PointList.append([p, index])
    #     index += 1
    # # print('min_cost=', min_cost)
    # # print('end')
    # for p in minCost_PointList:
    #     PhaseNum = len(map.Phase_use_list[p[0].x][p[0].y])
    #     if PhaseNum > MinPhaseNum:
    #         MinPhaseNum = PhaseNum
    #         selected_index = p[1]
    # return selected_index


class Dij:
    def __init__(self, Map, Ratio, UserSeq, UserPower, type):
        self.map = Map
        self.search_index = 0
        self.PhaseObCost = 10
        self.open_set = []  # 开集
        self.close_set = []  # 闭集
        self.ratio = Ratio
        self.UserSeq = UserSeq
        self.UserPower = UserPower
        self.OpenPointNum = 0
        self.type = type

    def HeuristicCost(self, p, end_point, TmpMinBandList):
        '''
        :param p: 点类
        :param end_point: 目标点
        :return: A*算法中的估计代价
        '''
        x_dis = abs(end_point.x - p.x)
        y_dis = abs(end_point.y - p.y)
        Path_Est = x_dis + y_dis
        '''
        BandEst = sys.maxsize
        for band in TmpMinBandList:
            if band not in self.map.Phase_use_list[end_point.x][end_point.y]:
                BandEst = min(BandEst, self.ratio)
            else:
                BandEst = min(BandEst, 0)
        '''
        HCost = Path_Est #  + BandEst

        return float('%.4f' % HCost)

    def Viterbi(self, map, past, x, y, ratio, basecost, end_point):  # 构建路径
        '''
        Viterbi选取频段
        '''
        path = []
        x_path = []
        y_path = []
        Tmpbasecost = copy.deepcopy(basecost)
        # print("base:", basecost[past.x][past.y])
        PastBandList = [0 for _ in range(10)]
        BandCost = [sys.maxsize for _ in range(10)]
        MinBandCost = sys.maxsize
        TmpMinBnadList = []
        TmpMinBand = -1
        for Band in map.Phase_use_list[x][y]:  # Viterbi选取频段
            ViterbiCost = sys.maxsize
            '''
            if Band in map.Phase_use_list[end_point.x][end_point.y]:
                ExHCost = 0.0
            else:
                ExHCost = ratio
            '''
            for PastBand in map.Phase_use_list[past.x][past.y]:
                if PastBand == 0:
                    continue
                if Band != PastBand:
                    ExCost = ratio
                else:
                    ExCost = 0.0
                TempCost = float('%.4f' % (basecost[past.x][past.y][PastBand] + ExCost))
                if TempCost < ViterbiCost:
                    ViterbiCost = TempCost
                    PastBandList[Band] = PastBand
            BandCost[Band] = ViterbiCost
            if ViterbiCost < MinBandCost:
                '''
                if ExHCost != 0:
                    flag = 1
                else:
                    flag = 0
                '''
                MinBandCost = ViterbiCost
                TmpMinBand = Band
                TmpMinBnadList.clear()
                TmpMinBnadList.append(Band)
            elif ViterbiCost == MinBandCost:
                TmpMinBnadList.append(Band)
        BandCost[0] = MinBandCost
        Tmpbasecost[x][y] = copy.deepcopy(BandCost)
        # ax.plot(x_path, y_path, linewidth = 0.5, color = 'r', label='A*路径')
        return Tmpbasecost, PastBandList, TmpMinBand, TmpMinBnadList


    def BuildPath(self, p, start_point, band, ax):  # 构建路径
        '''
        :return: A*路径
        '''
        A_star_cost = 0
        path = []
        x_path = []
        y_path = []
        path_coordinate = []
        Band = [band]
        # print("band:", band)
        # print('A*规划的路径')
        while True:  # 根据点的关系构建路径
            path.insert(0, p)
            p.band = band
            if IsStartPoint(p, start_point):
                break
            else:
                preband = int(BandChoose[p.x][p.y][band])
                Band.append(preband)
                band = preband
                p = p.parent
        for p in path:
            # rec = Rectangle((p.x, p.y), 1, 1, color='g')
            # ax.add_patch(rec)
            # plt.draw()
            # print('point_cost=', p.cost, 'point_x=', p.x, 'point_y=', p.y, 'basecost=', basecost[p.x][p.y])
            x_path.insert(0, p.x)
            y_path.insert(0, p.y)
            path_coordinate.append([p.x, p.y])
            # self.SaveImage(plt)
            if p == path[-1]:
                A_star_cost = p.cost
        Band.reverse()
        # ax.plot(x_path, y_path, linewidth = 0.5, color = 'r', label='A*路径')
        return path_coordinate, A_star_cost, Band, path, ax


    def RunAndSaveImage(self, start_point, end_point, ax):  # 实现A*算法
        start_point.cost = 0.
        start_point.time = 0
        basecost = np.zeros((self.map.size, self.map.size, 10))
        BaseTrueCost = np.zeros((self.map.size, self.map.size, 10))
        # path_coordinate = []
        # A_star_cost = 0
        self.open_set.append(start_point)

        if len(self.map.Phase_obstacle) != 0:
            ob = np.vstack(
                [self.map.obstacle_point, self.map.Phase_obstacle])
        else:
            ob = self.map.obstacle_point
        # self.map.obstacle_point = ob
        while True:
            index = SelectPointInOpenList(self.map, self.open_set)
            if index < 0:
                print('Dijvi Fail')
                return 0, 0, 0, 0, ax
            p = self.open_set[index]
            # rec = Rectangle((p.x, p.y), 1, 1, color='c')
            # ax.add_patch(rec)


            if IsEndPoint(p, end_point):  # 更新到目标点后构建路径
                return self.BuildPath(p, start_point, int(MinBand[p.x][p.y]), ax)


            del self.open_set[index]
            self.close_set.append(p)
            # pathtime += 1

            x = p.x
            y = p.y
            self.ProcessPoint(x - 1, y, p, 1, start_point, end_point, basecost, BaseTrueCost, ob, ax)
            self.ProcessPoint(x, y - 1, p, 1, start_point, end_point, basecost, BaseTrueCost, ob, ax)
            self.ProcessPoint(x + 1, y, p, 1, start_point, end_point, basecost, BaseTrueCost, ob, ax)
            self.ProcessPoint(x, y + 1, p, 1, start_point, end_point, basecost, BaseTrueCost, ob, ax)
            # self.ProcessPoint(x - 1, y - 1, p, 2, start_point, end_point, basecost, BaseTrueCost, ob, ax)
            # self.ProcessPoint(x - 1, y + 1, p, 2, start_point, end_point, basecost, BaseTrueCost, ob, ax)
            # self.ProcessPoint(x + 1, y - 1, p, 2, start_point, end_point, basecost, BaseTrueCost, ob, ax)
            # self.ProcessPoint(x + 1, y + 1, p, 2, start_point, end_point, basecost, BaseTrueCost, ob, ax)
            # print("x:", x, "y:", y, "basecost:", basecost[x][y])

    def ProcessPoint(self, x, y, past, type, start_point, end_point, basecost, BaseTrueCost, ob, ax):  # 点移动后更新开集与闭集中点的状态
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
        TmpBasecost, TempBandChoose, TempMinBand, TmpMinBandList = self.Viterbi(self.map, past, x, y, self.ratio, basecost, end_point)
        # if type == 1:  # 根据是否斜着走增加不同的实际代价
        temp_basecost = [float('%.4f' % (cost + 1)) for cost in TmpBasecost[x][y]]
            # print("tmpCost:", temp_basecost)
        '''
        if type == 2:
            temp_basecost = [float('%.4f' % (cost + np.sqrt(2))) for cost in TmpBasecost[x][y]]
        '''
        # print('temp_basecost=', temp_basecost)
        '''
        if TempMinBand not in self.map.Phase_use_list[end_point.x][end_point.y]:
            ExHCost = float('%.4f' % self.ratio)
        else:
            ExHCost = float('%.4f' % 0.0)
        '''
        if IsInOpenList(self.open_set, x, y):
            p = FindPointList(x, y, self.open_set)
            CmpCost = temp_basecost[0]
            '''
            if TempBandChoose[TempMinBand] != TempMinBand and MinBand[x][y] == TempMinBand and BandChoose[x][y][TempMinBand] == TempMinBand:
                print("x:", x, " y:", y, "plus")
                CmpCost += self.ratio
            '''
            # past_basecost = basecost[x][y][0]
            # p.cost = Decimal(basecost[p.x][p.y][0] + HeuristicCost(p, end_point) + ExHCost)
            if CmpCost < basecost[x][y][0]:  # 当有更小代价的路径时，更新路径
                basecost[x][y] = copy.deepcopy(temp_basecost)
                BandChoose[x][y] = copy.deepcopy(TempBandChoose)
                MinBand[x][y] = TempMinBand
                # rec = Rectangle((p.x, p.y), 1, 1, color='red')
                # ax.add_patch(rec)
                # self.SaveImage(plt)
                # print('##############################################')
                if self.type == "Dij JPF-Viterbi":
                    p.cost = Decimal(temp_basecost[0])  # + HeuristicCost(p, end_point))
                else:
                    p.cost = Decimal(temp_basecost[0] + self.HeuristicCost(p, end_point, TmpMinBandList))
                p.time = past.time + 1
                del p.parent
                p.parent = past
        else:
            p = point.Point(x, y, self.UserSeq, self.UserPower)
            p.time = past.time + 1
            '''
            if ax != 0:
                rec = Rectangle((p.x - 0.5, p.y - 0.5), 1, 1, color='lightblue', label='搜索过的点')
                ax.add_patch(rec)
            '''
            self.search_index += 1
            # self.SaveImage(plt)
            p.parent = past
            basecost[x][y] = copy.deepcopy(temp_basecost)
            BandChoose[x][y] = copy.deepcopy(TempBandChoose)
            MinBand[x][y] = TempMinBand
            if self.type == "Dij JPF-Viterbi":
                p.cost = Decimal(temp_basecost[0])  # + HeuristicCost(p, end_point))
            else:
                p.cost = Decimal(temp_basecost[0] + self.HeuristicCost(p, end_point, TmpMinBandList))
            # p.time = pathtime
            # print('temp_basecost=', temp_basecost, 'HeuristicCost=', self.HeuristicCost(p, end_point))
            # print('Process Point [', p.x, ',', p.y, ']', ', cost:', p.cost)
            self.open_set.append(p)
            self.OpenPointNum += 1