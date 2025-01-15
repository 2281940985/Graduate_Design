import sys
import os
import pickle
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import ConnectionPatch
import Dij_vi
import DrawBaseData
import Draw_Path
import Dij_Dp
import CheckPath
import copy
import pandas as pd
from matplotlib.collections import PatchCollection
import Astar
from matplotlib.patches import Rectangle
import point_solution_class
import map_Env
from collections import OrderedDict
from matplotlib.backends.backend_agg import FigureCanvasAgg as FigureCanvas
import time

plt.rcParams['font.sans-serif'] = ['SimHei']
plt.rcParams['axes.unicode_minus'] = False

# DrawBaseData.Draw()


LoadBaseOb = True  # 是否加载地图
PowerChange = False  # 用户功率是否改变
Bs_Power_Total = 3.2  # W
PowerRatio = [1]
StartPointList = [point_solution_class.Point(5, 5, 0, Bs_Power_Total*PowerRatio[0])]
EndPointList = [point_solution_class.Point(65, 65, 0, Bs_Power_Total*PowerRatio[0])]
UserNum = len(StartPointList)
# start_point = point_solution_class.Point(3, 4)  # 起始点
# end_point = point_solution_class.Point(67, 68)  # 目标点
'''
map = map_Env.RandomMap(StartPointList, EndPointList, Bs_Power_Total*PowerRatio[0], LoadBaseOb, PowerChange)  # 随机生成地图障碍物



f, ax_Phase = plt.subplots(3, 3)
plt.subplots_adjust(left=0.05, right=0.95, top=0.95, bottom=0.05)

for i in range(map.size):  # 根据障碍物生成地图
    for j in range(map.size):
        if map.Is_Phase_Obstacle1(i, j):
            rec_ob = Rectangle((i - 0.5, j - 0.5), 1, 1, facecolor='red', label='拥塞频段1')
            ax_Phase[0, 0].add_patch(rec_ob)
        if map.Is_Phase_Obstacle2(i, j):
            rec_ob = Rectangle((i - 0.5, j - 0.5), 1, 1, facecolor='orange', label='拥塞频段2')
            ax_Phase[0, 1].add_patch(rec_ob)
        if map.Is_Phase_Obstacle3(i, j):
            rec_ob = Rectangle((i - 0.5, j - 0.5), 1, 1, facecolor='yellow', label='拥塞频段3')
            ax_Phase[0, 2].add_patch(rec_ob)
        if map.Is_Phase_Obstacle4(i, j):
            rec_ob = Rectangle((i - 0.5, j - 0.5), 1, 1, facecolor='green', label='拥塞频段4')
            ax_Phase[1, 0].add_patch(rec_ob)
        if map.Is_Phase_Obstacle5(i, j):
            rec_ob = Rectangle((i - 0.5, j - 0.5), 1, 1, facecolor='blue', label='拥塞频段5')
            ax_Phase[1, 1].add_patch(rec_ob)
        if map.Is_Phase_Obstacle6(i, j):
            rec_ob = Rectangle((i - 0.5, j - 0.5), 1, 1, facecolor='cyan', label='拥塞频段6')
            ax_Phase[1, 2].add_patch(rec_ob)
        if map.Is_Phase_Obstacle7(i, j):
            rec_ob = Rectangle((i - 0.5, j - 0.5), 1, 1, facecolor='purple', label='拥塞频段7')
            ax_Phase[2, 0].add_patch(rec_ob)
        if map.Is_Phase_Obstacle8(i, j):
            rec_ob = Rectangle((i - 0.5, j - 0.5), 1, 1, facecolor='pink', label='拥塞频段8')
            ax_Phase[2, 1].add_patch(rec_ob)
        if map.Is_Phase_Obstacle9(i, j):
            rec_ob = Rectangle((i - 0.5, j - 0.5), 1, 1, facecolor='brown', label='拥塞频段9')
            ax_Phase[2, 2].add_patch(rec_ob)
        if map.IsObstacle(i, j, map.obstacle_point):
            for ii in range(3):
                for jj in range(3):
                    rec_ob = Rectangle((i - 0.5, j - 0.5), 1, 1, facecolor='gray', label='障碍物')
                    ax_Phase[ii, jj].add_patch(rec_ob)

for i in range(3):
    for j in range(3):
        handles, labels = ax_Phase[i, j].get_legend_handles_labels()
        by_label = OrderedDict(zip(labels, handles))
        ax_Phase[i, j].legend(by_label.values(), by_label.keys(), bbox_to_anchor=(0, 1), loc=3, borderaxespad=0,
                              fontsize=5,
                              frameon=False, ncol=5)
        ax_Phase[i, j].axis('equal')
        ax_Phase[i, j].tick_params(axis='x', labelsize=3)
        ax_Phase[i, j].tick_params(axis='y', labelsize=3)
        ax_Phase[i, j].set_xlim([-1, map.size])
        ax_Phase[i, j].set_ylim([-1, map.size])

f.savefig('频段障碍.jpg', dpi=1080, format='jpg')


def print_2d_list(lst):
    for row in lst:
        for element in row:
            print(element, end=' ')
        print()
    print("\n")


def print_list(lst):
    indexOld = 0
    for row in lst:
        indexNew = row[0]
        if indexNew != indexOld:
            indexOld = indexNew
            print("\n")
        print(row, end=' ')
    print("\n")
'''

PathLength_ori = []
PathCost_ori = []
BandChange_ori = []
OpenPointNum_ori = []
TimeCostList_ori = []



PathLength_Dijvi = []
PathCost_Dijvi = []
BandChange_Dijvi = []
OpenPointNum_Dijvi = []
TimeCostList_Dijvi = []


PathLength_Dij = []
PathCost_Dij = []
BandChange_Dij = []
OpenPointNum_Dij = []
TimeCostList_Dij = []

PathLength_Astar = []
PathCost_Astar = []
BandChange_Astar = []
OpenPointNum_Astar = []
TimeCostList_Astar = []

PathLength_Astarvi = []
PathCost_Astarvi = []
BandChange_Astarvi = []
OpenPointNum_Astarvi = []
TimeCostList_Astarvi = []


Ratio = [i*1.5 for i in range(1, 11)]
RatioPlotList = [i*1.5 for i in range(1, 11)]  # i*1.5 for i in range(1, 6)]
OpenPointNumRatio = []
CostRatio = []
PathInfo = []
Dij_VierbiPath = []
Dij_ViterbiCost = 0
BandUseDij_Viterbi = []
# Dijvi_pathinfo

'''
# Dij_Viterbi
f_Dij_Viterbi = plt.figure()
ax_Dij_Viterbi = f_Dij_Viterbi.add_subplot(111)
ax_Dij_Viterbi = Draw_Path.DrawAxBase(map, ax_Dij_Viterbi)


# Dij
f_Dij = plt.figure()
ax_Dij = f_Dij.add_subplot(111)
ax_Dij = Draw_Path.DrawAxBase(map, ax_Dij)
'''
ax_Dij = 0
ax_Dij_Viterbi = 0
TypeDij_Dp = "Dij JSPD-DP"
TypeAstar_Dp = "A* JSPD-DP"
TypeDij_vi = "Dij JSPD-Viterbi"
TypeAstar_vi = "A* JSPD-Viterbi"
RunTime = 1
runtime = 0
while runtime < RunTime:
    map = map_Env.RandomMap(StartPointList, EndPointList, Bs_Power_Total * PowerRatio[0], LoadBaseOb, PowerChange)
    for ratio in Ratio:
        # map = map_Env.RandomMap(StartPointList, EndPointList, Bs_Power_Total*PowerRatio[0], LoadBaseOb, PowerChange)
        if ratio in RatioPlotList:
            f_Dij = plt.figure()
            ax_Dij = f_Dij.add_subplot(111)
            ax_Dij = Draw_Path.DrawAxBase(map, ax_Dij)#ax_Dij为图片信息

            f_Dij_Viterbi = plt.figure()
            ax_Dij_Viterbi = f_Dij_Viterbi.add_subplot(111)
            ax_Dij_Viterbi = Draw_Path.DrawAxBase(map, ax_Dij_Viterbi)

            f_Astar = plt.figure()
            ax_Astar = f_Astar.add_subplot(111)
            ax_Astar = Draw_Path.DrawAxBase(map, ax_Astar)

            f_Astar_Viterbi = plt.figure()
            ax_Astar_Viterbi = f_Astar_Viterbi.add_subplot(111)
            ax_Astar_Viterbi = Draw_Path.DrawAxBase(map, ax_Astar_Viterbi)

            f_ori = plt.figure()
            ax_ori = f_ori.add_subplot(111)
            ax_ori = Draw_Path.DrawAxBase(map, ax_ori)
        else:
            ax_Dij = 0
            ax_Dij_Viterbi = 0
            ax_Astar = 0
            ax_Astar_Viterbi = 0
            ax_ori = 0
        UserSeq = 0
        PathViable = False
        for start_point, end_point, powerratio in zip(StartPointList, EndPointList, PowerRatio):
            print("ratio:", ratio)
            while not PathViable:
                # Dp

                BaseMap = copy.deepcopy(map)
                Dij_dp = Dij_Dp.Dij(BaseMap, ratio, UserSeq, Bs_Power_Total * powerratio, TypeDij_Dp)#初始化
                t_Dij = time.time()
                Dij_Path, Dij_Cost, Dij_PathPoint, ax_Dij = Dij_dp.RunAndSaveImage(start_point, end_point, ax_Dij)
                if Dij_Path == 0:
                    PathViable = False
                    map = map_Env.RandomMap(StartPointList, EndPointList, Bs_Power_Total*PowerRatio[0], LoadBaseOb, PowerChange)
                    continue
                else:
                    PathViable = True
                    #保存Dij_path
                    np.save(rf"./Path/Dij_path_{ratio}.npy", Dij_Path)
                BandChange, BandUseDij, _ = Draw_Path.minimize_frequency_switches_with_frequencis(map, Dij_Path)
                TimeCost_Dij = time.time() - t_Dij

                CheckCostDij = CheckPath.CheckPathCost(Dij_Path, BandChange, ratio)
                TimeCostList_Dij.append(TimeCost_Dij)
                PathLength_Dij.append(len(Dij_Path)/10)
                PathCost_Dij.append(CheckCostDij)
                BandChange_Dij.append(BandChange)
                OpenPointNum_Dij.append(Dij_dp.OpenPointNum)
                ax_Dij, _, x_Dij_dp_path, y_Dij_dp_path = Draw_Path.Astar_DrawPath(map, Dij_Path, BandUseDij, ax_Dij, 0)
                Draw_Path.DrawPhasePathMap(x_Dij_dp_path, y_Dij_dp_path, BandUseDij, f"Dij_Dp_{ratio}")#画频谱轨迹图
                print("OpenPointNum_Dij:", Dij_dp.OpenPointNum)
                print("Dij_Path:", Dij_Path)
                print("Dij_Cost:", Dij_Cost)
                print("Dij_Cost-Check:", CheckCostDij)
                print("Dij_Band:", BandUseDij, "\n")

                # Astar Dp
                BaseMap = copy.deepcopy(map)
                Astar_dp = Dij_Dp.Dij(BaseMap, ratio, UserSeq, Bs_Power_Total * powerratio, TypeAstar_Dp)
                t_Astar = time.time()
                Astar_Path, Astar_Cost, Astar_PathPoint, ax_Astar = Astar_dp.RunAndSaveImage(start_point, end_point, ax_Astar)
                BandChangeAstar, BandUseAstar, _ = Draw_Path.minimize_frequency_switches_with_frequencis(BaseMap, Astar_Path)
                TimeCost_Astar = time.time() - t_Astar

                CheckCostAstar = CheckPath.CheckPathCost(Astar_Path, BandChangeAstar, ratio)
                TimeCostList_Astar.append(TimeCost_Astar)
                PathLength_Astar.append(len(Astar_Path)/10)
                PathCost_Astar.append(CheckCostAstar)
                BandChange_Astar.append(BandChangeAstar)
                OpenPointNum_Astar.append(Astar_dp.OpenPointNum)
                ax_Astar, _, x_Astar_dp_path, y_Astar_dp_path = Draw_Path.Astar_DrawPath(BaseMap, Astar_Path, BandUseAstar, ax_Astar, 0)
                Draw_Path.DrawPhasePathMap(x_Astar_dp_path, y_Astar_dp_path, BandUseAstar, f"Astar_Dp_{ratio}")
                print("OpenPointNum_Astar:", Astar_dp.OpenPointNum)
                print("Astar_Path:", Astar_Path)
                print("Astar_Cost:", Astar_Cost)
                print("Astar_Cost-Check:", CheckCostAstar)
                print("Astar_Band:", BandUseAstar, "\n")



                # vi
                BaseMap = copy.deepcopy(map)
                Dij_Viterbi = Dij_vi.Dij(BaseMap, ratio, UserSeq, Bs_Power_Total*powerratio, TypeDij_vi)
                t_Dijvi = time.time()
                Dij_VierbiPath, Dij_ViterbiCost, BandUseDij_Viterbi, Dijvi_pathinfo, ax_Dij_Viterbi = Dij_Viterbi.RunAndSaveImage(start_point, end_point, ax_Dij_Viterbi)
                TimeCost_Dijvi = time.time() - t_Dijvi
                ChangeNumDijvi = Draw_Path.BandChangeNum(BandUseDij_Viterbi)


                TimeCostList_Dijvi.append(TimeCost_Dijvi)
                PathLength_Dijvi.append(len(Dij_VierbiPath)/10)
                BandChange_Dijvi.append(ChangeNumDijvi)
                OpenPointNum_Dijvi.append(Dij_Viterbi.OpenPointNum)
                CheckCostDijvi = CheckPath.CheckPathCost(Dij_VierbiPath, ChangeNumDijvi, ratio)
                PathCost_Dijvi.append(CheckCostDijvi)


                ax_Dij_Viterbi, BandUseDij_Vi, x_Dij_Viterbi_path, y_Dij_Viterbi_path = Draw_Path.Astar_DrawPath(map, Dij_VierbiPath, BandUseDij_Viterbi, ax_Dij_Viterbi, 1)
                Draw_Path.DrawPhasePathMap(x_Dij_Viterbi_path, y_Dij_Viterbi_path, BandUseDij_Vi, f"Dij_Viterbi_{ratio}")
                # PathInfo[UserSeq] = Dijvi_pathinfo
                print("OpenPointNum_Dijvi:", Dij_Viterbi.OpenPointNum)
                print("Dij_ViterbiPath:", Dij_VierbiPath)
                print("Dij_ViterbiCost:", Dij_ViterbiCost)
                print("Dij_ViterbiCost-Check:", CheckCostDijvi)
                print("Dij_ViterbiBand:", BandUseDij_Vi, "\n")

                #Astarvi
                BaseMap = copy.deepcopy(map)
                Astar_Viterbi = Dij_vi.Dij(BaseMap, ratio, UserSeq, Bs_Power_Total*powerratio, TypeAstar_vi)
                t_Astarvi = time.time()
                Astar_VierbiPath, Astar_ViterbiCost, BandUseAstar_Viterbi, Astarvi_pathinfo, ax_Astar_Viterbi = Astar_Viterbi.RunAndSaveImage(start_point, end_point, ax_Astar_Viterbi)
                TimeCost_Astarvi = time.time() - t_Astarvi
                ChangeNumAstarvi = Draw_Path.BandChangeNum(BandUseAstar_Viterbi)


                TimeCostList_Astarvi.append(TimeCost_Astarvi)
                PathLength_Astarvi.append(len(Astar_VierbiPath)/10)
                BandChange_Astarvi.append(ChangeNumAstarvi)
                OpenPointNum_Astarvi.append(Astar_Viterbi.OpenPointNum)
                CheckCostAstarvi = CheckPath.CheckPathCost(Astar_VierbiPath, ChangeNumAstarvi, ratio)
                PathCost_Astarvi.append(CheckCostAstarvi)


                ax_Astar_Viterbi, BandUseAstar_Vi, x_Astar_Viterbi_path, y_Astar_Viterbi_path = Draw_Path.Astar_DrawPath(BaseMap, Astar_VierbiPath, BandUseAstar_Viterbi, ax_Astar_Viterbi, 1)
                Draw_Path.DrawPhasePathMap(x_Astar_Viterbi_path, y_Astar_Viterbi_path, BandUseAstar_Vi,f"Astar_Viterbi_{ratio}")
                # PathInfo[UserSeq] = Dijvi_pathinfo
                print("OpenPointNum_Astarvi:", Astar_Viterbi.OpenPointNum)
                print("Astar_ViterbiPath:", Astar_VierbiPath)
                print("Astar_ViterbiCost:", Astar_ViterbiCost)
                print("Astar_ViterbiCost-Check:", CheckCostAstarvi)
                print("Astar_ViterbiBand:", BandUseAstar_Vi, "\n")


                #Astar
                BaseMap = copy.deepcopy(map)
                ori = Astar.astar(BaseMap, ratio, UserSeq, Bs_Power_Total * powerratio)
                t_ori = time.time()
                ori_Path, ori_Cost, ori_PathPoint, ax_ori = ori.RunAndSaveImage(start_point, end_point, ax_ori)
                BandChangeori, BandUseori, _ = Draw_Path.minimize_frequency_switches_with_frequencis(BaseMap, ori_Path)
                TimeCost_ori = time.time() - t_ori

                CheckCostori = CheckPath.CheckPathCost(ori_Path, BandChangeori, ratio)
                TimeCostList_ori.append(TimeCost_ori)
                PathLength_ori.append(len(ori_Path)/10)
                PathCost_ori.append(CheckCostori)
                BandChange_ori.append(BandChangeori)
                OpenPointNum_ori.append(ori.OpenPointNum)
                ax_ori, _, x_ori_path, y_ori_path = Draw_Path.Astar_DrawPath(BaseMap, ori_Path, BandUseori, ax_ori, 0)
                Draw_Path.DrawPhasePathMap(x_ori_path, y_ori_path, BandUseori,f"ori_{ratio}")
                print("ori_OpenPointNum:", ori.OpenPointNum)
                print("ori_Path:", ori_Path)
                print("ori_Cost:", ori_Cost)
                print("ori_Cost-Check:", CheckCostori)
                print("ori_Band:", BandUseori, "\n\n")


                UserSeq += 1

                if ratio in RatioPlotList:
                    # Dijvi画图
                    handles, labels = ax_Dij_Viterbi.get_legend_handles_labels()
                    by_label = OrderedDict(zip(labels, handles))
                    ax_Dij_Viterbi.legend(by_label.values(), by_label.keys(), bbox_to_anchor=(0, 1), loc='upper left',
                                          borderaxespad=0,
                                          fontsize=6,
                                          frameon=False, ncol=1)
                    # ax_Dij_Viterbi.set_title('Dijkstra JPF-Viterbi \u03BB = {}'.format(ratio), fontsize=10, pad=10, loc='center')
                    ax_Dij_Viterbi.axis('equal')
                    f_Dij_Viterbi.savefig(f'Dijvi \u03BB={ratio}下的路径.jpg', dpi=1080, format='jpg', bbox_inches='tight')

                    # Dij画图
                    handles, labels = ax_Dij.get_legend_handles_labels()
                    by_label = OrderedDict(zip(labels, handles))
                    ax_Dij.legend(by_label.values(), by_label.keys(), bbox_to_anchor=(0, 1), loc='upper left',
                                  borderaxespad=0,
                                  fontsize=6,
                                  frameon=False, ncol=1)
                    # ax_Dij.set_title('Dijkstra JPF-DP \u03BB = {}'.format(ratio), fontsize=10, pad=10, loc='center')
                    ax_Dij.axis('equal')
                    f_Dij.savefig(f'Dijdp \u03BB={ratio}下的路径.jpg', dpi=1080, format='jpg', bbox_inches='tight')

                    # Astar画图
                    handles, labels = ax_Astar.get_legend_handles_labels()
                    by_label = OrderedDict(zip(labels, handles))
                    ax_Astar.legend(by_label.values(), by_label.keys(), bbox_to_anchor=(0, 1), loc='upper left',
                                    borderaxespad=0,
                                    fontsize=6,
                                    frameon=False, ncol=1)
                    # ax_Astar.set_title('A* JPF-DP \u03BB = {}'.format(ratio), fontsize=10, pad=10, loc='center')
                    ax_Astar.axis('equal')
                    f_Astar.savefig(f'Astardp \u03BB={ratio}下的路径.jpg', dpi=1080, format='jpg', bbox_inches='tight')

                    # Astarvi画图
                    handles, labels = ax_Astar_Viterbi.get_legend_handles_labels()
                    by_label = OrderedDict(zip(labels, handles))
                    ax_Astar_Viterbi.legend(by_label.values(), by_label.keys(), bbox_to_anchor=(0, 1), loc='upper left',
                                            borderaxespad=0,
                                            fontsize=6,
                                            frameon=False, ncol=1)
                    # ax_Astar_Viterbi.set_title('A* JPF-Viterbi \u03BB = {}'.format(ratio), fontsize=10, pad=10, loc='center')
                    ax_Astar_Viterbi.axis('equal')
                    f_Astar_Viterbi.savefig(f'Astarvi \u03BB={ratio}下的路径.jpg', dpi=1080, format='jpg',
                                            bbox_inches='tight')

                    # ori画图
                    handles, labels = ax_ori.get_legend_handles_labels()
                    by_label = OrderedDict(zip(labels, handles))
                    ax_ori.legend(by_label.values(), by_label.keys(), bbox_to_anchor=(0, 1), loc='upper left',
                                  borderaxespad=0,
                                  fontsize=6,
                                  frameon=False, ncol=1)
                    # ax_ori.set_title('\u03BB={}'.format(ratio), fontsize=10, pad=10, loc='center')
                    ax_ori.axis('equal')
                    f_ori.savefig(f'ori \u03BB={ratio}下的路径.jpg', dpi=1080, format='jpg', bbox_inches='tight')

    runtime += 1
    print("Progress:", runtime/RunTime*100, "%\n\n\n")

'''
index = 0
for path in Dij_VierbiPath:
    flag = 0
    for band in map.Phase_use_list[path[0]][path[1]]:
        if band == BandUseDij_Viterbi[index]:
            flag = 1
    index += 1
    if flag == 0:
        print("fail")
        break
'''


'''
#Dijvi画图
handles, labels = ax_Dij_Viterbi.get_legend_handles_labels()
by_label = OrderedDict(zip(labels, handles))
ax_Dij_Viterbi.legend(by_label.values(), by_label.keys(), bbox_to_anchor=(0, 1), loc=3, borderaxespad=0, fontsize=6,
                frameon=False, ncol=6)
ax_Dij_Viterbi.set_title('Dijvi \u03B1 = {}'.format(ratio), fontsize=10, pad=10, loc='center')
ax_Dij_Viterbi.axis('equal')
f_Dij_Viterbi.savefig(f'\u03B1={ratio}下的路径.eps', dpi=1080, format='pdf', bbox_inches='tight')
'''


List_Dijvi = [PathLength_Dijvi, PathCost_Dijvi, BandChange_Dijvi, OpenPointNum_Dijvi, TimeCostList_Dijvi]
List_Dij = [PathLength_Dij, PathCost_Dij, BandChange_Dij, OpenPointNum_Dij, TimeCostList_Dij]
List_Astarvi = [PathLength_Astarvi, PathCost_Astarvi, BandChange_Astarvi, OpenPointNum_Astarvi, TimeCostList_Astarvi]
List_Astar = [PathLength_Astar, PathCost_Astar, BandChange_Astar, OpenPointNum_Astar, TimeCostList_Astar]
List_ori = [PathLength_ori, PathCost_ori, BandChange_ori, OpenPointNum_ori, TimeCostList_ori]

List_Single = copy.deepcopy([List_Dijvi, List_Dij, List_Astarvi, List_Astar, List_ori])
List_Total = []
for List in List_Single:
    TotalList = []
    for parameterList in List:
        TmpTotalList = [0]*len(Ratio)
        for index in range(len(parameterList)):
            AddIndex = index % len(Ratio)
            TmpTotalList[AddIndex] += parameterList[index]/RunTime
        TotalList.append(TmpTotalList)
    List_Total.append(TotalList)
np.save('List_Total.npy', List_Total)



def zone_and_linked(ax, axins, zone_left, zone_right, x, y, linked='bottom',
                    x_ratio=0.05, y_ratio=1):
    """缩放内嵌图形，并且进行连线
    ax:         调用plt.subplots返回的画布。例如： fig,ax = plt.subplots(1,1)
    axins:      内嵌图的画布。 例如 axins = ax.inset_axes((0.4,0.1,0.4,0.3))
    zone_left:  要放大区域的横坐标左端点
    zone_right: 要放大区域的横坐标右端点
    x:          X轴标签
    y:          列表，所有y值
    linked:     进行连线的位置，{'bottom','top','left','right'}
    x_ratio:    X轴缩放比例
    y_ratio:    Y轴缩放比例
    """
    xlim_left = x[zone_left] - (x[zone_right] - x[zone_left]) * x_ratio
    xlim_right = x[zone_right] + (x[zone_right] - x[zone_left]) * x_ratio

    y_data = np.hstack([yi[zone_left:zone_right] for yi in y])
    ylim_bottom = np.min(y_data) - (np.max(y_data) - np.min(y_data)) * y_ratio
    ylim_top = np.max(y_data) + (np.max(y_data) - np.min(y_data)) * y_ratio

    axins.set_xlim(xlim_left, xlim_right)
    axins.set_ylim(ylim_bottom, ylim_top)

    ax.plot([xlim_left, xlim_right, xlim_right, xlim_left, xlim_left],
            [ylim_bottom, ylim_bottom, ylim_top, ylim_top, ylim_bottom], "black")

    if linked == 'bottom':
        xyA_1, xyB_1 = (xlim_left, ylim_top), (xlim_left, ylim_bottom)
        xyA_2, xyB_2 = (xlim_right, ylim_top), (xlim_right, ylim_bottom)
    elif linked == 'top':
        xyA_1, xyB_1 = (xlim_left, ylim_bottom), (xlim_left, ylim_top)
        xyA_2, xyB_2 = (xlim_right, ylim_bottom), (xlim_right, ylim_top)
    elif linked == 'left':
        xyA_1, xyB_1 = (xlim_right, ylim_top), (xlim_left, ylim_top)
        xyA_2, xyB_2 = (xlim_right, ylim_bottom), (xlim_left, ylim_bottom)
    elif linked == 'right':
        xyA_1, xyB_1 = (xlim_left, ylim_top), (xlim_right, ylim_top)
        xyA_2, xyB_2 = (xlim_left, ylim_bottom), (xlim_right, ylim_bottom)

    con = ConnectionPatch(xyA=xyA_1, xyB=xyB_1, coordsA="data",
                          coordsB="data", axesA=axins, axesB=ax)
    axins.add_artist(con)
    con = ConnectionPatch(xyA=xyA_2, xyB=xyB_2, coordsA="data",
                          coordsB="data", axesA=axins, axesB=ax)
    axins.add_artist(con)


List_Total = np.load('List_Total.npy', allow_pickle=True)
print(List_Total)
List_y = ['路径长度/km', '总能耗/J', '次数', '个数', '时间/s']
List_x = ['路径步数', '路径代价', '频段切换次数', '扫描点个数', '时间/s']
List_Name = ['路径步数.jpg', '路径代价.jpg', '频段切换次数.jpg', '扫描点个数.jpg', '消耗时间.jpg']
markerList = ['^', 'p', 's', 'h', 'o']
colorList = ['r', 'g', 'y', 'blue', 'black']
labelList = [TypeDij_vi, TypeDij_Dp, TypeAstar_vi, TypeAstar_Dp, "分离算法"]
Index = 0
for list_y, list_x, list_name in zip(List_y, List_x, List_Name):
    figPathTotal = plt.figure()
    axPathTotal = figPathTotal.add_subplot(111)
    for index in range(len(List_Total)):
        axPathTotal.plot(Ratio, List_Total[index][Index], marker=markerList[index], markersize=3, color=colorList[index], label=labelList[index], linestyle='--', alpha=0.8)
    if list_name == '路径代价.jpg':
        axins = axPathTotal.inset_axes((0.1, 0.7, 0.3, 0.2))
        y_List = []
        for index in range(len(List_Total)):
            axins.plot(Ratio, List_Total[index][Index], marker=markerList[index], markersize=3, color=colorList[index], label=labelList[index], linestyle='--', alpha=0.8)
            if index != len(List_Total) - 1:
                y_List.append(List_Total[index][Index])
        zone_and_linked(axPathTotal, axins, 6, 8, Ratio, y_List, 'top')
    axPathTotal.set(xlabel='\u03BB', ylabel=list_y)
    axPathTotal.legend(bbox_to_anchor=(1.01, 1.0), prop={'size': 7})
    axPathTotal.relim()
    figPathTotal.savefig(list_name, dpi=1080, format='jpg', bbox_inches='tight')
    Index += 1


'''
figOpenPointNumRatio = plt.figure()
axOpenPointNumRatio = figOpenPointNumRatio.add_subplot(111)
axOpenPointNumRatio.plot(Ratio, OpenPointNumRatio, marker='^', color='r', label='扫描点比值', linestyle='--')
axOpenPointNumRatio.plot(Ratio, CostRatio, marker='s', color='g', label='代价比值', linestyle='-.')
axOpenPointNumRatio.set(title='Astar,Dij扫描点与代价比值', xlabel='\u03B1')
figPathTotal.savefig('Dijvi与Dij扫描点比值', dpi=1080, format='pdf', bbox_inches='tight')
'''



plt.tight_layout()
# plt.show()
print("done")