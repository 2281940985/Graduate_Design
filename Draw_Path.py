from collections import OrderedDict
import os
import numpy as np
from matplotlib import pyplot as plt
from matplotlib.patches import Rectangle
import gc
def minimize_frequency_switches_with_frequencis(map, path):
    '''
    DP分配最小切换次数的接入频段
    '''
    N = len(path)
    # if N == 100:
    #     print(N)
    INF = float('inf')
    PhaseNum = 10
    # 初始化DP表
    dp = [[INF for _ in range(PhaseNum)] for _ in range(N)]
    dp[0] = [0] * PhaseNum

    # 初始化频段分配列表
    assigned_frequencies = [[0 for _ in range(PhaseNum)] for _ in range(N)]
    assigned_frequencies[0] = list(range(PhaseNum))

    for i in range(1, N):#dp[i][j] 表示路径到第 i 个点时，使用第 j 个频段的最小切换次数。
        for j in range(len(map.Phase_use_list[path[i][0]][path[i][1]])):#遍历第i个点的所有可用频段
            for k in range(len(map.Phase_use_list[path[i - 1][0]][path[i - 1][1]])):#k 表示前一个点的频段（遍历前一个点的所有可用频段），索引
                # 找到前一个点到当前点的最小切换频段次数
                if map.Phase_use_list[path[i][0]][path[i][1]][j] != map.Phase_use_list[path[i - 1][0]][path[i - 1][1]][k]:#前一个点与当前点不在一个频段
                    if dp[i][j] > dp[i - 1][k] + 1:
                        dp[i][j] = dp[i - 1][k] + 1
                        assigned_frequencies[i][j] = k#记录每个点在路径上使用的最优频段的"来源频段"的索引即上一个频段，要到具体该点的Phase_use_list中查找具体的频段，j也是频段的索引
                else:
                    if dp[i][j] > dp[i - 1][k]:
                        dp[i][j] = dp[i - 1][k]
                        assigned_frequencies[i][j] = k

    # 找到最后一个点的最小切换频段次数，最后一个点就是终点
    min_switches = min(dp[N - 1])#dp[N-1]表示路径中最后一个点的所有频段的切换次数。

    # 找到最后一个点的最优频段
    min_index_list = [index for index, value in enumerate(dp[N-1]) if value == min_switches]#min_index_list存储最后1个点的所有最小频段切换次数的位置的索引
    last_point_frequency = []
    for min_index in min_index_list:
        last_point_frequency.append(map.Phase_use_list[path[N - 1][0]][path[N - 1][1]][min_index])#存储最后一个点可用的最小切换次数的频段。

    # 找到每个点的频段分配
    frequency_assignment = [last_point_frequency[0]]
    min_index = min_index_list[0]
    for i in range(N - 2, -1, -1):
        # if N == 100:
        #     print(N)#assigned_frequencies[i + 1][min_index] 表示在第 i+1 个点时，选择频段 min_index 对应的上一个点的频段。
        min_index = assigned_frequencies[i + 1][min_index]
        frequency_assignment.append(map.Phase_use_list[path[i][0]][path[i][1]][min_index])

    frequency_assignment.reverse()
    pathPhase = []
    for p in path:
        pathPhase.append(map.Phase_use_list[p[0]][p[1]])

    PhaseList = []#记录频段在何时发生切换,PhaseList[i][0]代表所用什么频段，PhaseList[i][1]代表在path中发生频段切换的序列
    for p in range(len(frequency_assignment)):
        if p > 1 and frequency_assignment[p] != frequency_assignment[p - 1]:
            PhaseList.append([frequency_assignment[p - 1], p])
        if p == len(frequency_assignment) - 1:
            PhaseList.append([frequency_assignment[p], p])
    '''
    print('\n')
    print('##########')
    print('path:', path)
    print('pathPhase:', pathPhase)
    print('phase:', frequency_assignment)
    print('MinChange:', min_switches)
    print('PhaseList', PhaseList)
    print('##########')
    print('\n')
    '''


    return min_switches, PhaseList, last_point_frequency  # frequency_assignment




def MinChangePhase(map, path, num):
    change_Phase = 0
    temp_Phase = [0, 1, 2, 3, 4, 5, 6, 7, 8, 9]
    Phase_list = []
    Phase_list_User_use = []
    for p in range(len(path)):
        temp_Phase_temp = temp_Phase
        temp_Phase = list(set(temp_Phase) & set(map.Phase_use_list[path[p][0]][path[p][1]]))
        if len(temp_Phase) == 0:
            change_Phase += 1
            temp_Phase = map.Phase_use_list[path[p][0]][path[p][1]]
            Phase_list.append(temp_Phase)
            Phase_list_User_use.append([temp_Phase_temp, p])
        else:
            if p == len(path) - 1:
                Phase_list_User_use.append([temp_Phase, p])
            Phase_list.append(temp_Phase)
    print('切换频段次数：', change_Phase)
    if num == 1:
        return Phase_list_User_use
    else:
        return change_Phase


def BandChangeNum(BandChange):
    ChangeNum = 0
    for i in range(1, len(BandChange)):
        if BandChange[i - 1] != BandChange[i]:
            if i == 1 and BandChange[i - 1] == 0:
                continue
            ChangeNum += 1

    return ChangeNum


from matplotlib.patches import Rectangle

def DrawAxBase(map, ax):
    for i in range(map.size):  # 遍历地图的每个单元格
        for j in range(map.size):
            rec_pa = Rectangle((i - 0.5, j - 0.5), 1, 1, edgecolor='gray', facecolor='w')  # 灰色边框，白色填充
            ax.add_patch(rec_pa)  # 绘制普通单元格
            if map.IsObstacle(i, j, map.obstacle_point):  # 障碍物：灰色
                rec_ob = Rectangle((i - 0.5, j - 0.5), 1, 1, facecolor='gray', label='障碍物')
                ax.add_patch(rec_ob)
            if map.IsObstacle(i, j, map.Phase_obstacle):  # 全频段拥塞：黑色
                rec_ob = Rectangle((i - 0.5, j - 0.5), 1, 1, facecolor='black', label='全频段拥塞')
                ax.add_patch(rec_ob)

    # 定义 3x3 网格的中心和范围
    x_center, y_center = 65, 65
    x_min, x_max = x_center - 1, x_center + 1
    y_min, y_max = y_center - 1, y_center + 1

    # 绘制红色虚线边框
    coverage_box = Rectangle(
        (x_min - 0.5, y_min - 0.5),  # 左下角坐标
        x_max - x_min + 1,  # 宽度
        y_max - y_min + 1,  # 高度
        edgecolor='r', facecolor='none', linestyle='dashed', linewidth=1, label='覆盖区域'
    )
    ax.add_patch(coverage_box)  # 将红色虚线边框添加到图中

    # 添加图例
    ax.legend(loc='upper right')

    return ax



def Astar_DrawPath(map, path, band, ax, Mod):

    '''
    for i in range(map.size):  # 根据障碍物生成地图
        for j in range(map.size):
            rec_pa = Rectangle((i - 0.5, j - 0.5), 1, 1, edgecolor='gray', facecolor='w')
            ax.add_patch(rec_pa)
            if map.IsObstacle(i, j, map.obstacle_point):
                rec_ob = Rectangle((i - 0.5, j - 0.5), 1, 1, facecolor='gray', label='障碍物')
                ax.add_patch(rec_ob)
            if map.IsObstacle(i, j, map.Phase_obstacle):
                rec_ob = Rectangle((i - 0.5, j - 0.5), 1, 1, facecolor='black', label='全频段拥塞')
                ax.add_patch(rec_ob)
    '''

    # Phase_list_User_use = MinChangePhase(map, path, 1)
    # Minchange, Phase_list_User_use, straightNum, diagNum = minimize_frequency_switches_with_frequencies(map, path)
    # if ax == 0:
    #    return Minchange, Phase_list_User_use, straightNum, diagNum

    if Mod == 1:
        Phase_list_User_use = []
        for i in range(1, len(band)):
            if i == len(band) - 1:
                Phase_list_User_use.append([band[i], i])
            elif band[i] != band[i - 1]:
                Phase_list_User_use.append([band[i - 1], i])
    else:
        Phase_list_User_use = band

    if ax == 0:
        return ax, Phase_list_User_use

    Index = len(Phase_list_User_use)
    # print(Index)
    x_path = [[] for i in range(Index)]
    y_path = [[] for i in range(Index)]
    phasePath = [[] for i in range(Index)]
    II = 0
    for i in range(len(path)):
        if i <= Phase_list_User_use[II][1]:
            x_path[II].append(path[i][0])
            y_path[II].append(path[i][1])
            phasePath[II].append([path[i][0], path[i][1]])

        else:
            x_path[II].append(path[i][0])
            y_path[II].append(path[i][1])
            phasePath[II].append([path[i][0], path[i][1]])

            if II != Index - 1:
                II += 1
            x_path[II].append(path[i][0])
            y_path[II].append(path[i][1])
            phasePath[II].append([path[i][0], path[i][1]])

    # print('x=', x_path)
    # print('y=', y_path)


    # DrawPhasePathMap(x_path, y_path, band, map)
    for i in range(Index):
        temp_list = Phase_list_User_use[i][0]
        if temp_list == 0:
            ax.plot(x_path[i], y_path[i], linewidth=1.5, color='black', label='频段中断')
        elif temp_list == 1:
            ax.plot(x_path[i], y_path[i], linewidth=1.5, color='red', label='频段1')
        elif temp_list == 2:
            ax.plot(x_path[i], y_path[i], linewidth=1.5, color='orange', label='频段2')
        elif temp_list == 3:
            ax.plot(x_path[i], y_path[i], linewidth=1.5, color='yellow', label='频段3')
        elif temp_list == 4:
            ax.plot(x_path[i], y_path[i], linewidth=1.5, color='green', label='频段4')
        elif temp_list == 5:
            ax.plot(x_path[i], y_path[i], linewidth=1.5, color='blue', label='频段5')
        elif temp_list == 6:
            ax.plot(x_path[i], y_path[i], linewidth=1.5, color='cyan', label='频段6')
        elif temp_list == 7:
            ax.plot(x_path[i], y_path[i], linewidth=1.5, color='purple', label='频段7')
        elif temp_list == 8:
            ax.plot(x_path[i], y_path[i], linewidth=1.5, color='pink', label='频段8')
        else:
            ax.plot(x_path[i], y_path[i], linewidth=1.5, color='brown', label='频段9')

    return ax, Phase_list_User_use, x_path, y_path


def DrawPhasePathMap(x_path, y_path, band, name):
    plt.rcParams['font.family'] = 'SimHei'  # 设置中文字体

    for i in range(len(band)):
        try:
            data = np.load(f"Phase{band[i][0]}.npy", allow_pickle=True)
        except Exception as e:
            print(f"Draw_path_249line_加载文件失败: {e}")
            return

        # 处理NaN和无穷大值
        data = np.nan_to_num(data, nan=0, posinf=np.max(data[np.isfinite(data)]),
                             neginf=np.min(data[np.isfinite(data)]))

        # 创建图形，调整尺寸
        fig = plt.figure(figsize=(10, 10))
        ax = fig.add_subplot(111)

        # 转置数据
        data = data.T
        heatmap = ax.imshow(data, cmap='jet', interpolation='nearest', origin='lower',
                            extent=[0, data.shape[1], 0, data.shape[0]],
                            vmin=np.min(data), vmax=np.max(data))#jet -> inferno/hot

        # 添加颜色条
        cbar = plt.colorbar(heatmap, ax=ax, fraction=0.046, pad=0.04)
        cbar.set_label('SINR Strength', fontsize=14)

        ax.set_xlim(-13, 83)  #
        ax.set_ylim(-3, 73)  #
        # 设置坐标轴
        # ax.set_xlabel('X axis (0-69)', fontsize=12)
        # ax.set_ylabel('Y axis (0-69)', fontsize=12)
        ax.set_xticks(np.arange(0.5, data.shape[1] + 1, 10))
        ax.set_yticks(np.arange(0.5, data.shape[0] + 1, 10))
        ax.set_xticklabels(np.arange(0, data.shape[1]+1, 10))  # 标签为离散值
        ax.set_yticklabels(np.arange(0, data.shape[0] + 1, 10))  # 标签为离散值

        # 添加网格线
        # ax.grid(True, color='white', linestyle='--', linewidth=0.5)
        ax.grid(False)

        # 调整路径坐标
        x_centered = [x + 0.5 for x in x_path[i]]
        y_centered = [y + 0.5 for y in y_path[i]]

        # 绘制路径
        ax.plot(x_centered, y_centered, color='black', linewidth=1.5, label='轨迹')

        # 标记起点和终点
        ax.scatter(x_centered[0], y_centered[0], color='purple', s=20, marker='s', label='起点')
        ax.scatter(x_centered[-1], y_centered[-1], color='red', s=20, marker='s', label='终点')

        # 将图例放在图像内的左上角空白区域
        ax.legend(loc='upper left', bbox_to_anchor=(0, 1),
                 fontsize=8, frameon=True, borderpad=0.5,
                 labelspacing=0.5, handlelength=2.0)

        # 强制刷新画布，防止图例溢出
        fig.canvas.draw()

        # 添加标题
        plt.title(f"Phase {band[i][0]}_{i} Map with Path", fontsize=14, pad=20)

        # 创建存储路径
        save_path = rf"E:\Graduate_Design\ZuHao_Code\test_SingleUser\热力图\{name}"
        os.makedirs(save_path, exist_ok=True)

        # 保存图片
        plt.savefig(os.path.join(save_path, f"{name}_{i}.png"), dpi=800, bbox_inches='tight')

        # 显示并释放资源
        plt.show()
        plt.close(fig)
        del data
        gc.collect()