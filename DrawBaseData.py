import matplotlib.pyplot as plt
import numpy as np


def Draw():
    List_Total = np.load('List_Total.npy', allow_pickle=True)
    Ratio = [i*1.5 for i in range(1, 11)]
    TypeDij_Dp = "Dij JSPD-DP"
    TypeAstar_Dp = "A* JSPD-DP"
    TypeDij_vi = "Dij JSPD-Viterbi"
    TypeAstar_vi = "A* JSPD-Viterbi"
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
            axPathTotal.plot(Ratio, List_Total[index][Index], marker=markerList[index], markersize=3, color=colorList[index], label=labelList[index], linestyle='--', alpha=1)
        axPathTotal.set(xlabel='\u03BB', ylabel=list_y)
        axPathTotal.legend(bbox_to_anchor=(1.01, 1.0), prop={'size': 7})
        # axPathTotal.relim()
        axPathTotal.set_xticks(Ratio)
        axPathTotal.set_xlim(1, 15.5)
        axPathTotal.grid(color='orange', linestyle='--', linewidth=0.3)
        figPathTotal.savefig(list_name, dpi=1080, format='jpg', bbox_inches='tight')
        Index += 1

    plt.show()
    return
