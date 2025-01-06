import numpy as np

def CheckPathCost(path, ChangeNum, ratio):
    cost = 0
    PathLen = len(path)
    for i in range(1, PathLen):
        if abs(path[i][0] + path[i][1] - path[i - 1][0] - path[i - 1][1]) == 1:
            cost += 1
        else:
            cost += np.sqrt(2)
    cost += ChangeNum * ratio
    return cost*10


def CheckPathBand(path, bandlist):

    return True