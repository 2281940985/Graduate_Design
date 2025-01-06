import sys
class Point:
    def __init__(self, x, y, UserSeq, Power):
        self.x = x
        self.y = y
        self.UserNum = UserSeq
        self.Power = Power
        self.time = -1
        self.cost = sys.maxsize
        self.band = -1

    def __eq__(self, other):
        return self.UserNum == self.UserNum

    def __hash__(self):
        return hash(self.UserNum)

class Solution:
    def __init__(self, cost, path, path_band):
        self.cost = cost
        self.path = path
        self.path_band = path_band
