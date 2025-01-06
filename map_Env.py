import numpy as np
import pickle
import point_solution_class as point
import math
import sys

class RandomMap:
    def __init__(self, start_point_list, end_point_list, Bs_Power, LoadBaseOb, PowerChange):
        self.size = 70  # 10m
        self.obsacle = self.size * self.size * 0.20
        self.start_point_list = start_point_list
        self.end_point_list = end_point_list
        self.Bs_Gain = 10  # dB
        self.Bs = [self.size/2, self.size/2]
        self.Phase_Gain = 2.6

        self.Freq1 = 1 * pow(10, 9)
        self.Freq2 = 1.1 * pow(10, 9)
        self.Freq3 = 1.2 * pow(10, 9)
        self.Freq4 = 1.3 * pow(10, 9)
        self.Freq5 = 1.4 * pow(10, 9)
        self.Freq6 = 1.5 * pow(10, 9)
        self.Freq7 = 1.6 * pow(10, 9)
        self.Freq8 = 1.7 * pow(10, 9)
        self.Freq9 = 1.8 * pow(10, 9)

        PreObNum = 26
        self.GenerateObstacle(start_point_list, end_point_list, Bs_Power, LoadBaseOb, PowerChange, PreObNum)

    #初始化网格区域所有点，每个点传入用户序列 UserSeq 和功率 Power。初始化每个点的cost，time，与band
    def Reset_pointInfo(self, UserSeq, Power):
        for x in range(self.size):
            for y in range(self.size):
                p = point.Point(x, y, UserSeq, Power)
                p.cost = sys.maxsize
                p.time = -1
                p.band = -1

    def Calculate_Rss(self, Phase_obstacle, x, y): #计算障碍物对该点（x,y）的干扰，信号强度与障碍物的功率 p[2] 成正比，与障碍物到目标点的距离平方成反比。
        Rss = 0
        for p in Phase_obstacle:
            if(p[0] == x and p[1] == y): return float('inf')
            Rss += p[2]/(pow((p[0] - x), 2) + pow((p[1] - y), 2))#干扰信号强度与障碍物的功率 p[2] 成正比，与障碍物到目标点的距离平方成反比。
        return Rss



    def Bs_channel_gain(self, x, y, Fc):#计算基站对目标点（x,y）的信道增益，计算目标点与基站之间的距离和频率（）的对数损耗。
        if x == self.Bs[0] and y == self.Bs[1]:
            Pl = -float('inf')
        else:
            Pl = 32.45 + 20*math.log(math.sqrt(pow(x - self.Bs[0], 2) + pow(y - self.Bs[1], 2)), 10) + 20 * math.log(Fc, 10)
        return self.Bs_Gain - Pl

    def Phase_obstacle_channel_gain(self, point, x, y, Fc):
        if x == point[0] and y == point[1]:
            Pl = -float('inf')
        else:
            Pl = 32.45 + 20*math.log(math.sqrt(pow(x - point[0], 2) + pow(y - point[1], 2)), 10) + 20 * math.log(Fc, 10)
        return self.Phase_Gain - Pl

    def Calculate_SINR(self, Phase_obstacle_point, x, y, Fc, Bs_Power):
        Noise = 0
        for p in Phase_obstacle_point:
            if p[0] == x and p[1] == y:
                return -float('inf')
            #计算每个障碍物对目标点的噪声贡献，并累加到 Noise 中
            Noise += pow(pow(10, self.Phase_obstacle_channel_gain(p, x, y, Fc)/10), 2) * p[2]*1000
        Noise_dB = 10 * math.log(Noise, 10)
        #计算信号强度
        S = 10 * math.log(Bs_Power * pow(10, 3), 10) + 2 * self.Bs_channel_gain(x, y, Fc)
        # print(S - Noise_dB)
        return S - Noise_dB

    def GenerateObstacle(self, start_point, end_point, Bs_Power, LoadBaseOb, PowerChange, PreObNum):
        self.obstacle_point = []
        self.obstacle_coordinate = []
        self.Phase_obstacle = []

        self.temp_obstacle_coordinate = []
        self.move_obstacle = []

        self.Phase_obstacle1_point = []
        self.Phase_obstacle2_point = []
        self.Phase_obstacle3_point = []
        self.Phase_obstacle4_point = []
        self.Phase_obstacle5_point = []
        self.Phase_obstacle6_point = []
        self.Phase_obstacle7_point = []
        self.Phase_obstacle8_point = []
        self.Phase_obstacle9_point = []

        self.Phase_obstacle1 = []
        self.Phase_obstacle2 = []
        self.Phase_obstacle3 = []
        self.Phase_obstacle4 = []
        self.Phase_obstacle5 = []
        self.Phase_obstacle6 = []
        self.Phase_obstacle7 = []
        self.Phase_obstacle8 = []
        self.Phase_obstacle9 = []

        self.Phase1 = [[[]for i in range(self.size)]for j in range(self.size)]
        self.Phase2 = [[[]for i in range(self.size)]for j in range(self.size)]
        self.Phase3 = [[[]for i in range(self.size)]for j in range(self.size)]
        self.Phase4 = [[[]for i in range(self.size)]for j in range(self.size)]
        self.Phase5 = [[[]for i in range(self.size)]for j in range(self.size)]
        self.Phase6 = [[[]for i in range(self.size)]for j in range(self.size)]
        self.Phase7 = [[[]for i in range(self.size)]for j in range(self.size)]
        self.Phase8 = [[[]for i in range(self.size)]for j in range(self.size)]
        self.Phase9 = [[[]for i in range(self.size)]for j in range(self.size)]

        self.Phase_use_list = [[[]for i in range(self.size)]for j in range(self.size)]

        # 随机频段障碍

        if not LoadBaseOb:
            '''
            mean = self.size/2
            std_dev = self.size/15
            minX = 0; maxX = self.size
            minY = 0; maxY = self.size
            '''
            State_x = [[0, 20], [10, 30], [25, 45], [40, 60], [50, 70]]  # 调整了地图中频段干扰源出现位置的比率，越靠近基站干扰源数目越多
            State_y = [[0, 20], [10, 30], [25, 45], [40, 60], [50, 70]]
            Rate_Matrix = [[0.02, 0.04, 0.06, 0.04, 0.02],
                           [0.04, 0.08, 0.1, 0.08, 0.04],
                           [0.06, 0.1, 0.8, 0.1, 0.06],
                           [0.04, 0.08, 0.1, 0.08, 0.04],
                           [0.02, 0.04, 0.06, 0.04, 0.02]]
            for i in range(len(State_x)):
                for j in range(len(State_y)):
                    for _ in range(round(PreObNum * Rate_Matrix[i][j])):
                        x = np.random.randint(State_x[i][0], State_x[i][1])
                        y = np.random.randint(State_y[j][0], State_y[j][1])
                        Pow = 0.1
                        self.Phase_obstacle1_point.append([x, y, Pow])

                    for _ in range(round(PreObNum * Rate_Matrix[i][j])):
                        x = np.random.randint(State_x[i][0], State_x[i][1])
                        y = np.random.randint(State_y[j][0], State_y[j][1])
                        Pow = 0.1
                        self.Phase_obstacle2_point.append([x, y, Pow])

                    for _ in range(round(PreObNum * Rate_Matrix[i][j])):
                        x = np.random.randint(State_x[i][0], State_x[i][1])
                        y = np.random.randint(State_y[j][0], State_y[j][1])
                        Pow = 0.1
                        self.Phase_obstacle3_point.append([x, y, Pow])

                    for _ in range(round(PreObNum * Rate_Matrix[i][j])):
                        x = np.random.randint(State_x[i][0], State_x[i][1])
                        y = np.random.randint(State_y[j][0], State_y[j][1])
                        Pow = 0.1
                        self.Phase_obstacle4_point.append([x, y, Pow])

                    for _ in range(round(PreObNum * Rate_Matrix[i][j])):
                        x = np.random.randint(State_x[i][0], State_x[i][1])
                        y = np.random.randint(State_y[j][0], State_y[j][1])
                        Pow = 0.1
                        self.Phase_obstacle5_point.append([x, y, Pow])

                    for _ in range(round(PreObNum * Rate_Matrix[i][j])):
                        x = np.random.randint(State_x[i][0], State_x[i][1])
                        y = np.random.randint(State_y[j][0], State_y[j][1])
                        Pow = 0.1
                        self.Phase_obstacle6_point.append([x, y, Pow])

                    for _ in range(round(PreObNum * Rate_Matrix[i][j])):
                        x = np.random.randint(State_x[i][0], State_x[i][1])
                        y = np.random.randint(State_y[j][0], State_y[j][1])
                        Pow = 0.1
                        self.Phase_obstacle7_point.append([x, y, Pow])

                    for _ in range(round(PreObNum * Rate_Matrix[i][j])):
                        x = np.random.randint(State_x[i][0], State_x[i][1])
                        y = np.random.randint(State_y[j][0], State_y[j][1])
                        Pow = 0.1
                        self.Phase_obstacle8_point.append([x, y, Pow])

                    for _ in range(round(PreObNum * Rate_Matrix[i][j])):
                        x = np.random.randint(State_x[i][0], State_x[i][1])
                        y = np.random.randint(State_y[j][0], State_y[j][1])
                        Pow = 0.1
                        self.Phase_obstacle9_point.append([x, y, Pow])

            ObNum = len(self.Phase_obstacle1_point)
            self.Phase_obstacle1_num = ObNum
            self.Phase_obstacle2_num = ObNum
            self.Phase_obstacle3_num = ObNum
            self.Phase_obstacle4_num = ObNum
            self.Phase_obstacle5_num = ObNum
            self.Phase_obstacle6_num = ObNum
            self.Phase_obstacle7_num = ObNum
            self.Phase_obstacle8_num = ObNum
            self.Phase_obstacle9_num = ObNum

            '''
            for i in range(self.Phase_obstacle1_num):
                x = np.clip(np.random.normal(loc=mean, scale=std_dev), minX, maxX)
                y = np.clip(np.random.normal(loc=mean, scale=std_dev), minY, maxY)
                # x = np.random.uniform(minX, maxX)
                # y = np.random.uniform(minY, maxY)
                Pow = np.random.uniform(4, 5) * 0.2
                self.Phase_obstacle1_point.append([x, y, Pow])

            for i in range(self.Phase_obstacle2_num):
                x = np.clip(np.random.normal(loc=mean, scale=std_dev), minX, maxX)
                y = np.clip(np.random.normal(loc=mean, scale=std_dev), minY, maxY)
                # x = np.random.uniform(minX, maxX)
                # y = np.random.uniform(minY, maxY)
                Pow = np.random.uniform(4, 5) * 0.2
                self.Phase_obstacle2_point.append([x, y, Pow])

            for i in range(self.Phase_obstacle3_num):
                x = np.clip(np.random.normal(loc=mean, scale=std_dev), minX, maxX)
                y = np.clip(np.random.normal(loc=mean, scale=std_dev), minY, maxY)
                # x = np.random.uniform(minX, maxX)
                # y = np.random.uniform(minY, maxY)
                Pow = np.random.uniform(4, 5) * 0.2
                self.Phase_obstacle3_point.append([x, y, Pow])

            for i in range(self.Phase_obstacle4_num):
                x = np.clip(np.random.normal(loc=mean, scale=std_dev), minX, maxX)
                y = np.clip(np.random.normal(loc=mean, scale=std_dev), minY, maxY)
                # x = np.random.uniform(minX, maxX)
                # y = np.random.uniform(minY, maxY)
                Pow = np.random.uniform(4, 5) * 0.2
                self.Phase_obstacle4_point.append([x, y, Pow])

            for i in range(self.Phase_obstacle5_num):
                x = np.clip(np.random.normal(loc=mean, scale=std_dev), minX, maxX)
                y = np.clip(np.random.normal(loc=mean, scale=std_dev), minY, maxY)
                # x = np.random.uniform(minX, maxX)
                # y = np.random.uniform(minY, maxY)
                Pow = np.random.uniform(4, 5) * 0.2
                self.Phase_obstacle5_point.append([x, y, Pow])

            for i in range(self.Phase_obstacle6_num):
                x = np.clip(np.random.normal(loc=mean, scale=std_dev), minX, maxX)
                y = np.clip(np.random.normal(loc=mean, scale=std_dev), minY, maxY)
                # x = np.random.uniform(minX, maxX)
                # y = np.random.uniform(minY, maxY)
                Pow = np.random.uniform(4, 5) * 0.2
                self.Phase_obstacle6_point.append([x, y, Pow])

            for i in range(self.Phase_obstacle7_num):
                x = np.clip(np.random.normal(loc=mean, scale=std_dev), minX, maxX)
                y = np.clip(np.random.normal(loc=mean, scale=std_dev), minY, maxY)
                # x = np.random.uniform(minX, maxX)
                # y = np.random.uniform(minY, maxY)
                Pow = np.random.uniform(4, 5) * 0.2
                self.Phase_obstacle7_point.append([x, y, Pow])

            for i in range(self.Phase_obstacle8_num):
                x = np.clip(np.random.normal(loc=mean, scale=std_dev), minX, maxX)
                y = np.clip(np.random.normal(loc=mean, scale=std_dev), minY, maxY)
                # x = np.random.uniform(minX, maxX)
                # y = np.random.uniform(minY, maxY)
                Pow = np.random.uniform(4, 5) * 0.2
                self.Phase_obstacle8_point.append([x, y, Pow])

            for i in range(self.Phase_obstacle9_num):
                x = np.clip(np.random.normal(loc=mean, scale=std_dev), minX, maxX)
                y = np.clip(np.random.normal(loc=mean, scale=std_dev), minY, maxY)
                x = np.random.uniform(minX, maxX)
                # y = np.random.uniform(minY, maxY)
                Pow = np.random.uniform(4, 5) * 0.2
                self.Phase_obstacle9_point.append([x, y, Pow])
            '''
            # 随机现实障碍
            i = 0
            while i != self.obsacle - 1:
                x = np.random.randint(0, self.size)
                y = np.random.randint(0, self.size)
                temp = True
                for start_point, end_point in zip(self.start_point_list, self.end_point_list):
                    if (start_point.x == x and start_point.y == y) or (end_point.x == x and end_point.y == y):
                        temp = False
                        break
                for p in self.obstacle_point:
                    if (x == p[0] and y == p[1]):
                        temp = False
                        break
                if temp:
                    ob = [x, y]
                    i += 1
                    self.obstacle_point.append([x, y])
                    self.obstacle_coordinate.append(ob)
            np.save('map_obstacle.npy', self.obstacle_point)
            np.save('map_obstacle_coordinate.npy', self.obstacle_coordinate)
            np.save('Phase_obstacle1_point.npy', self.Phase_obstacle1_point)
            np.save('Phase_obstacle2_point.npy', self.Phase_obstacle2_point)
            np.save('Phase_obstacle3_point.npy', self.Phase_obstacle3_point)
            np.save('Phase_obstacle4_point.npy', self.Phase_obstacle4_point)
            np.save('Phase_obstacle5_point.npy', self.Phase_obstacle5_point)
            np.save('Phase_obstacle6_point.npy', self.Phase_obstacle6_point)
            np.save('Phase_obstacle7_point.npy', self.Phase_obstacle7_point)
            np.save('Phase_obstacle8_point.npy', self.Phase_obstacle8_point)
            np.save('Phase_obstacle9_point.npy', self.Phase_obstacle9_point)
        else:
            self.obstacle_point = np.load('map_obstacle.npy', allow_pickle=True)  # 加载确定地图，障碍物以类的形式
            self.obstacle_coordinate = np.load('map_obstacle_coordinate.npy', allow_pickle=True)  # 加载确定地图，障碍物以横纵坐标的形式
            self.Phase_obstacle1_point = np.load('Phase_obstacle1_point.npy', allow_pickle=True)  # 加载确定地图，频段拥塞1
            self.Phase_obstacle2_point = np.load('Phase_obstacle2_point.npy', allow_pickle=True)  # 加载确定地图，频段拥塞2
            self.Phase_obstacle3_point = np.load('Phase_obstacle3_point.npy', allow_pickle=True)  # 加载确定地图，频段拥塞3
            self.Phase_obstacle4_point = np.load('Phase_obstacle4_point.npy', allow_pickle=True)  # 加载确定地图，频段拥塞4
            self.Phase_obstacle5_point = np.load('Phase_obstacle5_point.npy', allow_pickle=True)  # 加载确定地图，频段拥塞5
            self.Phase_obstacle6_point = np.load('Phase_obstacle6_point.npy', allow_pickle=True)  # 加载确定地图，频段拥塞6
            self.Phase_obstacle7_point = np.load('Phase_obstacle7_point.npy', allow_pickle=True)  # 加载确定地图，频段拥塞7
            self.Phase_obstacle8_point = np.load('Phase_obstacle8_point.npy', allow_pickle=True)  # 加载确定地图，频段拥塞8
            self.Phase_obstacle9_point = np.load('Phase_obstacle8_point.npy', allow_pickle=True)  # 加载确定地图，频段拥塞9

        if PowerChange:
            for x in range(self.size):
                for y in range(self.size):
                    Rss_set1 = 3
                    Rss_set2 = 3
                    Rss_set3 = 3
                    Rss_set4 = 3
                    Rss_set5 = 3
                    Rss_set6 = 3
                    Rss_set7 = 3
                    Rss_set8 = 3
                    Rss_set9 = 3
                    Rss1 = self.Calculate_SINR(self.Phase_obstacle1_point, x, y, self.Freq1, Bs_Power)
                    self.Phase1[x][y] = Rss1
                    Rss2 = self.Calculate_SINR(self.Phase_obstacle2_point, x, y, self.Freq2, Bs_Power)
                    self.Phase2[x][y] = Rss2
                    Rss3 = self.Calculate_SINR(self.Phase_obstacle3_point, x, y, self.Freq3, Bs_Power)
                    self.Phase3[x][y] = Rss3
                    Rss4 = self.Calculate_SINR(self.Phase_obstacle4_point, x, y, self.Freq4, Bs_Power)
                    self.Phase4[x][y] = Rss4
                    Rss5 = self.Calculate_SINR(self.Phase_obstacle5_point, x, y, self.Freq5, Bs_Power)
                    self.Phase5[x][y] = Rss5
                    Rss6 = self.Calculate_SINR(self.Phase_obstacle6_point, x, y, self.Freq6, Bs_Power)
                    self.Phase6[x][y] = Rss6
                    Rss7 = self.Calculate_SINR(self.Phase_obstacle7_point, x, y, self.Freq7, Bs_Power)
                    self.Phase7[x][y] = Rss7
                    Rss8 = self.Calculate_SINR(self.Phase_obstacle8_point, x, y, self.Freq8, Bs_Power)
                    self.Phase8[x][y] = Rss8
                    Rss9 = self.Calculate_SINR(self.Phase_obstacle9_point, x, y, self.Freq9, Bs_Power)
                    self.Phase9[x][y] = Rss9
                    if Rss1 < Rss_set1:  # 在多用户地图生成中改成了以方格中心的SINR来代表该方格的SINR，以前写的单用户懒得改了
                        self.Phase_obstacle1.append([x, y])#self.Phase_obstacle1—9是根据self.Phase_obstacle1-9_point计算得来的
                    else: #筛选出频段拥塞的坐标点
                        Rss = 0
                        for i in np.arange(x - 0.35, x + 0.35, 0.05):
                            for j in np.arange(y - 0.35, y + 0.35, 0.7):
                                Rss = max(Rss, self.Calculate_SINR(self.Phase_obstacle1_point, i, j, self.Freq1, Bs_Power))
                        for j in np.arange(y - 0.35, y + 0.35, 0.05):
                            for i in np.arange(x - 0.35, x + 0.35, 0.7):
                                Rss = max(Rss, self.Calculate_SINR(self.Phase_obstacle1_point, i, j, self.Freq1, Bs_Power))
                        if Rss < Rss_set1:
                            self.Phase_obstacle1.append([x, y])
                    if Rss2 < Rss_set2:
                        self.Phase_obstacle2.append([x, y])
                    else:
                        Rss = 0
                        for i in np.arange(x - 0.35, x + 0.35, 0.05):
                            for j in np.arange(y - 0.35, y + 0.35, 0.7):
                                Rss = max(Rss, self.Calculate_SINR(self.Phase_obstacle2_point, i, j, self.Freq2, Bs_Power))
                        for j in np.arange(y - 0.35, y + 0.35, 0.05):
                            for i in np.arange(x - 0.35, x + 0.35, 0.7):
                                Rss = max(Rss, self.Calculate_SINR(self.Phase_obstacle2_point, i, j, self.Freq2, Bs_Power))
                        if Rss < Rss_set2:
                            self.Phase_obstacle2.append([x, y])
                    if Rss3 < Rss_set3:
                        self.Phase_obstacle3.append([x, y])
                    else:
                        Rss = 0
                        for i in np.arange(x - 0.35, x + 0.35, 0.05):
                            for j in np.arange(y - 0.35, y + 0.35, 0.7):
                                Rss = max(Rss, self.Calculate_SINR(self.Phase_obstacle3_point, i, j, self.Freq3, Bs_Power))
                        for j in np.arange(y - 0.35, y + 0.35, 0.05):
                            for i in np.arange(x - 0.35, x + 0.35, 0.7):
                                Rss = max(Rss, self.Calculate_SINR(self.Phase_obstacle3_point, i, j, self.Freq3, Bs_Power))
                        if Rss < Rss_set3:
                            self.Phase_obstacle3.append([x, y])
                    if Rss4 < Rss_set4:
                        self.Phase_obstacle4.append([x, y])
                    else:
                        Rss = 0
                        for i in np.arange(x - 0.35, x + 0.35, 0.05):
                            for j in np.arange(y - 0.35, y + 0.35, 0.7):
                                Rss = max(Rss, self.Calculate_SINR(self.Phase_obstacle4_point, i, j, self.Freq4, Bs_Power))
                        for j in np.arange(y - 0.35, y + 0.35, 0.05):
                            for i in np.arange(x - 0.35, x + 0.35, 0.7):
                                Rss = max(Rss, self.Calculate_SINR(self.Phase_obstacle4_point, i, j, self.Freq4, Bs_Power))
                        if Rss < Rss_set4:
                            self.Phase_obstacle4.append([x, y])
                    if Rss5 < Rss_set5:
                        self.Phase_obstacle5.append([x, y])
                    else:
                        Rss = 0
                        for i in np.arange(x - 0.35, x + 0.35, 0.05):
                            for j in np.arange(y - 0.35, y + 0.35, 0.7):
                                Rss = max(Rss, self.Calculate_SINR(self.Phase_obstacle5_point, i, j, self.Freq5, Bs_Power))
                        for j in np.arange(y - 0.35, y + 0.35, 0.05):
                            for i in np.arange(x - 0.35, x + 0.35, 0.7):
                                Rss = max(Rss, self.Calculate_SINR(self.Phase_obstacle5_point, i, j, self.Freq5, Bs_Power))
                        if Rss < Rss_set5:
                            self.Phase_obstacle5.append([x, y])
                    if Rss6 < Rss_set6:
                        self.Phase_obstacle6.append([x, y])
                    else:
                        Rss = 0
                        for i in np.arange(x - 0.35, x + 0.35, 0.05):
                            for j in np.arange(y - 0.35, y + 0.35, 0.7):
                                Rss = max(Rss, self.Calculate_SINR(self.Phase_obstacle6_point, i, j, self.Freq6, Bs_Power))
                        for j in np.arange(y - 0.35, y + 0.35, 0.05):
                            for i in np.arange(x - 0.35, x + 0.35, 0.7):
                                Rss = max(Rss, self.Calculate_SINR(self.Phase_obstacle6_point, i, j, self.Freq6, Bs_Power))
                        if Rss < Rss_set6:
                            self.Phase_obstacle6.append([x, y])
                    if Rss7 < Rss_set7:
                        self.Phase_obstacle7.append([x, y])
                    else:
                        Rss = 0
                        for i in np.arange(x - 0.35, x + 0.35, 0.05):
                            for j in np.arange(y - 0.35, y + 0.35, 0.7):
                                Rss = max(Rss, self.Calculate_SINR(self.Phase_obstacle7_point, i, j, self.Freq7, Bs_Power))
                        for j in np.arange(y - 0.35, y + 0.35, 0.05):
                            for i in np.arange(x - 0.35, x + 0.35, 0.7):
                                Rss = max(Rss, self.Calculate_SINR(self.Phase_obstacle7_point, i, j, self.Freq7, Bs_Power))
                        if Rss < Rss_set7:
                            self.Phase_obstacle7.append([x, y])
                    if Rss8 < Rss_set8:
                        self.Phase_obstacle8.append([x, y])
                    else:
                        Rss = 0
                        for i in np.arange(x - 0.35, x + 0.35, 0.05):
                            for j in np.arange(y - 0.35, y + 0.35, 0.7):
                                Rss = max(Rss, self.Calculate_SINR(self.Phase_obstacle8_point, i, j, self.Freq8, Bs_Power))
                        for j in np.arange(y - 0.35, y + 0.35, 0.05):
                            for i in np.arange(x - 0.35, x + 0.35, 0.7):
                                Rss = max(Rss, self.Calculate_SINR(self.Phase_obstacle8_point, i, j, self.Freq8, Bs_Power))
                        if Rss < Rss_set8:
                            self.Phase_obstacle8.append([x, y])
                    if Rss9 < Rss_set9:
                        self.Phase_obstacle9.append([x, y])
                    else:
                        Rss = 0
                        for i in np.arange(x - 0.35, x + 0.35, 0.05):
                            for j in np.arange(y - 0.35, y + 0.35, 0.7):
                                Rss = max(Rss, self.Calculate_SINR(self.Phase_obstacle9_point, i, j, self.Freq9, Bs_Power))
                        for j in np.arange(y - 0.35, y + 0.35, 0.05):
                            for i in np.arange(x - 0.35, x + 0.35, 0.7):
                                Rss = max(Rss, self.Calculate_SINR(self.Phase_obstacle9_point, i, j, self.Freq9, Bs_Power))
                        if Rss < Rss_set9:
                            self.Phase_obstacle9.append([x, y])
            print(type(self.Phase_obstacle1_point))

            New_obstacle_num = 0
            for x in range(self.size):#如果有一个点在所有的频段中均是拥塞的就添加进self.Phase_obstacle数组中
                for y in range(self.size):
                    Temp_Phase_List = [1, 2, 3, 4, 5, 6, 7, 8, 9]
                    for p in self.Phase_obstacle1:
                        if p[0] == x and p[1] == y:
                            Temp_Phase_List.remove(1)
                            break
                    for p in self.Phase_obstacle2:
                        if p[0] == x and p[1] == y:
                            Temp_Phase_List.remove(2)
                            break
                    for p in self.Phase_obstacle3:
                        if p[0] == x and p[1] == y:
                            Temp_Phase_List.remove(3)
                            break
                    for p in self.Phase_obstacle4:
                        if p[0] == x and p[1] == y:
                            Temp_Phase_List.remove(4)
                            break
                    for p in self.Phase_obstacle5:
                        if p[0] == x and p[1] == y:
                            Temp_Phase_List.remove(5)
                            break
                    for p in self.Phase_obstacle6:
                        if p[0] == x and p[1] == y:
                            Temp_Phase_List.remove(6)
                            break
                    for p in self.Phase_obstacle7:
                        if p[0] == x and p[1] == y:
                            Temp_Phase_List.remove(7)
                            break
                    for p in self.Phase_obstacle8:
                        if p[0] == x and p[1] == y:
                            Temp_Phase_List.remove(8)
                            break
                    for p in self.Phase_obstacle9:
                        if p[0] == x and p[1] == y:
                            Temp_Phase_List.remove(9)
                            break
                    if len(Temp_Phase_List) == 0:
                        Temp_Phase_List = [0]
                        self.Phase_obstacle.append([x, y])
                        New_obstacle_num += 1
                    self.Phase_use_list[x][y] = Temp_Phase_List
            np.save('Phase_obstacle1.npy', self.Phase_obstacle1)
            np.save('Phase_obstacle2.npy', self.Phase_obstacle2)
            np.save('Phase_obstacle3.npy', self.Phase_obstacle3)
            np.save('Phase_obstacle4.npy', self.Phase_obstacle4)
            np.save('Phase_obstacle5.npy', self.Phase_obstacle5)
            np.save('Phase_obstacle6.npy', self.Phase_obstacle6)
            np.save('Phase_obstacle7.npy', self.Phase_obstacle7)
            np.save('Phase_obstacle8.npy', self.Phase_obstacle8)
            np.save('Phase_obstacle9.npy', self.Phase_obstacle9)
            np.save('Phase_use_list.npy', np.array(self.Phase_use_list, dtype=object))
            np.save('Phase_obstacle.npy', self.Phase_obstacle)

            np.save('Phase1.npy', self.Phase1)
            np.save('Phase2.npy', self.Phase2)
            np.save('Phase3.npy', self.Phase3)
            np.save('Phase4.npy', self.Phase4)
            np.save('Phase5.npy', self.Phase5)
            np.save('Phase6.npy', self.Phase6)
            np.save('Phase7.npy', self.Phase7)
            np.save('Phase8.npy', self.Phase8)
            np.save('Phase9.npy', self.Phase9)


        else:
            self.Phase_obstacle1 = np.load('Phase_obstacle1.npy', allow_pickle=True)  # 加载确定地图，频段拥塞1
            self.Phase_obstacle2 = np.load('Phase_obstacle2.npy', allow_pickle=True)  # 加载确定地图，频段拥塞2
            self.Phase_obstacle3 = np.load('Phase_obstacle3.npy', allow_pickle=True)  # 加载确定地图，频段拥塞3
            self.Phase_obstacle4 = np.load('Phase_obstacle4.npy', allow_pickle=True)  # 加载确定地图，频段拥塞4
            self.Phase_obstacle5 = np.load('Phase_obstacle5.npy', allow_pickle=True)  # 加载确定地图，频段拥塞5
            self.Phase_obstacle6 = np.load('Phase_obstacle6.npy', allow_pickle=True)  # 加载确定地图，频段拥塞6
            self.Phase_obstacle7 = np.load('Phase_obstacle7.npy', allow_pickle=True)  # 加载确定地图，频段拥塞7
            self.Phase_obstacle8 = np.load('Phase_obstacle8.npy', allow_pickle=True)  # 加载确定地图，频段拥塞8
            self.Phase_obstacle9 = np.load('Phase_obstacle9.npy', allow_pickle=True)  # 加载确定地图，频段拥塞9
            self.Phase_use_list = np.load('Phase_use_list.npy', allow_pickle=True)  # 加载确定地图，可用频段
            self.Phase_obstacle = np.load('Phase_obstacle.npy', allow_pickle=True)  # 加载确定地图，频段障碍

        '''
        New_obstacle_num = 0
        for x in range(self.size):
            for y in range(self.size):
                Temp_Phase_List = [1, 2, 3, 4, 5, 6, 7, 8, 9]
                for p in self.Phase_obstacle1:
                    if p[0] == x and p[1] == y:
                        Temp_Phase_List.remove(1)
                        break
                for p in self.Phase_obstacle2:
                    if p[0] == x and p[1] == y:
                        Temp_Phase_List.remove(2)
                        break
                for p in self.Phase_obstacle3:
                    if p[0] == x and p[1] == y:
                        Temp_Phase_List.remove(3)
                        break
                for p in self.Phase_obstacle4:
                    if p[0] == x and p[1] == y:
                        Temp_Phase_List.remove(4)
                        break
                for p in self.Phase_obstacle5:
                    if p[0] == x and p[1] == y:
                        Temp_Phase_List.remove(5)
                        break
                for p in self.Phase_obstacle6:
                    if p[0] == x and p[1] == y:
                        Temp_Phase_List.remove(6)
                        break
                for p in self.Phase_obstacle7:
                    if p[0] == x and p[1] == y:
                        Temp_Phase_List.remove(7)
                        break
                for p in self.Phase_obstacle8:
                    if p[0] == x and p[1] == y:
                        Temp_Phase_List.remove(8)
                        break
                for p in self.Phase_obstacle9:
                    if p[0] == x and p[1] == y:
                        Temp_Phase_List.remove(9)
                        break
                if len(Temp_Phase_List) == 0:
                    Temp_Phase_List = [0]
                    self.Phase_obstacle.append([x, y])
                    New_obstacle_num += 1
                self.Phase_use_list[x][y] = Temp_Phase_List

        print(New_obstacle_num)
        '''


    def IsObstacle(self, i, j, ob):
        for p in ob: # set(self.obstacle_point) | set(ob):
            if i == p[0] and j == p[1]:
                return True
        return False

    def IsPhaseOb(self, i, j):
        for p in self.Phase_obstacle:
            if i == p[0] and j == p[1]:
                return True
        return False

    def Is_Phase_Obstacle1(self, i, j):
        for p in self.Phase_obstacle1:
            if i == p[0] and j == p[1]:
                return True
        return False

    def Is_Phase_Obstacle2(self, i, j):
        for p in self.Phase_obstacle2:
            if i == p[0] and j == p[1]:
                return True
        return False

    def Is_Phase_Obstacle3(self, i, j):
        for p in self.Phase_obstacle3:
            if i == p[0] and j == p[1]:
                return True
        return False

    def Is_Phase_Obstacle4(self, i, j):
        for p in self.Phase_obstacle4:
            if i == p[0] and j == p[1]:
                return True
        return False

    def Is_Phase_Obstacle5(self, i, j):
        for p in self.Phase_obstacle5:
            if i == p[0] and j == p[1]:
                return True
        return False

    def Is_Phase_Obstacle6(self, i, j):
        for p in self.Phase_obstacle6:
            if i == p[0] and j == p[1]:
                return True
        return False

    def Is_Phase_Obstacle7(self, i, j):
        for p in self.Phase_obstacle7:
            if i == p[0] and j == p[1]:
                return True
        return False

    def Is_Phase_Obstacle8(self, i, j):
        for p in self.Phase_obstacle8:
            if i == p[0] and j == p[1]:
                return True
        return False

    def Is_Phase_Obstacle9(self, i, j):
        for p in self.Phase_obstacle9:
            if i == p[0] and j == p[1]:
                return True
        return False
    

if __name__ == "__main__":
    map = RandomMap(100, 100)
    map.Generate_Map()
    print(map.Phase_obstacle1)
    print(map.Phase_obstacle2)
    print(map.Phase_obstacle3)
    print(map.Phase_obstacle4)
    print(map.Phase_obstacle5)
    print(map.Phase_obstacle6)
    print(map.Phase_obstacle7)
    print(map.Phase_obstacle8)
    print(map.Phase_obstacle9)
