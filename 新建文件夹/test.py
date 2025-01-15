import matplotlib.pyplot as plt
import numpy as np

# 创建数据
x = np.linspace(0, 10, 100)
y = np.sin(x)

# 创建图形和坐标轴
fig, ax = plt.subplots()

# 绘制图形
ax.plot(x, y)

# 在 x 轴和 y 轴上添加 10% 的空白区域
ax.margins(x=0.1, y=0.1)

# 显示图形
plt.show()