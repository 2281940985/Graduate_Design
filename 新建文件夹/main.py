import numpy as np
import matplotlib.pyplot as plt

# 生成示例数据
data = np.load(r"E:\Graduate_Design\ZuHao_Code\test_SingleUser\Phase1.npy")  # 10x10 的随机数据
plt.rcParams['font.family'] = 'SimHei'  # 设置中文字体
# 创建图形和坐标轴
fig, ax = plt.subplots(figsize=(10,10))
data = np.nan_to_num(data, nan=0, posinf=np.max(data[np.isfinite(data)]),
                             neginf=np.min(data[np.isfinite(data)]))
# 绘制热力图
heatmap = ax.imshow(data, cmap='jet', interpolation='nearest', origin='lower',
                    extent=[0, data.shape[1], 0, data.shape[0]],
                    vmin=np.min(data), vmax=np.max(data))
ax.margins(x=0.5, y=0.5)
# 添加颜色条
cbar = plt.colorbar(heatmap, ax=ax, fraction=0.046, pad=0.04)
cbar.set_label('SINR Strength', fontsize=14)  # 设置颜色条标签

ax.set_xlim(-10, 80)  # x 轴范围从 -1 到 11
ax.set_ylim(-3, 73)  # y 轴范围从 -1.2 到 1.2

ax.set_xlabel('X axis (0-69)', fontsize=12)
ax.set_ylabel('Y axis (0-69)', fontsize=12)
ax.grid(True, color='white', linestyle='--', linewidth=1.0)

ax.set_xticks(np.arange(0, data.shape[1] + 1, 10))
ax.set_yticks(np.arange(0, data.shape[0] + 1, 10))
x_centered = [6]
y_centered =[6]
ax.scatter(x_centered[0], y_centered[0], color='purple', s=10, marker='s', label='起点')
ax.legend(loc='upper left', bbox_to_anchor=(0, 1),
                 fontsize=8, frameon=True, borderpad=0.5,
                 labelspacing=0.99, handlelength=1.5)
# 显示图形
plt.show()