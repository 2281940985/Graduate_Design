import numpy as np
import matplotlib.pyplot as plt
import os
import matplotlib.pyplot as plt
from matplotlib.colors import Normalize, LinearSegmentedColormap, BoundaryNorm


# 加载.npy文件
def load_npy_file(file_path):
    return np.load(file_path, allow_pickle=True)


# 绘制70x70网格图并标记.npy文件中的坐标
def plot_grid_with_coordinates(npy_data):
    # 创建一个70x70的网格图
    x = np.arange(71)
    y = np.arange(71)

    # 创建图形和轴对象
    fig, ax = plt.subplots()
    
    # 设置图形大小为正方形
    fig.set_size_inches(10, 10)

    # 绘制网格
    for i in x[:-1]:
        for j in y[:-1]:
            rect = plt.Rectangle((i, j), 1, 1, edgecolor='gray', facecolor='w')
            ax.add_patch(rect)

    # 标记.npy文件中的坐标
    if npy_data.size > 0:
        # 假设.npy文件中的数据是二维的，每行是一个坐标(x, y)
        for coord in npy_data:
            x, y = coord[0], coord[1]
            # 绘制方形标记
            rect = plt.Rectangle((x, y), 1, 1, facecolor='gray', label='障碍物')
            ax.add_patch(rect)

    # 设置坐标轴范围和刻度
    ax.set_xlim(-0.5, 70.5)
    ax.set_ylim(-0.5, 70.5)
    ax.set_xticks(np.arange(0, 71, 5))
    ax.set_yticks(np.arange(0, 71, 5))
    
    # 设置坐标轴比例相等,保证方格为正方形
    ax.set_aspect('equal')

    # 设置标题和标签
    ax.set_title('70x70 Map Obstacle')
    ax.set_xlabel('X Coordinate')
    ax.set_ylabel('Y Coordinate')

    # 显示图形
    plt.show()


def plot_heatmap(file_path,i):
    # 加载.npy文件并处理异常值
    try:
        data = np.load(file_path, allow_pickle=True)
    except Exception as e:
        print(f"加载文件失败: {e}")
        return

    # 检查数据有效性
    if data.size == 0:
        print("数据为空")
        return

    # 确保数据是70x70的二维数组
    if data.shape != (70, 70):
        try:
            data = data.reshape(70, 70)
        except ValueError:
            print("数据无法reshape为70x70网格")
            return

    # 处理特殊值
    data = np.nan_to_num(data, nan=0, posinf=np.max(data[np.isfinite(data)]), 
                        neginf=np.min(data[np.isfinite(data)]))

    # 创建热力图
    plt.figure(figsize=(10, 8))
    
    # 使用jet色图
    heatmap = plt.imshow(data, cmap='jet', interpolation='nearest', 
                        extent=[0, 70, 0, 70], vmin=np.min(data), vmax=np.max(data))
    
    # 添加颜色条
    cbar = plt.colorbar(heatmap)
    cbar.set_label('SINR Strength',fontsize=20)

    # 添加更明显的网格线
    plt.grid(True, color='white', linestyle='--', linewidth=1.2)
    
    # 在每个网格单元之间添加白色虚线
    for x in range(70):
        plt.axvline(x=x, color='white', linestyle='--', linewidth=0.5, alpha=0.7)
    for y in range(70):
        plt.axhline(y=y, color='white', linestyle='--', linewidth=0.5, alpha=0.7)
    
    # 设置标题和标签
    pic_name = 'Phase'
    plt.title(f"{pic_name}{i+1} SINR Heatmap", fontsize=14)
    plt.xlabel('X axis (0-69)', fontsize=12)
    plt.ylabel('Y axis (0-69)', fontsize=12)
    
    # 设置刻度
    plt.xticks(np.arange(0, 71, 5))
    plt.yticks(np.arange(0, 71, 5))
    plt.savefig(rf"E:\Graduate_Design\ZuHao_Code\test_SingleUser\频谱地图\{pic_name}{i+1}.png", dpi=1080)

    # 显示图形
    plt.tight_layout()
    plt.show()
# 主函数



if __name__ == "__main__":
    data_npy = np.load('map_obstacle_coordinate.npy')
    print(data_npy.shape)
    plot_grid_with_coordinates(data_npy)
    file_path = r'E:\Graduate_Design\ZuHao_Code\test_SingleUser\频谱地图'
    for i in range(9):
        plot_heatmap(f"{file_path}\\Phase{i+1}.npy",i)
    # plot_grid_with_coordinates(data_npy)
    # plot_heatmap('Phase9.npy')
