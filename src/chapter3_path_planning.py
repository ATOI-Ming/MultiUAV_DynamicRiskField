import numpy as np
from scipy.ndimage import gaussian_filter

def global_path_planning(risk_map, start, goal, num_waypoints=10):
    """
    基于全局信息生成无人机的初步路径。
    参数:
        risk_map: 风险地图，二维数组
        start: 起始点坐标 (x, y)
        goal: 目标点坐标 (x, y)
        num_waypoints: 路径点数量，默认值为 10
    返回:
        path: 初步规划的路径列表 [(x1, y1), (x2, y2), ...]
    """
    # 简单线性路径规划，连接起点和终点
    path = [
        (
            int(start[0] + i * (goal[0] - start[0]) / num_waypoints),
            int(start[1] + i * (goal[1] - start[1]) / num_waypoints)
        )
        for i in range(num_waypoints + 1)
    ]
    return path


def local_path_optimization(path, risk_map, threshold=0.5):
    """
    对全局规划的路径进行局部优化，避开高风险区域。
    参数:
        path: 全局路径 [(x1, y1), (x2, y2), ...]
        risk_map: 风险地图，二维数组
        threshold: 风险值阈值，高于该值的区域视为高风险，默认值为 0.5
    返回:
        optimized_path: 优化后的路径列表 [(x1, y1), (x2, y2), ...]
    """
    optimized_path = []
    for point in path:
        x, y = point
        # 如果风险值低于阈值，直接加入路径
        if risk_map[x, y] < threshold:
            optimized_path.append(point)
        else:
            # 在 3x3 邻域内找到最低风险点
            nearby_points = [
                (x + dx, y + dy)
                for dx in range(-1, 2)
                for dy in range(-1, 2)
                if 0 <= x + dx < risk_map.shape[0] and 0 <= y + dy < risk_map.shape[1]
            ]
            min_risk_point = min(nearby_points, key=lambda p: risk_map[p[0], p[1]])
            optimized_path.append(min_risk_point)
    return optimized_path


def visualize_path(risk_map, path, title="Path Visualization"):
    """
    可视化路径与风险地图。
    参数:
        risk_map: 风险地图，二维数组
        path: 路径点列表 [(x1, y1), (x2, y2), ...]
        title: 图像标题，默认值为 "Path Visualization"
    """
    import matplotlib.pyplot as plt

    plt.imshow(risk_map, cmap='hot', interpolation='nearest')
    plt.colorbar(label='Risk Level')

    # 绘制路径
    path_x, path_y = zip(*path)
    plt.plot(path_y, path_x, marker='o', color='blue', label='Path')

    plt.title(title)
    plt.xlabel("Y")
    plt.ylabel("X")
    plt.legend()
    plt.grid()
    plt.show()


if __name__ == "__main__":
    # ========== 参数配置 ==========
    MAP_WIDTH = 50
    MAP_HEIGHT = 50
    NUM_WAYPOINTS = 10
    RISK_THRESHOLD = 0.5

    # ========== 风险地图初始化 ==========
    risk_map = np.random.random((MAP_WIDTH, MAP_HEIGHT))  # 随机生成风险地图
    risk_map = gaussian_filter(risk_map, sigma=2)  # 平滑化风险地图

    # ========== 无人机起点和目标点 ==========
    start = (5, 5)
    goal = (45, 45)

    # ========== 全局路径规划 ==========
    path = global_path_planning(risk_map, start, goal, NUM_WAYPOINTS)
    print(f"Global Path: {path}")

    # ========== 局部路径优化 ==========
    optimized_path = local_path_optimization(path, risk_map, RISK_THRESHOLD)
    print(f"Optimized Path: {optimized_path}")

    # ========== 可视化路径 ==========
    visualize_path(risk_map, optimized_path, title="Optimized Path")
