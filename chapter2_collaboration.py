import numpy as np

def check_collision(drones, safe_distance):
    """
    检测无人机之间的潜在冲突。
    参数:
        drones: 无人机状态列表，每个状态包含 'x', 'y' 等关键位置参数。
        safe_distance: 冲突检测的安全距离阈值。
    返回:
        collisions: 冲突对列表 [(i, j), ...]，表示无人机 i 和 j 之间的冲突。
    """
    collisions = []
    num_drones = len(drones)
    
    for i in range(num_drones):
        for j in range(i + 1, num_drones):
            drone_i = drones[i]
            drone_j = drones[j]
            # 计算无人机 i 和 j 的距离
            distance = np.sqrt((drone_i['x'] - drone_j['x'])**2 + (drone_i['y'] - drone_j['y'])**2)
            if distance < safe_distance:
                collisions.append((i, j))
    return collisions


def resolve_collision(drones, collisions, delta_v, delta_phi_max):
    """
    解决无人机之间的冲突，通过调整速度和航向规避碰撞。
    参数:
        drones: 无人机状态列表。
        collisions: 冲突对列表 [(i, j), ...]。
        delta_v: 在解决冲突时调整的速度量。
        delta_phi_max: 在解决冲突时调整的最大航向角度。
    """
    for i, j in collisions:
        drone_i = drones[i]
        drone_j = drones[j]

        # 调整无人机 i 的速度和航向
        drone_i['v'] = max(0.1, drone_i['v'] - delta_v)  # 保证速度不为零
        drone_i['phi'] += delta_phi_max  # 向一个方向偏转

        # 调整无人机 j 的速度和航向
        drone_j['v'] = max(0.1, drone_j['v'] - delta_v)
        drone_j['phi'] -= delta_phi_max  # 向相反方向偏转


def visualize_collisions(drones, collisions):
    """
    可视化无人机的位置和冲突情况。
    参数:
        drones: 无人机状态列表。
        collisions: 冲突对列表。
    """
    import matplotlib.pyplot as plt

    plt.figure(figsize=(8, 8))
    plt.xlim(0, 50)
    plt.ylim(0, 50)
    
    # 绘制无人机位置
    for i, drone in enumerate(drones):
        plt.scatter(drone['x'], drone['y'], label=f"Drone {i}")
    
    # 绘制冲突情况
    for i, j in collisions:
        drone_i = drones[i]
        drone_j = drones[j]
        plt.plot(
            [drone_i['x'], drone_j['x']],
            [drone_i['y'], drone_j['y']],
            'r--', label=f"Collision {i}-{j}"
        )
    
    plt.legend()
    plt.title("Drone Positions and Collisions")
    plt.xlabel("X")
    plt.ylabel("Y")
    plt.grid()
    plt.show()


if __name__ == "__main__":
    # ========== 参数配置 ==========
    SAFE_DISTANCE = 5  # 冲突检测的安全距离
    DELTA_V = 0.5  # 冲突规避时的速度调整量
    DELTA_PHI_MAX = np.pi / 8  # 冲突规避时的最大航向角度

    # ========== 无人机初始化 ==========
    drones = [
        {'x': 10, 'y': 10, 'v': 2.0, 'phi': np.pi / 4},
        {'x': 12, 'y': 12, 'v': 1.8, 'phi': -np.pi / 6},
        {'x': 20, 'y': 20, 'v': 1.5, 'phi': np.pi / 3},
        {'x': 11, 'y': 11, 'v': 1.6, 'phi': np.pi / 8},
    ]
    
    # ========== 检测冲突 ==========
    collisions = check_collision(drones, SAFE_DISTANCE)
    print(f"Collisions detected: {collisions}")
    
    # ========== 解决冲突 ==========
    resolve_collision(drones, collisions, DELTA_V, DELTA_PHI_MAX)
    print("Resolved Drone States:")
    for i, drone in enumerate(drones):
        print(f"Drone {i}: {drone}")

    # ========== 可视化冲突 ==========
    visualize_collisions(drones, collisions)
