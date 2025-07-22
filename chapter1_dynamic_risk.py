import numpy as np

def calculate_risk_field(x, y, drone, alpha, eta, kappa, epsilon, A):
    """
    计算坐标点 (x, y) 在当前无人机的动态风险场中的风险值，符合论文公式。
    参数:
        x, y: 实际坐标（范围 0-100）
        alpha: 风险衰减速率
        eta: 速度增益系数
        kappa: 转弯代价系数
        epsilon: 后方基础风险值
        A: 风险幅度系数
    """
    x_i, y_i, v_i, phi_i, delta_phi = drone['x'], drone['y'], drone['v'], drone['phi'], drone['delta_phi']
    
    # 距离衰减项：f_distance = exp(-alpha * d)
    d = np.sqrt((x - x_i)**2 + (y - y_i)**2)
    if isinstance(d, np.ndarray):
        d = np.where(d == 0, 1e-10, d)
    else:
        d = 1e-10 if d == 0 else d
    f_distance = np.exp(-alpha * d)
    
    # 方向性影响项：f_direction = epsilon + (1 - epsilon) * (1 + cos(theta)) / 2
    cos_theta = ((x - x_i) * np.cos(phi_i) + (y - y_i) * np.sin(phi_i)) / d
    f_direction = epsilon + (1 - epsilon) * (1 + cos_theta) / 2
    
    # 速度增益项：f_speed = 1 + eta * v
    f_speed = 1 + eta * v_i
    
    # 转弯代价项：f_turn = 1 + kappa * |delta_phi|
    f_turn = 1 + kappa * abs(delta_phi)
    
    # 总风险：R = A * f_distance * f_direction * f_speed * f_turn
    risk = A * f_distance * f_direction * f_speed * f_turn
    return risk

def update_risk_field(risk_map, drones, alpha, eta, kappa, epsilon, A, step_size=0.5, map_width=100, map_height=100):
    """
    使用所有无人机的状态更新整个地图的动态风险场，并裁剪风险值范围。
    参数:
        risk_map: 风险场地图，二维数组（例如 200x200）
        step_size: 网格步长，默认 0.5
        map_width, map_height: 实际地图范围（0-100）
    """
    width, height = risk_map.shape
    risk_map.fill(0)
    
    for i in range(width):
        for j in range(height):
            x = i * step_size
            y = j * step_size
            for drone in drones:
                risk_map[i, j] += calculate_risk_field(x, y, drone, alpha, eta, kappa, epsilon, A)
    
    # 裁剪风险值范围到 [0, 5]
    risk_map = np.clip(risk_map, 0, 5)
    return risk_map

def visualize_risk_map(risk_map, step_size=0.5, map_width=100, map_height=100, output_file="risk_field.svg"):
    import matplotlib.pyplot as plt
    plt.figure(figsize=(6, 6), dpi=300)
    extent = [0, map_width, 0, map_height]
    
    plt.imshow(risk_map, cmap='hot', interpolation='bilinear', extent=extent, origin='lower', vmin=0, vmax=5)
    plt.colorbar(label='Risk Level', shrink=0.7, aspect=15, pad=0.02, ticks=[0, 1, 2, 3, 4, 5])
    plt.title('Dynamic Risk Field', fontsize=12, pad=15)
    plt.xlabel('X/m', fontsize=10)
    plt.ylabel('Y/m', fontsize=10)
    plt.grid(True, alpha=0.2)
    
    plt.savefig(output_file, format='svg', dpi=300, bbox_inches='tight')
    print(f"Risk field saved as {output_file}")
    plt.close()

if __name__ == "__main__":
    MAP_WIDTH = 100
    MAP_HEIGHT = 100
    NUM_DRONES = 3
    STEP_SIZE = 0.5
    GRID_WIDTH = int(MAP_WIDTH / STEP_SIZE)
    GRID_HEIGHT = int(MAP_HEIGHT / STEP_SIZE)

    ALPHA = 0.1
    ETA = 0.2
    KAPPA = 0.1
    EPSILON = 0.1
    A = 1.0
    
    drones = [
        {'x': 20, 'y': 20, 'v': 2.0, 'phi': np.pi / 4, 'delta_phi': np.pi / 12},
        {'x': 60, 'y': 60, 'v': 1.5, 'phi': -np.pi / 6, 'delta_phi': np.pi / 8},
        {'x': 40, 'y': 80, 'v': 1.8, 'phi': np.pi / 3, 'delta_phi': np.pi / 10}
    ]
    
    risk_map = np.zeros((GRID_WIDTH, GRID_HEIGHT))
    risk_map = update_risk_field(risk_map, drones, ALPHA, ETA, KAPPA, EPSILON, A, STEP_SIZE, MAP_WIDTH, MAP_HEIGHT)
    visualize_risk_map(risk_map, STEP_SIZE, MAP_WIDTH, MAP_HEIGHT, output_file="risk_field.svg")
