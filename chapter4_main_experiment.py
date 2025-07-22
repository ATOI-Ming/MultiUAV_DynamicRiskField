import numpy as np
from scipy.ndimage import gaussian_filter
from chapter1_dynamic_risk import update_risk_field, calculate_risk_field
from chapter2_collaboration import check_collision, resolve_collision
from chapter3_path_planning import global_path_planning, local_path_optimization
import matplotlib.pyplot as plt
from matplotlib.colors import LinearSegmentedColormap
from matplotlib.patches import PathPatch, Circle
from matplotlib.path import Path
import pandas as pd
import os

# 字体处理函数
def setup_fonts_for_plot():
    from matplotlib.font_manager import FontProperties, findfont
    chinese_fonts = ['WenQuanYi Zen Hei', 'SimHei', 'Microsoft YaHei', 'SimSun', 
                    'AR PL UMing CN', 'AR PL UKai CN', 'NSimSun', 'STSong']
    font_found = False
    for font_name in chinese_fonts:
        try:
            font_path = findfont(FontProperties(family=font_name))
            if font_path and not font_path.endswith('DejaVuSans.ttf'):
                plt.rcParams['font.family'] = ['sans-serif']
                plt.rcParams['font.sans-serif'] = [font_name] + plt.rcParams['font.sans-serif']
                plt.rcParams['axes.unicode_minus'] = False
                print(f"使用中文字体: {font_name}")
                font_found = True
                break
        except:
            continue
    if not font_found:
        print("未找到合适的中文字体，将使用英文标签")
    return font_found

USE_CHINESE = setup_fonts_for_plot()

# 参数配置
MAP_WIDTH = 100
MAP_HEIGHT = 100
NUM_DRONES = 3
SAFE_DISTANCE = 5
DELTA_V = 0.5
DELTA_PHI_MAX = np.pi / 6
ALPHA = 0.1
ETA = 0.2
KAPPA = 0.1
EPSILON = 0.1
A = 1.0
RISK_THRESHOLD = 0.5
NUM_WAYPOINTS = 20
MAX_TIMESTEPS = 200
TARGET_TOLERANCE = 2
MIN_SPEED = 0.5
MIN_PATH_DISTANCE = 2
STEP_SIZE = 0.5
GRID_WIDTH = int(MAP_WIDTH / STEP_SIZE)
GRID_HEIGHT = int(MAP_HEIGHT / STEP_SIZE)

V_MAX = 20.0
V_REF = 12.0
A_MAX = 2.0
T_INTERVAL = 0.1

W1 = 0.4  # 距离目标权重
W2 = 0.3  # 速度权重
W3 = 0.3  # 路径风险权重

ETA_SHARED = 0.1
SHARED_PATH_POINTS = 3
DYNAMIC_OPTIMIZATION_THRESHOLD = 0.8
REPLAN_INTERVAL = 15

custom_cmap = LinearSegmentedColormap.from_list(
    "custom_cmap", ["#000033", "#003366", "#00CCCC", "#FFFF99", "#FF6600", "#CC0000", "#660000"]
)

def get_label(zh_text, en_text):
    return zh_text if USE_CHINESE else en_text

def calculate_priority(drone, drones, risk_map):
    d = np.sqrt((drone['x'] - drone['target'][0])**2 + (drone['y'] - drone['target'][1])**2)
    d = max(d, 0.1)
    if drone['path']:
        path_points = drone['path'][:min(3, len(drone['path']))]
        risk_sum = 0
        for point in path_points:
            x_idx = min(GRID_WIDTH - 1, max(0, int(point[0] / STEP_SIZE)))
            y_idx = min(GRID_HEIGHT - 1, max(0, int(point[1] / STEP_SIZE)))
            risk_sum += risk_map[x_idx, y_idx]
        risk_value = risk_sum / len(path_points)
    else:
        x_idx = min(GRID_WIDTH - 1, max(0, int(drone['x'] / STEP_SIZE)))
        y_idx = min(GRID_HEIGHT - 1, max(0, int(drone['y'] / STEP_SIZE)))
        risk_value = risk_map[x_idx, y_idx]
    risk_value = max(risk_value, 0.1)
    proximity_factor = 0
    for other_drone in drones:
        if other_drone != drone:
            dist = np.sqrt((drone['x'] - other_drone['x'])**2 + (drone['y'] - other_drone['y'])**2)
            if dist < 20:
                proximity_factor += (20 - dist) / 20
    proximity_factor = min(proximity_factor, 1.0)
    priority = W1 * (1/d) + W2 * (drone['v']/V_MAX) + W3 * (1/risk_value) + 0.1 * proximity_factor
    return priority

def calculate_point_risk(x_idx, y_idx, drone, drones):
    return max(0.1, sum(calculate_risk_field(x_idx * STEP_SIZE, y_idx * STEP_SIZE, d, ALPHA, ETA, KAPPA, EPSILON, A) 
                      for d in drones if d != drone))

def resolve_collision_optimized(drones, collisions):
    for i, j in collisions:
        if drones[i]['priority'] > drones[j]['priority']:
            drones[j]['v'] = max(MIN_SPEED, drones[j]['v'] - DELTA_V)
            relative_x = drones[i]['x'] - drones[j]['x']
            relative_y = drones[i]['y'] - drones[j]['y']
            avoidance_angle = np.arctan2(relative_y, relative_x)
            angle_diff = np.arctan2(np.sin(avoidance_angle + np.pi - drones[j]['phi']), 
                                  np.cos(avoidance_angle + np.pi - drones[j]['phi']))
            adjustment = np.clip(angle_diff, -DELTA_PHI_MAX * 0.7, DELTA_PHI_MAX * 0.7)
            drones[j]['phi'] = drones[j]['phi'] + adjustment
        else:
            drones[i]['v'] = max(MIN_SPEED, drones[i]['v'] - DELTA_V)
            relative_x = drones[j]['x'] - drones[i]['x']
            relative_y = drones[j]['y'] - drones[i]['y']
            avoidance_angle = np.arctan2(relative_y, relative_x)
            angle_diff = np.arctan2(np.sin(avoidance_angle + np.pi - drones[i]['phi']), 
                                  np.cos(avoidance_angle + np.pi - drones[i]['phi']))
            adjustment = np.clip(angle_diff, -DELTA_PHI_MAX * 0.7, DELTA_PHI_MAX * 0.7)
            drones[i]['phi'] = drones[i]['phi'] + adjustment

def update_risk_field_shared(risk_map, drones):
    risk_map = update_risk_field(risk_map, drones, ALPHA, ETA, KAPPA, EPSILON, A, STEP_SIZE, MAP_WIDTH, MAP_HEIGHT)
    for drone in drones:
        if len(drone['path_history']) > 2:
            history_points = drone['path_history'][-SHARED_PATH_POINTS:] if len(drone['path_history']) > SHARED_PATH_POINTS else drone['path_history']
            for point in history_points:
                x_idx = min(GRID_WIDTH - 1, max(0, int(point[0] / STEP_SIZE)))
                y_idx = min(GRID_HEIGHT - 1, max(0, int(point[1] / STEP_SIZE)))
                for i in range(max(0, x_idx-1), min(GRID_WIDTH, x_idx+2)):
                    for j in range(max(0, y_idx-1), min(GRID_HEIGHT, y_idx+2)):
                        risk_map[i, j] = max(0, risk_map[i, j] - ETA_SHARED * 0.5)
    return np.clip(risk_map, 0, 5)

def check_path_optimization_trigger(drone, risk_map, timestep):
    if 'last_replan' not in drone:
        drone['last_replan'] = 0
    if timestep - drone['last_replan'] < REPLAN_INTERVAL:
        return False
    if len(drone['log']) < 3:
        return False
    current_risk = drone['log'][-1]['Risk Value']
    previous_risk = drone['log'][-3]['Risk Value']
    risk_change = abs(current_risk - previous_risk)
    if risk_change > DYNAMIC_OPTIMIZATION_THRESHOLD:
        drone['last_replan'] = timestep
        return True
    if drone['path'] and len(drone['path']) > 0:
        next_point = drone['path'][0]
        x_idx = min(GRID_WIDTH - 1, max(0, int(next_point[0] / STEP_SIZE)))
        y_idx = min(GRID_HEIGHT - 1, max(0, int(next_point[1] / STEP_SIZE)))
        if risk_map[x_idx, y_idx] > RISK_THRESHOLD * 2.5:
            drone['last_replan'] = timestep
            return True
    return False

def create_drone_shape(x, y, phi, size=2.0, color='blue'):
    patches = []
    center = Circle((x, y), radius=0.3 * size, facecolor=color, edgecolor='black', alpha=0.7)
    patches.append(center)
    arm_length = 0.8 * size
    propeller_radius = 0.1 * size
    arms = [(arm_length, arm_length), (-arm_length, arm_length), (-arm_length, -arm_length), (arm_length, -arm_length)]
    rotation = np.array([[np.cos(phi), -np.sin(phi)], [np.sin(phi), np.cos(phi)]])
    for arm in arms:
        arm_rotated = np.dot(arm, rotation) + np.array([x, y])
        vertices = [(x, y), arm_rotated]
        codes = [Path.MOVETO, Path.LINETO]
        path = Path(vertices, codes)
        arm_patch = PathPatch(path, color='black', linewidth=0.5, alpha=0.7)
        patches.append(arm_patch)
        propeller = Circle(arm_rotated, radius=propeller_radius, facecolor='gray', edgecolor='black', alpha=0.7)
        patches.append(propeller)
    return patches

def initialize_visualization():
    plt.figure(figsize=(6, 6), dpi=300)
    plt.gca().set_facecolor('white')
    plt.xlim(0, MAP_WIDTH)
    plt.ylim(0, MAP_HEIGHT)
    plt.xlabel(get_label("X坐标/m", "X-coordinate/m"), fontsize=10)
    plt.ylabel(get_label("Y坐标/m", "Y-coordinate/m"), fontsize=10)

def update_visualization(risk_map, drones, timestep, output_dir="figures", display_interval=1):
    if not os.path.exists(output_dir):
        os.makedirs(output_dir)
    plt.clf()
    extent = [0, MAP_WIDTH, 0, MAP_HEIGHT]
    plt.imshow(risk_map, cmap=custom_cmap, interpolation='bilinear', extent=extent, origin='lower', alpha=0.9, vmin=0, vmax=5)
    plt.colorbar(label=get_label('风险等级', 'Risk Level'), shrink=0.7, aspect=15, pad=0.02, ticks=[0, 1, 2, 3, 4, 5])
    colors = ['blue', 'green', 'purple', 'orange', 'cyan']
    for i, drone in enumerate(drones):
        drone_color = colors[i % len(colors)]
        path_x, path_y = zip(*drone['path_history']) if drone['path_history'] else ([drone['x']], [drone['y']])
        plt.plot(path_y, path_x, color=drone_color, linewidth=1.5, 
                label=get_label(f'无人机 {i} 路径', f'UAV {i} Path') if timestep == 1 else "", alpha=0.8)
        drone_patches = create_drone_shape(drone['y'], drone['x'], drone['phi'], size=2.0, color=drone_color)
        for patch in drone_patches:
            plt.gca().add_patch(patch)
        if timestep == 1:
            plt.plot([], [], 's', color=drone_color, 
                    label=get_label(f'无人机 {i}', f'UAV {i}'), alpha=0.7)
        plt.plot(drone['target'][1], drone['target'][0], 'r*', markersize=5, 
                label=get_label(f'无人机 {i} 目标点', f'UAV {i} Target') if timestep == 1 else "", alpha=0.7)
        if 'priority' in drone:
            plt.annotate(get_label(f"优先级:{drone['priority']:.2f}", f"Priority:{drone['priority']:.2f}"), 
                         (drone['y'], drone['x']), 
                         xytext=(drone['y']+2, drone['x']+2),
                         fontsize=7, color=drone_color)
    if timestep == 1:
        plt.legend(loc='upper right', fontsize=8, framealpha=0.8)
    plt.xlabel(get_label("X坐标/m", "X-coordinate/m"), fontsize=10)
    plt.ylabel(get_label("Y坐标/m", "Y-coordinate/m"), fontsize=10)
    plt.grid(True, alpha=0.2)
    output_file = os.path.join(output_dir, f"timestep_{timestep:03d}.svg")
    plt.savefig(output_file, format='svg', dpi=300, bbox_inches='tight')
    if timestep % display_interval == 0:
        plt.pause(0.01)

def update_metrics_visualization(drones, timestep, output_dir="metrics"):
    if not os.path.exists(output_dir):
        os.makedirs(output_dir)
    if timestep % 10 == 0 and any(len(drone['log']) > 0 for drone in drones):
        plt.figure(figsize=(6, 5))
        for i, drone in enumerate(drones):
            if len(drone['log']) > 0:
                times = [log['Timestamp'] for log in drone['log']]
                risks = [log['Risk Value'] for log in drone['log']]
                plt.plot(times, risks, label=get_label(f'无人机 {i}', f'UAV {i}'))
        plt.xlabel(get_label('时间/s', 'Time/s'))
        plt.ylabel(get_label('风险值', 'Risk Value'))
        plt.grid(True, alpha=0.3)
        plt.legend()
        plt.tight_layout()
        risk_output_file = os.path.join(output_dir, f"risk_values_{timestep:03d}.svg")
        plt.savefig(risk_output_file, format='svg', dpi=300)
        plt.close()
        plt.figure(figsize=(6, 5))
        for i, drone in enumerate(drones):
            if len(drone['log']) > 0:
                times = [log['Timestamp'] for log in drone['log']]
                turns = [log['Turn Angle'] for log in drone['log']]
                plt.plot(times, turns, label=get_label(f'无人机 {i}', f'UAV {i}'))
        plt.xlabel(get_label('时间/s', 'Time/s'))
        plt.ylabel(get_label('转向角度/rad', 'Turn Angle/rad'))
        plt.grid(True, alpha=0.3)
        plt.legend()
        plt.tight_layout()
        turn_output_file = os.path.join(output_dir, f"turn_angles_{timestep:03d}.svg")
        plt.savefig(turn_output_file, format='svg', dpi=300)
        plt.close()

def initialize_drones(num_drones, map_width, map_height):
    drones = []
    for _ in range(num_drones):
        drone = {
            'x': np.random.uniform(0, map_width),
            'y': np.random.uniform(0, map_height),
            'v': np.random.uniform(12, 20),
            'phi': np.random.uniform(-np.pi, np.pi),
            'delta_phi': 0,
            'path': [],
            'reached': False,
            'target': (np.random.uniform(0, map_width), np.random.uniform(0, map_height)),
            'path_history': [],
            'log': [],
            'priority': 0.0,
            'last_replan': 0
        }
        drones.append(drone)
    return drones

def filter_path(path, min_distance):
    filtered_path = [path[0]]
    for point in path[1:]:
        if np.linalg.norm(np.array(filtered_path[-1]) - np.array(point)) >= min_distance:
            filtered_path.append(point)
    return filtered_path

def limit_velocity_change(current_velocity, target_velocity):
    velocity_change = np.clip(target_velocity - current_velocity, -A_MAX * T_INTERVAL, A_MAX * T_INTERVAL)
    return current_velocity + velocity_change

# 修复语法错误的函数
def limit_turning_angle(current_angle, target_angle):
    delta_angle = np.arctan2(np.sin(target_angle - current_angle), np.cos(target_angle - current_angle))
    delta_angle = np.clip(delta_angle, -DELTA_PHI_MAX, DELTA_PHI_MAX)
    return current_angle + delta_angle

def export_logs_to_file(drones, output_file="drone_logs.csv"):
    all_logs = []
    for drone_id, drone in enumerate(drones):
        for log_entry in drone['log']:
            log_entry['Drone ID'] = drone_id
            all_logs.append(log_entry)
    df = pd.DataFrame(all_logs)
    df.to_csv(output_file, index=False)
    print(f"日志已导出到 {output_file}")

def follow_path(drone, risk_map):
    if not drone['path']:
        return
    next_point = drone['path'][0]
    dx, dy = next_point[0] - drone['x'], next_point[1] - drone['y']
    distance_to_point = np.sqrt(dx*dx + dy*dy)
    if distance_to_point < 2.0:
        drone['path'].pop(0)
        if drone['path']:
            next_point = drone['path'][0]
            dx, dy = next_point[0] - drone['x'], next_point[1] - drone['y']
    target_phi = np.arctan2(dy, dx)
    drone['delta_phi'] = np.arctan2(np.sin(target_phi - drone['phi']), np.cos(target_phi - drone['phi']))
    drone['phi'] = limit_turning_angle(drone['phi'], target_phi)
    x_idx = min(GRID_WIDTH - 1, max(0, int(drone['x'] / STEP_SIZE)))
    y_idx = min(GRID_HEIGHT - 1, max(0, int(drone['y'] / STEP_SIZE)))
    local_risk = risk_map[x_idx, y_idx]
    risk_factor = max(0.5, 1.0 - local_risk / 10.0)
    distance_factor = min(1.0, distance_to_point / 10.0)
    target_velocity = V_REF * risk_factor * (0.5 + 0.5 * distance_factor)
    target_velocity = min(V_MAX, max(MIN_SPEED, target_velocity))
    drone['v'] = limit_velocity_change(drone['v'], target_velocity)
    drone['x'] += drone['v'] * np.cos(drone['phi']) * T_INTERVAL
    drone['y'] += drone['v'] * np.sin(drone['phi']) * T_INTERVAL
    drone['x'] = max(0, min(MAP_WIDTH - 1, drone['x']))
    drone['y'] = max(0, min(MAP_HEIGHT - 1, drone['y']))
    drone['path_history'].append((drone['x'], drone['y']))
    acceleration = (drone['v'] - drone.get('prev_v', drone['v'])) / T_INTERVAL
    drone['prev_v'] = drone['v']
    drone['log'].append({
        'Timestamp': len(drone['log']) * T_INTERVAL,
        'Speed': drone['v'],
        'Acceleration': acceleration,
        'Turn Angle': np.abs(drone['delta_phi']),
        'Risk Value': local_risk
    })

if __name__ == "__main__":
    risk_map = np.zeros((GRID_WIDTH, GRID_HEIGHT))
    drones = initialize_drones(NUM_DRONES, MAP_WIDTH, MAP_HEIGHT)

    for drone in drones:
        start, goal = (drone['x'], drone['y']), drone['target']
        path = global_path_planning(risk_map, start, goal, NUM_WAYPOINTS)
        filtered_path = filter_path(path, MIN_PATH_DISTANCE)
        drone['path'] = local_path_optimization(filtered_path, risk_map, RISK_THRESHOLD)
        drone['path_history'] = [(drone['x'], drone['y'])]

    initialize_visualization()

    timestep = 0
    while timestep < MAX_TIMESTEPS:
        timestep += 1
        
        risk_map = update_risk_field_shared(risk_map, drones)
        
        for drone in drones:
            drone['priority'] = calculate_priority(drone, drones, risk_map)
        
        collisions = check_collision(drones, SAFE_DISTANCE)
        if collisions:
            resolve_collision_optimized(drones, collisions)

        reached_count = 0
        for drone in drones:
            if drone['reached']:
                reached_count += 1
                continue
                
            if check_path_optimization_trigger(drone, risk_map, timestep):
                start = (drone['x'], drone['y'])
                path = global_path_planning(risk_map, start, drone['target'], NUM_WAYPOINTS)
                filtered_path = filter_path(path, MIN_PATH_DISTANCE)
                drone['path'] = local_path_optimization(filtered_path, risk_map, RISK_THRESHOLD)
                print(f"{get_label('无人机', 'UAV')} {drones.index(drone)} {get_label('在时间步', 'at timestep')} {timestep} {get_label('重新规划路径', 'replanned path')}")

            if not drone['path']:
                start = (drone['x'], drone['y'])
                path = global_path_planning(risk_map, start, drone['target'], NUM_WAYPOINTS)
                filtered_path = filter_path(path, MIN_PATH_DISTANCE)
                drone['path'] = local_path_optimization(filtered_path, risk_map, RISK_THRESHOLD)

            follow_path(drone, risk_map)
            
            distance_to_target = np.linalg.norm([drone['x'] - drone['target'][0], drone['y'] - drone['target'][1]])
            if distance_to_target < TARGET_TOLERANCE:
                drone['reached'] = True
                print(f"{get_label('无人机', 'UAV')} {drones.index(drone)} {get_label('在时间步', 'at timestep')} {timestep} {get_label('到达目标点', 'reached target')}")
                reached_count += 1

        update_visualization(risk_map, drones, timestep, output_dir="figures", display_interval=1)
        
        if timestep % 10 == 0:
            update_metrics_visualization(drones, timestep, output_dir="metrics")
            print(f"{get_label('时间步', 'Timestep')} {timestep}: {reached_count}/{NUM_DRONES} {get_label('架无人机已到达目标点', 'UAVs reached target')}")

        if reached_count == NUM_DRONES:
            print(f"{get_label('所有无人机已在', 'All UAVs reached targets in')} {timestep} {get_label('个时间步内到达目标点', 'timesteps')}")
            break

    update_metrics_visualization(drones, MAX_TIMESTEPS, output_dir="metrics")
    export_logs_to_file(drones)
    plt.show()
