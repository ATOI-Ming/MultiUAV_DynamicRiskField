import numpy as np
import time
import os
import matplotlib.pyplot as plt
from matplotlib.colors import LinearSegmentedColormap
from matplotlib.patches import Circle, PathPatch, Path
from main import (initialize_drones, update_risk_field_shared, check_collision, 
                 resolve_collision_optimized, follow_path, calculate_priority,
                 global_path_planning, filter_path, local_path_optimization,
                 check_path_optimization_trigger, MAP_WIDTH, MAP_HEIGHT, 
                 STEP_SIZE, GRID_WIDTH, GRID_HEIGHT, MAX_TIMESTEPS, 
                 TARGET_TOLERANCE, RISK_THRESHOLD, NUM_WAYPOINTS, 
                 MIN_PATH_DISTANCE, T_INTERVAL)
from utils import a_star_path_planning, potential_field_path_planning

# 场景配置
SCENARIO = {"num_drones": 6, "safe_distance": 3}

custom_cmap = LinearSegmentedColormap.from_list(
    "custom_cmap", ["#000033", "#003366", "#00CCCC", "#FFFF99", "#FF6600", "#CC0000", "#660000"]
)

def calculate_metrics(drones, completion_timestep, total_compute_time, total_collisions):
    reached_count = sum(1 for d in drones if np.linalg.norm([d['x'] - d['target'][0], d['y'] - d['target'][1]]) < TARGET_TOLERANCE)
    num_drones = len(drones)
    success_rate = reached_count / num_drones if num_drones > 0 else 0
    avg_compute_time = total_compute_time / completion_timestep if completion_timestep > 0 else 0
    avg_path_length = np.mean([calculate_path_length(d['path_history']) for d in drones if np.linalg.norm([d['x'] - d['target'][0], d['y'] - d['target'][1]]) < TARGET_TOLERANCE]) if reached_count > 0 else 0
    return {
        "success_rate": success_rate,
        "avg_compute_time": avg_compute_time,
        "avg_path_length": avg_path_length,
        "collision_count": total_collisions,
        "completion_time": completion_timestep
    }

def calculate_path_length(path):
    if len(path) < 2:
        return 0
    length = 0
    for i in range(len(path) - 1):
        length += np.sqrt((path[i+1][0] - path[i][0])**2 + (path[i+1][1] - path[i][1])**2)
    return length

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

def visualize_map(method, drones, risk_map, timestep, total_collisions, explored_nodes=None, output_dir="maps"):
    if not os.path.exists(output_dir):
        os.makedirs(output_dir)
    
    plt.figure(figsize=(6, 6), dpi=300)
    plt.xlim(0, MAP_WIDTH)
    plt.ylim(0, MAP_HEIGHT)
    colors = ['blue', 'green', 'purple', 'orange', 'cyan', 'red']
    
    if method == "proposed":
        plt.imshow(risk_map, cmap=custom_cmap, interpolation='bilinear', 
                   extent=[0, MAP_WIDTH, 0, MAP_HEIGHT], origin='lower', alpha=0.9, vmin=0, vmax=5)
        plt.colorbar(label="风险等级", shrink=0.7, aspect=15, pad=0.02, ticks=[0, 1, 2, 3, 4, 5])
        plt.title("本文方法 - 动态风险场")
    
    elif method == "astar":
        obstacle_map = (risk_map > RISK_THRESHOLD).astype(float)
        plt.imshow(obstacle_map, cmap='gray', interpolation='nearest', 
                   extent=[0, MAP_WIDTH, 0, MAP_HEIGHT], origin='lower', alpha=0.5)
        if explored_nodes:
            explored_x, explored_y = zip(*[(x * STEP_SIZE, y * STEP_SIZE) for x, y in explored_nodes])
            plt.scatter(explored_y, explored_x, c='lightblue', s=5, alpha=0.3, label="探索节点")
        plt.colorbar(label="障碍物 (1) / 可通行 (0)", shrink=0.7, aspect=15, pad=0.02)
        plt.title("A*算法 - 网格搜索路径")
    
    elif method == "potential":
        potential_field = np.zeros((GRID_WIDTH, GRID_HEIGHT))
        for drone in drones:
            for i in range(GRID_WIDTH):
                for j in range(GRID_HEIGHT):
                    pos = np.array([i * STEP_SIZE, j * STEP_SIZE])
                    dist_drone = np.linalg.norm(pos - np.array([drone['x'], drone['y']]))
                    dist_goal = np.linalg.norm(pos - np.array(drone['target']))
                    if dist_drone > 0.1 and dist_drone < 30:  # 增大斥力范围
                        potential_field[i, j] += 30.0 / dist_drone  # 增强斥力
                    if dist_goal > 0.1 and dist_goal < 50:  # 增大引力范围
                        potential_field[i, j] += -20.0 / dist_goal  # 增强引力
        plt.imshow(potential_field, cmap='RdBu', interpolation='bilinear', 
                   extent=[0, MAP_WIDTH, 0, MAP_HEIGHT], origin='lower', alpha=0.9, vmin=-100, vmax=100)
        plt.colorbar(label="势场强度 (引力:蓝 / 斥力:红)", shrink=0.7, aspect=15, pad=0.02)
        plt.title("人工势场法 - 引力与斥力引导")
    
    for i, drone in enumerate(drones):
        color = colors[i % len(colors)]
        path_x, path_y = zip(*drone['path_history']) if drone['path_history'] else ([drone['x']], [drone['y']])
        plt.plot(path_y, path_x, color=color, linewidth=1.5, label=f"无人机 {i} 路径")
        drone_patches = create_drone_shape(drone['y'], drone['x'], drone['phi'], size=2.0, color=color)
        for patch in drone_patches:
            plt.gca().add_patch(patch)
        plt.plot(drone['target'][1], drone['target'][0], 'r*', markersize=5, label=f"无人机 {i} 目标" if i == 0 else "")
    
    plt.xlabel("X坐标/m", fontsize=10)
    plt.ylabel("Y坐标/m", fontsize=10)
    plt.legend(loc='upper right', fontsize=8, framealpha=0.8)
    plt.grid(True, alpha=0.2)
    save_path = f"{output_dir}/{method}_map_timestep_{timestep:03d}.svg"
    plt.savefig(save_path, format='svg', dpi=300, bbox_inches='tight')
    print(f"地图已保存至: {os.path.abspath(save_path)}")
    plt.close()

def run_simulation(method, scenario_config):
    risk_map = np.zeros((GRID_WIDTH, GRID_HEIGHT))
    drones = initialize_drones(scenario_config["num_drones"], MAP_WIDTH, MAP_HEIGHT)
    
    planning_time = 0
    explored_nodes = set()  # 记录 A* 探索的节点
    for drone in drones:
        start_time = time.time()
        start, goal = (drone['x'], drone['y']), drone['target']
        if method == "proposed":
            path = global_path_planning(risk_map, start, goal, NUM_WAYPOINTS)
            filtered_path = filter_path(path, MIN_PATH_DISTANCE)
            drone['path'] = local_path_optimization(filtered_path, risk_map, RISK_THRESHOLD)
        elif method == "astar":
            drone['path'], nodes = a_star_path_planning(start, goal, risk_map, STEP_SIZE, MAP_WIDTH, MAP_HEIGHT, RISK_THRESHOLD)
            explored_nodes.update(nodes)
        elif method == "potential":
            drone['path'] = potential_field_path_planning(start, goal, risk_map, drones, STEP_SIZE, MAP_WIDTH, MAP_HEIGHT, max_steps=500)
        planning_time += time.time() - start_time
        drone['path_history'] = [(drone['x'], drone['y'])]
    
    timestep = 0
    reached_count = 0
    total_compute_time = planning_time
    total_collisions = 0
    completion_timestep = 0
    
    while timestep < MAX_TIMESTEPS:
        timestep += 1
        start_time = time.time()
        
        risk_map = update_risk_field_shared(risk_map, drones)
        if method == "proposed":
            for drone in drones:
                drone['priority'] = calculate_priority(drone, drones, risk_map)
        
        collisions = check_collision(drones, scenario_config["safe_distance"])
        total_collisions += len(collisions)
        if collisions:
            if method == "proposed":
                resolve_collision_optimized(drones, collisions)
            else:
                for i, j in collisions:
                    drones[i]['v'] = max(0.5, drones[i]['v'] * 0.8)
                    drones[j]['v'] = max(0.5, drones[j]['v'] * 0.8)
                    if method == "astar":
                        drones[i]['path'], nodes_i = a_star_path_planning((drones[i]['x'], drones[i]['y']), drones[i]['target'], risk_map, STEP_SIZE, MAP_WIDTH, MAP_HEIGHT, RISK_THRESHOLD)
                        drones[j]['path'], nodes_j = a_star_path_planning((drones[j]['x'], drones[j]['y']), drones[j]['target'], risk_map, STEP_SIZE, MAP_WIDTH, MAP_HEIGHT, RISK_THRESHOLD)
                        explored_nodes.update(nodes_i)
                        explored_nodes.update(nodes_j)
                    elif method == "potential":
                        drones[i]['path'] = potential_field_path_planning((drones[i]['x'], drones[i]['y']), drones[i]['target'], risk_map, drones, STEP_SIZE, MAP_WIDTH, MAP_HEIGHT)
                        drones[j]['path'] = potential_field_path_planning((drones[j]['x'], drones[j]['y']), drones[j]['target'], risk_map, drones, STEP_SIZE, MAP_WIDTH, MAP_HEIGHT)
        
        for drone in drones:
            if np.linalg.norm([drone['x'] - drone['target'][0], drone['y'] - drone['target'][1]]) < TARGET_TOLERANCE:
                if not drone.get('reached', False):
                    drone['reached'] = True
                    reached_count += 1
                    if reached_count == scenario_config["num_drones"]:
                        completion_timestep = timestep
                continue
            if method == "proposed" and check_path_optimization_trigger(drone, risk_map, timestep):
                start = (drone['x'], drone['y'])
                path = global_path_planning(risk_map, start, drone['target'], NUM_WAYPOINTS)
                filtered_path = filter_path(path, MIN_PATH_DISTANCE)
                drone['path'] = local_path_optimization(filtered_path, risk_map, RISK_THRESHOLD)
            
            follow_path(drone, risk_map)
        
        total_compute_time += time.time() - start_time
        
        if timestep % 10 == 0 or reached_count == scenario_config["num_drones"]:
            visualize_map(method, drones, risk_map, timestep, total_collisions, explored_nodes if method == "astar" else None)
        
        if reached_count == scenario_config["num_drones"]:
            break
    
    if completion_timestep == 0:
        completion_timestep = timestep
    return calculate_metrics(drones, completion_timestep, total_compute_time, total_collisions)

if __name__ == "__main__":
    methods = ["proposed", "astar", "potential"]
    results = {}
    
    print("场景: 密集冲突")
    for method in methods:
        print(f"\n方法: {method}")
        metrics = run_simulation(method, SCENARIO)
        results[method] = metrics
        print(f"  任务成功率: {metrics['success_rate']:.2%}")
        print(f"  平均计算时间: {metrics['avg_compute_time']:.4f} s")
        print(f"  平均路径长度: {metrics['avg_path_length']:.2f} m")
        print(f"  冲突次数: {metrics['collision_count']}")
        print(f"  任务完成时间: {metrics['completion_time']} 时间步")
