import numpy as np
from heapq import heappush, heappop

def a_star_path_planning(start, goal, risk_map, step_size, map_width, map_height, risk_threshold):
    grid_width = int(map_width / step_size)
    grid_height = int(map_height / step_size)
    start_grid = (int(start[0] / step_size), int(start[1] / step_size))
    goal_grid = (int(goal[0] / step_size), int(goal[1] / step_size))

    open_set = [(0, start_grid)]
    came_from = {}
    g_score = {start_grid: 0}
    f_score = {start_grid: np.hypot(goal_grid[0] - start_grid[0], goal_grid[1] - start_grid[1])}
    explored_nodes = set([start_grid])

    while open_set:
        current = heappop(open_set)[1]
        if current == goal_grid:
            path = []
            while current in came_from:
                path.append((current[0] * step_size, current[1] * step_size))
                current = came_from[current]
            path.append((start[0], start[1]))
            return path[::-1], explored_nodes

        for dx, dy in [(0, 1), (1, 0), (0, -1), (-1, 0), (1, 1), (1, -1), (-1, 1), (-1, -1)]:
            neighbor = (current[0] + dx, current[1] + dy)
            if (0 <= neighbor[0] < grid_width and 0 <= neighbor[1] < grid_height):
                risk_cost = risk_map[neighbor[0], neighbor[1]] * 5 if risk_map[neighbor[0], neighbor[1]] < risk_threshold else float('inf')
                tentative_g = g_score[current] + np.hypot(dx, dy) * step_size + risk_cost
                if neighbor not in g_score or tentative_g < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g
                    f_score[neighbor] = tentative_g + np.hypot(goal_grid[0] - neighbor[0], goal_grid[1] - neighbor[1]) * step_size
                    heappush(open_set, (f_score[neighbor], neighbor))
                    explored_nodes.add(neighbor)
    return [], explored_nodes

def potential_field_path_planning(start, goal, risk_map, drones, step_size, map_width, map_height, max_steps=500):
    path = [start]
    current = list(start)
    eta_attr, eta_rep, d_goal = 10.0, 30.0, 15.0  # 进一步增强引力和斥力
    visited = set()

    for _ in range(max_steps):
        dist_to_goal = np.linalg.norm(np.array(goal) - np.array(current))
        if dist_to_goal < 1.5:
            path.append(goal)
            break

        attractive_force = eta_attr * (np.array(goal) - np.array(current)) / max(dist_to_goal, 0.1)
        if dist_to_goal > d_goal:
            attractive_force *= (d_goal / dist_to_goal)

        repulsive_force = np.zeros(2)
        x_idx, y_idx = int(current[0] / step_size), int(current[1] / step_size)
        x_idx = min(int(map_width / step_size) - 1, max(0, x_idx))
        y_idx = min(int(map_height / step_size) - 1, max(0, y_idx))
        risk_factor = risk_map[x_idx, y_idx] * 15.0  # 增强风险场影响
        for d in drones:
            if d['x'] != current[0] or d['y'] != current[1]:
                dist = np.linalg.norm(np.array([d['x'], d['y']]) - np.array(current))
                if dist < 30:  # 增大斥力范围
                    repulsive_force += eta_rep * (np.array(current) - np.array([d['x'], d['y']])) / (dist**2 + 0.1)
        repulsive_force += risk_factor * np.random.randn(2) * 0.3  # 增强随机扰动

        total_force = attractive_force + repulsive_force
        step = total_force / np.linalg.norm(total_force + 0.1) * step_size
        current[0] = np.clip(current[0] + step[0], 0, map_width - 1)
        current[1] = np.clip(current[1] + step[1], 0, map_height - 1)
        
        current_tuple = (round(current[0], 2), round(current[1], 2))
        if current_tuple in visited:
            step += np.random.randn(2) * step_size * 1.5  # 更大随机跳出
            current[0] = np.clip(current[0] + step[0], 0, map_width - 1)
            current[1] = np.clip(current[1] + step[1], 0, map_height - 1)
        visited.add(current_tuple)
        path.append((current[0], current[1]))

    return path
