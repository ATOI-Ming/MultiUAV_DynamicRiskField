import numpy as np
from chapter1_dynamic_risk import calculate_risk_field, update_risk_field
import matplotlib.pyplot as plt
from matplotlib.colors import LinearSegmentedColormap
from matplotlib.patches import PathPatch, Circle
from matplotlib.path import Path
import os

# 字体设置（复用文件4的逻辑）
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

# 参数配置（参考文件4）
MAP_WIDTH = 100
MAP_HEIGHT = 100
STEP_SIZE = 0.5
GRID_WIDTH = int(MAP_WIDTH / STEP_SIZE)
GRID_HEIGHT = int(MAP_HEIGHT / STEP_SIZE)
ALPHA = 0.1
ETA = 0.2
KAPPA = 0.1
EPSILON = 0.1
A = 1.0
V_REF = 12.0  # 参考速度 (m/s)
V_MAX = 20.0  # 最大速度
MIN_SPEED = 8.0  # 调整后的减速速度 (m/s)
DELTA_PHI_MAX = np.pi / 6  # 最大转向角度

# 自定义颜色映射（复用文件4）
custom_cmap = LinearSegmentedColormap.from_list(
    "custom_cmap", ["#000033", "#003366", "#00CCCC", "#FFFF99", "#FF6600", "#CC0000", "#660000"]
)

def get_label(zh_text, en_text):
    return zh_text if USE_CHINESE else en_text

# 无人机形状函数（复用文件4）
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

# 可视化函数（新增无人机形状）
def visualize_risk_factor(risk_map, drone, title, v, delta_phi, output_dir="risk_factors", filename="risk_factor"):
    if not os.path.exists(output_dir):
        os.makedirs(output_dir)
    
    plt.figure(figsize=(6, 6), dpi=300)
    plt.imshow(risk_map, cmap=custom_cmap, interpolation='bilinear', extent=[0, MAP_WIDTH, 0, MAP_HEIGHT], 
               origin='lower', alpha=0.9, vmin=0, vmax=5)
    plt.colorbar(label=get_label('风险等级', 'Risk Level'), shrink=0.7, aspect=15, pad=0.02, ticks=[0, 1, 2, 3, 4, 5])
    
    # 添加无人机形状
    drone_patches = create_drone_shape(drone['x'], drone['y'], drone['phi'], size=2.0, color='blue')
    for patch in drone_patches:
        plt.gca().add_patch(patch)
    
    plt.title(f"{title}\n{get_label(f'速度 v = {v:.1f} m/s, 转向角 δφ = {delta_phi:.2f} rad', f'Speed v = {v:.1f} m/s, Turn Angle δφ = {delta_phi:.2f} rad')}", 
              fontsize=10, pad=10)
    plt.xlabel(get_label("X坐标/m", "X-coordinate/m"), fontsize=10)
    plt.ylabel(get_label("Y坐标/m", "Y-coordinate/m"), fontsize=10)
    plt.grid(True, alpha=0.2)
    output_file = os.path.join(output_dir, f"{filename}.svg")
    plt.savefig(output_file, format='svg', bbox_inches='tight')
    plt.close()
    print(f"已保存图像: {output_file}")

# 主程序：生成6个风险场示意图
if __name__ == "__main__":
    # 无人机初始位置（地图中心）
    drone_base = {
        'x': MAP_WIDTH / 2,
        'y': MAP_HEIGHT / 2,
        'phi': 0.0,  # 初始朝向（沿X轴正方向）
        'vx': 0.0,
        'vy': 0.0
    }

    # 场景配置
    scenarios = [
        {"title": get_label("参考速度下无转向", "Reference Speed, No Turn"), 
         "v": V_REF, "delta_phi": 0.0, "phi": 0.0, "filename": "reference_no_turn"},
        {"title": get_label("仅加速", "Acceleration Only"), 
         "v": V_MAX, "delta_phi": 0.0, "phi": 0.0, "filename": "acceleration_only"},
        {"title": get_label("仅减速", "Deceleration Only"), 
         "v": MIN_SPEED, "delta_phi": 0.0, "phi": 0.0, "filename": "deceleration_only"},
        {"title": get_label("仅转向", "Turn Only"), 
         "v": V_REF, "delta_phi": DELTA_PHI_MAX, "phi": DELTA_PHI_MAX, "filename": "turn_only"},
        {"title": get_label("加速和转向", "Acceleration and Turn"), 
         "v": V_MAX, "delta_phi": DELTA_PHI_MAX, "phi": DELTA_PHI_MAX, "filename": "acceleration_turn"},
        {"title": get_label("减速和转向", "Deceleration and Turn"), 
         "v": MIN_SPEED, "delta_phi": DELTA_PHI_MAX, "phi": DELTA_PHI_MAX, "filename": "deceleration_turn"}
    ]

    # 为每个场景生成风险场并可视化
    for scenario in scenarios:
        # 配置无人机参数
        drone = drone_base.copy()
        drone['v'] = scenario["v"]
        drone['delta_phi'] = scenario["delta_phi"]
        drone['phi'] = scenario["phi"]  # 调整朝向与转向角度一致
        drone['vx'] = drone['v'] * np.cos(drone['phi'])
        drone['vy'] = drone['v'] * np.sin(drone['phi'])

        # 初始化风险场
        risk_map = np.zeros((GRID_WIDTH, GRID_HEIGHT))
        
        # 更新风险场（单一无人机）
        risk_map = update_risk_field(risk_map, [drone], ALPHA, ETA, KAPPA, EPSILON, A, STEP_SIZE, MAP_WIDTH, MAP_HEIGHT)
        
        # 可视化
        visualize_risk_factor(risk_map, drone, scenario["title"], drone['v'], drone['delta_phi'], filename=scenario["filename"])

    print("所有风险场示意图生成完成！")
