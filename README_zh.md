# 基于动态风险场的多无人机路径规划与协作

本仓库包含论文《基于动态风险场的多无人机路径规划与协作》的实现代码及配套数据。该项目开发了一个新颖的多无人机（UAV）路径规划与协作框架，利用动态风险场模型提高复杂环境下的安全性和效率。

## 概述

本研究通过引入动态风险场模型（公式1-5）解决多无人机协同的挑战，该模型整合了实时环境风险、碰撞规避（公式7-11、13-15）和分层路径规划（公式16-20）。主要贡献包括：

- 用于评估和缓解威胁的动态风险场。
- 基于优先级的碰撞解决机制。
- 所提方法与A*和势场算法的对比。
- 对速度、转向角等风险因素的分析。

代码库支持仿真实验、轨迹可视化和性能指标计算，输出结果与论文中的图表（如图1-4）和表格（如表2-3）一致。

## 安装

如需搭建环境并运行代码，请遵循以下步骤：

1. 克隆仓库：
   ```bash
   git clone https://github.com/your-username/MultiUAV_DynamicRiskField.git
   cd MultiUAV_DynamicRiskField
   ```

2. 安装所需依赖：
   ```bash
   pip install -r requirements.txt
   ```

### 依赖要求
- Python 3.8及以上版本
- numpy==1.21.0
- matplotlib==3.4.0
- scipy==1.7.0
- pandas==1.3.0

## 文件说明

### 源代码（`src/`）
- `chapter1_dynamic_risk.py`：实现动态风险场模型（公式1-5），基于无人机状态和环境因素计算风险值。
- `chapter2_collaboration.py`：处理碰撞检测与解决（公式7-11），包括基于优先级的优化（公式13-15）。
- `chapter3_path_planning.py`：提供用于无人机导航的全局和局部路径规划算法（公式16-20）。
- `chapter4_main_experiment.py`：整合上述模块进行多无人机仿真，生成轨迹和日志。
- `chapter5_risk_factor_analysis.py`：分析速度和转向角对风险场的影响（与图1-3相关）。
- `chapter6_method_comparison.py`：将所提方法与A*和势场方法进行对比（与图4和表3相关）。
- `utils.py`：实现A*和势场路径规划算法，用于方法对比。

### 论文与配置
- `paper/MultiUAV_DynamicRiskField.pdf`：详细阐述方法和结果的完整研究论文。
- `requirements.txt`：列出Python包依赖。
- `LICENSE`：开源使用的MIT许可证。
- `.gitignore`：指定需忽略的文件（如`.pyc`、`*.svg`）。

### 数据与可视化
- `data/`：包含CSV文件（如`drone_logs.csv`），其中存储无人机实时实验数据（位置、速度、风险值）。
- `figures/`：包含SVG文件（如`timestep_001.svg`），展示每一步算法在地图上的显示效果（与图1-3相关）。
- `metrics/`：包含SVG文件（如`risk_values_010.svg`、`turn_angles_010.svg`），其中包含每一步无人机的性能参数（与表2相关）。
- `risk_factors/`：存储SVG文件（如`reference_no_turn.svg`），可视化速度和转向角对风险场的影响（与图1-3相关）。
- `maps/`：包含SVG文件（如`proposed_map_timestep_001.svg`），用于对比实验中每一步的地图（与图4相关）。
- `results/`：来自`figures/`和`metrics/`的精选SVG文件，代表论文的有效实验结果（与图4和表3相关）。

## 使用方法

1. 运行主仿真：
   ```bash
   python src/chapter4_main_experiment.py
   ```
   在`figures/`、`metrics/`和`data/`中生成轨迹、日志和初始可视化结果。

2. 分析风险因素：
   ```bash
   python src/chapter5_risk_factor_analysis.py
   ```
   在`risk_factors/`中生成风险场可视化结果。

3. 对比方法：
   ```bash
   python src/chapter6_method_comparison.py
   ```
   在`maps/`中生成对比地图和性能指标。

4. 验证输出：
   - 检查`data/`、`figures/`、`metrics/`、`risk_factors/`和`maps/`中生成的文件。
   - 使用`results/`获取可用于论文的可视化结果。

## 输出结果
- `data/drone_logs.csv`：用于仿真验证的无人机实时数据。
- `figures/timestep_XXX.svg`：每一时间步无人机轨迹和风险场的快照。
- `metrics/risk_values_XXX.svg`、`turn_angles_XXX.svg`：风险和机动性的性能曲线。
- `risk_factors/reference_no_turn.svg`、`acceleration_only.svg`等：风险因素影响的可视化结果。
- `maps/proposed_map_timestep_XXX.svg`、`astar_map_timestep_XXX.svg`、`potential_map_timestep_XXX.svg`：用于方法对比的地图。
- `results/`：入选论文的精选图表和指标。

*注：由于SVG文件数量较多，仅上传关键帧。*

## 许可证
本项目采用MIT许可证授权，允许在适当署名的情况下自由使用、修改和分发。

## 致谢
本工作得到国家科技部国家重点研发计划课题的支持，特别感谢清华大学启元实验室提供的计算资源和反馈。。

## 联系方式
如有问题或合作意向，请提交issue或联系[1757772673@qq.com]。

## 未来工作
- 在`chapter4_main_experiment.py`中实现动态威胁源（公式6）。
- 在`maps/`中添加方法对比的汇总图表。
- 增强`utils.py`，增加A*和势场方法的参数敏感性分析。

