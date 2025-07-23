# Multi-UAV Path Planning and Collaboration with Dynamic Risk Field

This repository contains the implementation code and supporting data for the paper *"Multi-UAV Path Planning and Collaboration Based on Dynamic Risk Field"*. The project develops a novel framework for multi-unmanned aerial vehicle (UAV) path planning and collaboration, utilizing a dynamic risk field model to enhance safety and efficiency in complex environments.

## Overview

The research addresses the challenge of multi-UAV coordination by introducing a dynamic risk field model (Equations 1-5), which integrates real-time environmental risks, collision avoidance (Equations 7-11, 13-15), and hierarchical path planning (Equations 16-20). Key contributions include:

- A dynamic risk field to assess and mitigate threats.
- A priority-based collision resolution mechanism.
- A comparison of the proposed method with A* and potential field algorithms.
- Analysis of risk factors such as speed and turn angle.

The codebase supports simulation experiments, visualization of trajectories, and performance metrics, with outputs aligned with the paper's figures (e.g., Figures 1-4) and tables (e.g., Tables 2-3).

## Installation

To set up the environment and run the code, follow these steps:

1. Clone the repository:
   ```bash
   git clone https://github.com/your-username/MultiUAV_DynamicRiskField.git
   cd MultiUAV_DynamicRiskField
   ```

2. Install the required dependencies:
   ```bash
   pip install -r requirements.txt
   ```

### Requirements
- Python 3.8+
- numpy==1.21.0
- matplotlib==3.4.0
- scipy==1.7.0
- pandas==1.3.0

## Files

### Source Code (`src/`)
- `chapter1_dynamic_risk.py`: Implements the dynamic risk field model (Equations 1-5), calculating risk values based on UAV states and environmental factors.
- `chapter2_collaboration.py`: Handles collision detection and resolution (Equations 7-11), including priority-based optimization (Equations 13-15).
- `chapter3_path_planning.py`: Provides global and local path planning algorithms (Equations 16-20) for UAV navigation.
- `chapter4_main_experiment.py`: Integrates the above modules for multi-UAV simulation, generating trajectories and logs.
- `chapter5_risk_factor_analysis.py`: Analyzes the impact of speed and turn angle on the risk field (related to Figures 1-3).
- `chapter6_method_comparison.py`: Compares the proposed method with A* and potential field methods (related to Figure 4 and Table 3).
- `utils.py`: Implements A* and potential field path planning algorithms for method comparison.

### Paper and Configuration
- `paper/MultiUAV_DynamicRiskField.pdf`: The full research paper detailing the methodology and results.
- `requirements.txt`: Lists Python package dependencies.
- `LICENSE`: MIT License for open-source usage.
- `.gitignore`: Specifies files to ignore (e.g., `.pyc`, `*.svg`).

### Data and Visualizations
- `data/`: Contains CSV files (e.g., `drone_logs.csv`) with real-time UAV experiment data (position, velocity, risk values).
- `figures/`: Includes SVG files (e.g., `timestep_001.svg`) showing algorithm display effects on the map at each step (related to Figures 1-3).
- `metrics/`: Contains SVG files (e.g., `risk_values_010.svg`, `turn_angles_010.svg`) with UAV performance parameters at each step (related to Table 2).
- `risk_factors/`: Holds SVG files (e.g., `reference_no_turn.svg`) visualizing speed and turn angle effects on the risk field (related to Figures 1-3).
- `maps/`: Includes SVG files (e.g., `proposed_map_timestep_001.svg`) for each step's map in the comparison experiments (related to Figure 4).
- `results/`: Curated SVG files from `figures/` and `metrics/` representing effective experimental results for the paper (related to Figures 4 and Table 3).

## Usage

1. Run the main simulation:
   ```bash
   python src/chapter4_main_experiment.py
   ```
   Generates trajectories, logs, and initial visualizations in `figures/`, `metrics/`, and `data/`.

2. Analyze risk factors:
   ```bash
   python src/chapter5_risk_factor_analysis.py
   ```
   Produces risk field visualizations in `risk_factors/`.

3. Compare methods:
   ```bash
   python src/chapter6_method_comparison.py
   ```
   Generates comparison maps in `maps/` and performance metrics.

4. Verify outputs:
   - Check generated files in `data/`, `figures/`, `metrics/`, `risk_factors/`, and `maps/`.
   - Use `results/` for paper-ready visuals.

## Outputs
- `data/drone_logs.csv`: Real-time UAV data for simulation validation.
- `figures/timestep_XXX.svg`: Snapshots of UAV trajectories and risk fields at each timestep.
- `metrics/risk_values_XXX.svg`, `turn_angles_XXX.svg`: Performance curves for risk and maneuverability.
- `risk_factors/reference_no_turn.svg`, `acceleration_only.svg`, etc.: Visualizations of risk factor impacts.
- `maps/proposed_map_timestep_XXX.svg`, `astar_map_timestep_XXX.svg`, `potential_map_timestep_XXX.svg`: Maps for method comparison.
- `results/`: Selected figures and metrics for paper inclusion.

*Note: Due to the large number of SVG files, only key frames are uploaded. Full visualizations are available at [Google Drive link] (to be updated).*

## License
This project is licensed under the MIT License, allowing free use, modification, and distribution with proper attribution.

## Acknowledgments
This work was supported by [insert funding or institutional acknowledgments, if any]. Special thanks to the xAI team for their computational resources and feedback.

## Contact
For questions or collaborations, please open an issue or contact [your-email@example.com].

## Future Work
- Implement dynamic threat sources (Equation 6) in `chapter4_main_experiment.py`.
- Add a summary chart in `maps/` for method comparison.
- Enhance `utils.py` with parameter sensitivity analysis for A* and potential field methods.
