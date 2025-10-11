# RoCo: Dialectic Multi-Robot Collaboration with Large Language Models

Codebase for paper: RoCo: Dialectic Multi-Robot Collaboration with Large Language Models

[Mandi Zhao](https://mandizhao.github.io), [Shreeya Jain](https://www.linkedin.com), [Shuran Song](https://www.cs.columbia.edu/~shurans/) 

[Arxiv](https://arxiv.org/abs/2307.04738) | [Project Website](https://project-roco.github.io) 

<img src="method.jpeg" alt="method" width="800"/>

## 🎯 Dual-Robot Navigation Demo

### Overview
This repository contains an enhanced dual-robot navigation demonstration built on top of the original RoCo project. The demo features two robots (Alice and Bob) navigating through a challenging obstacle course using A* pathfinding algorithm with MuJoCo physics simulation.

### Features
- **A* Path Planning**: Intelligent obstacle avoidance with optimal path finding
- **Dual-Robot Coordination**: Two robots navigating simultaneously without collisions
- **3D Visualization**: Real-time MuJoCo 3D viewer with interactive controls
- **2D Animation**: Matplotlib-based trajectory visualization and animation
- **Challenging Layout**: 6 strategically placed obstacles requiring complex navigation

### Demo Files
- **`my_demos/robot_navigation_demo.py`** - Main demo script (matplotlib animation)
- **`my_demos/robot_navigation_animation.gif`** - Robot motion animation
- **`my_demos/robot_navigation_trajectory.png`** - Trajectory plot
- **`fixed_mujoco_viewer.py`** - MuJoCo 3D Viewer version

### Running the Demo

#### Method 1: MuJoCo 3D Viewer (Recommended)
```bash
cd /Users/claire/co-robot-pathfinding
mjpython fixed_mujoco_viewer.py
```
- ✅ True 3D visualization
- ✅ Real-time robot movement
- ✅ Interactive controls (ESC to exit, Space to pause)

#### Method 2: Matplotlib Animation
```bash
cd /Users/claire/co-robot-pathfinding
python my_demos/robot_navigation_demo.py
```
- ✅ Generates GIF animation
- ✅ Cross-platform compatible

#### Method 3: Original Project Video Generation
```bash
cd /Users/claire/co-robot-pathfinding
python real_world/runners/run_nav_full.py
```
- ✅ Generates high-quality MP4 video
- ✅ Suitable for paper production

### Viewing Results
```bash
open my_demos/robot_navigation_animation.gif
open my_demos/robot_navigation_trajectory.png
```

### Technical Details

#### Robot Configuration
- **Alice**: Blue robot navigating from (-3.2, 2.0) to (3.0, 1.6)
- **Bob**: Green robot navigating from (-3.2, -2.3) to (3.2, -1.0)

#### Obstacle Layout
The environment features 6 strategically placed obstacles:
1. **Top-left obstacle**: Blocks upper-left area
2. **Top-right obstacle**: Blocks upper-right area  
3. **Central vertical obstacle**: Blocks center path
4. **Left horizontal obstacle**: Blocks left horizontal path
5. **Right horizontal obstacle**: Blocks right horizontal path
6. **Bottom obstacle**: Blocks lower area

#### Path Planning Algorithm
- **A* Algorithm**: Optimal pathfinding with obstacle avoidance
- **Grid Resolution**: 0.1m for precise navigation
- **Kinematic Control**: Direct position control using MuJoCo freejoint
- **Multi-robot Coordination**: Independent path planning for each robot

### Requirements
- Python 3.8+
- MuJoCo 2.3.0+
- NumPy, Matplotlib, ImageIO
- macOS users need `mjpython` for 3D viewer

### Installation
```bash
# Clone the repository
git clone https://github.com/huiwenxue122/co-robot-pathfinding.git
cd co-robot-pathfinding

# Install dependencies
pip install -r requirements.txt

# For macOS users, install MuJoCo viewer
pip install mujoco
```

### Project Structure
```
co-robot-pathfinding/
├── fixed_mujoco_viewer.py          # 3D Viewer (recommended)
├── my_demos/                       # Demo project
│   ├── robot_navigation_demo.py   # Matplotlib animation
│   ├── robot_navigation_animation.gif
│   └── robot_navigation_trajectory.png
├── real_world/                     # Original project code
│   ├── nav_world/nav_env.py        # Core navigation environment
│   ├── nav_world/room.xml          # 3D scene definition
│   └── runners/                    # Original project runners
└── README.md                       # This file
```

---

## Original Project Setup

### Setup conda environment and package installation

```bash
conda create -n roco python=3.8 
conda activate roco
```

### Install MuJoCo and dm_control

```bash
pip install mujoco==2.3.0
pip install dm_control==1.0.8 
```

**If you have M1 Macbook and would like to visualize the task scenes locally:**

Download the macOS-compatible `.dmg` file from MuJoCo release page, inside it should have a `MuJoCo.app` file that you can drag into your /Application folder, so it becomes just like other apps in your Mac. You could then open up the app and drag xml files in it. Find more information in the official documentation.

### Install other packages

```bash
pip install -r requirements.txt
```

### Acquire OpenAI/Claude API Keys

This is required for prompting GPTs or Claude LLMs. You don't necessarily need both of them. Put your key string somewhere safely in your local repo, and provide a file path (something like `./roco/openai_key.json`) and load them in the scripts. Example code snippet:

```python
import openai  
openai.api_key = YOUR_OPENAI_KEY

import anthropic
client = anthropic.Client(api_key=YOUR_CLAUDE_KEY)
streamed = client.completion_stream(...)  
```

## Usage

### Run multi-robot dialog on the PackGrocery Task using the latest GPT-4 model

```bash
$ conda activate roco
(roco) $ python run_dialog.py --task pack -llm gpt-4
```

## Contact

Please direct to Mandi Zhao. If you are interested in contributing or collaborating, please feel free to reach out! I'm more than happy to chat and brainstorm together.

## Cite

```bibtex
@misc{mandi2023roco,
      title={RoCo: Dialectic Multi-Robot Collaboration with Large Language Models}, 
      author={Zhao Mandi and Shreeya Jain and Shuran Song},
      year={2023},
      eprint={2307.04738},
      archivePrefix={arXiv},
      primaryClass={cs.RO}
}
```