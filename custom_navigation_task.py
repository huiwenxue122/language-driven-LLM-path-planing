#!/usr/bin/env python3
"""
自定义导航任务
两个地面机器人在有障碍物的房间中导航到目标地点
"""
import os
import numpy as np
import mujoco
import dm_control.mujoco as dm_mujoco
from dm_control import mujoco as dm_mujoco
from typing import Dict, List, Tuple, Any
import matplotlib.pyplot as plt

class CustomNavigationTask:
    """自定义导航任务类"""
    
    def __init__(self, xml_path: str = "custom_navigation.xml"):
        """初始化导航任务"""
        self.xml_path = xml_path
        self.model = None
        self.data = None
        self.robots = {}
        self.obstacles = []
        self.targets = []
        
    def create_navigation_scene(self):
        """创建导航场景XML"""
        xml_content = """
<mujoco model="custom_navigation">
  <compiler angle="radian" autolimits="true"/>
  <option timestep="0.002" gravity="0 0 -9.81"/>
  
  <visual>
    <headlight diffuse="0.6 0.6 0.6" ambient="0.1 0.1 0.1" specular="0 0 0"/>
    <rgba haze="0.15 0.25 0.35 1"/>
    <global azimuth="120" elevation="-20"/>
  </visual>

  <worldbody>
    <!-- 光照 -->
    <light pos="0 0 2" dir="0 0 -1" directional="true"/>
    
    <!-- 地面 -->
    <geom name="floor" pos="0 0 -0.5" size="0 0 0.05" type="plane" rgba="0.5 0.5 0.5 1"/>
    
    <!-- 房间边界 -->
    <body name="room" pos="0 0 0">
      <!-- 墙壁 -->
      <geom name="wall_north" pos="0 2 0.5" size="2 0.1 0.5" type="box" rgba="0.8 0.8 0.8 1"/>
      <geom name="wall_south" pos="0 -2 0.5" size="2 0.1 0.5" type="box" rgba="0.8 0.8 0.8 1"/>
      <geom name="wall_east" pos="2 0 0.5" size="0.1 2 0.5" type="box" rgba="0.8 0.8 0.8 1"/>
      <geom name="wall_west" pos="-2 0 0.5" size="0.1 2 0.5" type="box" rgba="0.8 0.8 0.8 1"/>
    </body>
    
    <!-- 障碍物 -->
    <body name="obstacle1" pos="0.5 0.5 0.2">
      <geom name="obs1" type="box" size="0.2 0.2 0.2" rgba="0.7 0.3 0.3 1"/>
    </body>
    
    <body name="obstacle2" pos="-0.5 -0.5 0.2">
      <geom name="obs2" type="box" size="0.2 0.2 0.2" rgba="0.3 0.7 0.3 1"/>
    </body>
    
    <body name="obstacle3" pos="0.8 -0.8 0.2">
      <geom name="obs3" type="box" size="0.15 0.15 0.2" rgba="0.3 0.3 0.7 1"/>
    </body>
    
    <!-- 机器人1 (红色) -->
    <body name="robot1_base" pos="-1.5 -1.5 0.1">
      <geom name="robot1_base_geom" type="cylinder" size="0.1 0.05" rgba="1 0 0 1"/>
      <joint name="robot1_joint" type="free"/>
      <body name="robot1_body" pos="0 0 0.1">
        <geom name="robot1_body_geom" type="box" size="0.08 0.08 0.1" rgba="1 0 0 1"/>
        <site name="robot1_site" pos="0 0 0" size="0.05" rgba="1 0 0 0.5"/>
      </body>
    </body>
    
    <!-- 机器人2 (绿色) -->
    <body name="robot2_base" pos="1.5 1.5 0.1">
      <geom name="robot2_base_geom" type="cylinder" size="0.1 0.05" rgba="0 1 0 1"/>
      <joint name="robot2_joint" type="free"/>
      <body name="robot2_body" pos="0 0 0.1">
        <geom name="robot2_body_geom" type="box" size="0.08 0.08 0.1" rgba="0 1 0 1"/>
        <site name="robot2_site" pos="0 0 0" size="0.05" rgba="0 1 0 0.5"/>
      </body>
    </body>
    
    <!-- 目标地点 -->
    <body name="target1" pos="1.5 -1.5 0.1">
      <geom name="target1_geom" type="cylinder" size="0.1 0.05" conaffinity="0" contype="0" rgba="0 0 1 0.5"/>
      <site name="target1_site" pos="0 0 0" size="0.05" rgba="0 0 1 1"/>
    </body>
    
    <body name="target2" pos="-1.5 1.5 0.1">
      <geom name="target2_geom" type="cylinder" size="0.1 0.05" conaffinity="0" contype="0" rgba="1 1 0 0.5"/>
      <site name="target2_site" pos="0 0 0" size="0.05" rgba="1 1 0 1"/>
    </body>
  </worldbody>
  
  <actuator>
    <motor name="robot1_motor_x" joint="robot1_joint" gear="100"/>
    <motor name="robot1_motor_y" joint="robot1_joint" gear="100"/>
    <motor name="robot2_motor_x" joint="robot2_joint" gear="100"/>
    <motor name="robot2_motor_y" joint="robot2_joint" gear="100"/>
  </actuator>
</mujoco>
"""
        
        with open(self.xml_path, "w") as f:
            f.write(xml_content)
        print(f"✅ 导航场景XML创建成功: {self.xml_path}")
    
    def load_model(self):
        """加载MuJoCo模型"""
        try:
            self.model = mujoco.MjModel.from_xml_path(self.xml_path)
            self.data = mujoco.MjData(self.model)
            print("✅ 导航模型加载成功!")
            print(f"   关节数量: {self.model.njnt}")
            print(f"   物体数量: {self.model.nbody}")
            print(f"   执行器数量: {self.model.nu}")
            return True
        except Exception as e:
            print(f"❌ 模型加载失败: {e}")
            return False
    
    def get_robot_positions(self) -> Dict[str, np.ndarray]:
        """获取机器人位置"""
        positions = {}
        try:
            robot1_pos = self.data.xpos[self.model.body_name2id("robot1_base")]
            robot2_pos = self.data.xpos[self.model.body_name2id("robot2_base")]
            positions = {
                "robot1": robot1_pos.copy(),
                "robot2": robot2_pos.copy()
            }
        except Exception as e:
            print(f"❌ 获取机器人位置失败: {e}")
        return positions
    
    def get_target_positions(self) -> Dict[str, np.ndarray]:
        """获取目标位置"""
        positions = {}
        try:
            target1_pos = self.data.xpos[self.model.body_name2id("target1")]
            target2_pos = self.data.xpos[self.model.body_name2id("target2")]
            positions = {
                "target1": target1_pos.copy(),
                "target2": target2_pos.copy()
            }
        except Exception as e:
            print(f"❌ 获取目标位置失败: {e}")
        return positions
    
    def simple_navigation_control(self, step: int) -> Tuple[float, float, float, float]:
        """简单的导航控制策略"""
        # 获取当前位置
        robot_positions = self.get_robot_positions()
        target_positions = self.get_target_positions()
        
        if not robot_positions or not target_positions:
            return 0, 0, 0, 0
        
        # 机器人1: 从(-1.5, -1.5)到(1.5, -1.5)
        robot1_pos = robot_positions["robot1"]
        target1_pos = target_positions["target1"]
        
        # 机器人2: 从(1.5, 1.5)到(-1.5, 1.5)
        robot2_pos = robot_positions["robot2"]
        target2_pos = target_positions["target2"]
        
        # 计算控制信号
        kp = 2.0  # 比例增益
        
        # 机器人1控制
        error1 = target1_pos - robot1_pos
        robot1_ctrl_x = kp * error1[0]
        robot1_ctrl_y = kp * error1[1]
        
        # 机器人2控制
        error2 = target2_pos - robot2_pos
        robot2_ctrl_x = kp * error2[0]
        robot2_ctrl_y = kp * error2[1]
        
        # 限制控制信号
        max_ctrl = 5.0
        robot1_ctrl_x = np.clip(robot1_ctrl_x, -max_ctrl, max_ctrl)
        robot1_ctrl_y = np.clip(robot1_ctrl_y, -max_ctrl, max_ctrl)
        robot2_ctrl_x = np.clip(robot2_ctrl_x, -max_ctrl, max_ctrl)
        robot2_ctrl_y = np.clip(robot2_ctrl_y, -max_ctrl, max_ctrl)
        
        return robot1_ctrl_x, robot1_ctrl_y, robot2_ctrl_x, robot2_ctrl_y
    
    def run_navigation_simulation(self, max_steps: int = 1000):
        """运行导航仿真"""
        if not self.model or not self.data:
            print("❌ 模型未加载")
            return
        
        print("🚀 开始导航仿真...")
        print("   机器人1: 从(-1.5, -1.5)到(1.5, -1.5)")
        print("   机器人2: 从(1.5, 1.5)到(-1.5, 1.5)")
        
        positions_history = []
        
        for step in range(max_steps):
            # 应用控制策略
            ctrl1_x, ctrl1_y, ctrl2_x, ctrl2_y = self.simple_navigation_control(step)
            
            # 设置控制信号
            self.data.ctrl[0] = ctrl1_x  # robot1 x
            self.data.ctrl[1] = ctrl1_y  # robot1 y
            self.data.ctrl[2] = ctrl2_x  # robot2 x
            self.data.ctrl[3] = ctrl2_y  # robot2 y
            
            # 步进仿真
            mujoco.mj_step(self.model, self.data)
            
            # 记录位置
            if step % 50 == 0:
                robot_positions = self.get_robot_positions()
                target_positions = self.get_target_positions()
                
                if robot_positions and target_positions:
                    positions_history.append({
                        'step': step,
                        'robot1': robot_positions["robot1"].copy(),
                        'robot2': robot_positions["robot2"].copy(),
                        'target1': target_positions["target1"].copy(),
                        'target2': target_positions["target2"].copy()
                    })
                    
                    # 计算距离
                    dist1 = np.linalg.norm(robot_positions["robot1"] - target_positions["target1"])
                    dist2 = np.linalg.norm(robot_positions["robot2"] - target_positions["target2"])
                    
                    print(f"  步骤 {step}: 机器人1距离目标 {dist1:.3f}, 机器人2距离目标 {dist2:.3f}")
                    
                    # 检查是否到达目标
                    if dist1 < 0.2 and dist2 < 0.2:
                        print(f"🎉 任务完成! 两个机器人都到达了目标位置")
                        break
        
        # 可视化结果
        if positions_history:
            self.visualize_navigation(positions_history)
        
        print("✅ 导航仿真完成!")
    
    def visualize_navigation(self, positions_history: List[Dict]):
        """可视化导航过程"""
        if not positions_history:
            print("❌ 没有轨迹数据可视化")
            return
        
        # 提取数据
        steps = [p['step'] for p in positions_history]
        robot1_x = [p['robot1'][0] for p in positions_history]
        robot1_y = [p['robot1'][1] for p in positions_history]
        robot2_x = [p['robot2'][0] for p in positions_history]
        robot2_y = [p['robot2'][1] for p in positions_history]
        target1_x = [p['target1'][0] for p in positions_history]
        target1_y = [p['target1'][1] for p in positions_history]
        target2_x = [p['target2'][0] for p in positions_history]
        target2_y = [p['target2'][1] for p in positions_history]
        
        # 创建可视化
        plt.figure(figsize=(15, 10))
        
        # 子图1: 机器人轨迹
        plt.subplot(2, 2, 1)
        plt.plot(robot1_x, robot1_y, 'r-', linewidth=2, label='Robot1轨迹')
        plt.plot(robot2_x, robot2_y, 'g-', linewidth=2, label='Robot2轨迹')
        plt.scatter(robot1_x[0], robot1_y[0], color='red', s=100, marker='o', label='Robot1起点')
        plt.scatter(robot2_x[0], robot2_y[0], color='green', s=100, marker='o', label='Robot2起点')
        plt.scatter(target1_x[0], target1_y[0], color='blue', s=100, marker='x', label='Target1')
        plt.scatter(target2_x[0], target2_y[0], color='yellow', s=100, marker='x', label='Target2')
        
        # 添加障碍物
        obstacles = [(0.5, 0.5), (-0.5, -0.5), (0.8, -0.8)]
        for obs in obstacles:
            plt.scatter(obs[0], obs[1], color='black', s=200, marker='s', label='障碍物' if obs == obstacles[0] else "")
        
        plt.xlabel('X Position')
        plt.ylabel('Y Position')
        plt.title('机器人导航轨迹')
        plt.legend()
        plt.grid(True)
        plt.axis('equal')
        
        # 子图2: 机器人1轨迹
        plt.subplot(2, 2, 2)
        plt.plot(steps, robot1_x, 'r-', linewidth=2, label='Robot1 X')
        plt.plot(steps, robot1_y, 'r--', linewidth=2, label='Robot1 Y')
        plt.xlabel('Simulation Steps')
        plt.ylabel('Position')
        plt.title('Robot1 Position Over Time')
        plt.legend()
        plt.grid(True)
        
        # 子图3: 机器人2轨迹
        plt.subplot(2, 2, 3)
        plt.plot(steps, robot2_x, 'g-', linewidth=2, label='Robot2 X')
        plt.plot(steps, robot2_y, 'g--', linewidth=2, label='Robot2 Y')
        plt.xlabel('Simulation Steps')
        plt.ylabel('Position')
        plt.title('Robot2 Position Over Time')
        plt.legend()
        plt.grid(True)
        
        # 子图4: 距离目标的变化
        plt.subplot(2, 2, 4)
        distances1 = [np.linalg.norm(np.array([robot1_x[i], robot1_y[i]]) - np.array([target1_x[i], target1_y[i]])) 
                    for i in range(len(robot1_x))]
        distances2 = [np.linalg.norm(np.array([robot2_x[i], robot2_y[i]]) - np.array([target2_x[i], target2_y[i]])) 
                    for i in range(len(robot2_x))]
        
        plt.plot(steps, distances1, 'r-', linewidth=2, label='Robot1到目标距离')
        plt.plot(steps, distances2, 'g-', linewidth=2, label='Robot2到目标距离')
        plt.xlabel('Simulation Steps')
        plt.ylabel('Distance to Target')
        plt.title('Distance to Target Over Time')
        plt.legend()
        plt.grid(True)
        
        plt.tight_layout()
        plt.show()
        
        print("📈 导航可视化完成!")

def main():
    """主函数"""
    print("🤖 自定义导航任务演示")
    print("=" * 50)
    
    # 创建导航任务
    task = CustomNavigationTask()
    
    # 创建场景
    task.create_navigation_scene()
    
    # 加载模型
    if not task.load_model():
        return
    
    # 运行仿真
    task.run_navigation_simulation(max_steps=1000)
    
    print("\n✅ 自定义导航任务演示完成!")

if __name__ == "__main__":
    main()
