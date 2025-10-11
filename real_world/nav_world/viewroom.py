import mujoco
import mujoco.viewer
import numpy as np

# 加载模型
model = mujoco.MjModel.from_xml_path("room.xml")
data = mujoco.MjData(model)

# 启动可交互的 MuJoCo 可视化窗口
mujoco.viewer.launch(model, data)
