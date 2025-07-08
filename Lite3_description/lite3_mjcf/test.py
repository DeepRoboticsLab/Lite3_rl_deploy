# 测试生成的xml文件是否能被mujoco加载


import mujoco
import mujoco.viewer
import os

# 路径设置
xml_path = "/home/percy/tmp/Lite3_rl_deploy/third_party/URDF_model/Lite3/Lite3_mjcf/mjcf/Lite3.xml"  # 或直接用完整路径："/home/mayuxuan/..."

# 加载模型
model = mujoco.MjModel.from_xml_path(xml_path)
data = mujoco.MjData(model)

# 启动可视化窗口
with mujoco.viewer.launch_passive(model, data) as viewer:
    print("Press ESC to exit viewer.")
    while viewer.is_running():
        step_start = data.time
        mujoco.mj_step(model, data)
        viewer.sync()
        while data.time - step_start < model.opt.timestep:
            pass  # busy wait for real-time
