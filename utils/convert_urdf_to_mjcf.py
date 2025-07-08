from urdfpy import URDF
from mujoco import mjcf
import os


urdf_path = "../third_party/URDF_model/lite3_urdf/lite3_pybullet/Lite3/urdf/Lite3.urdf"

# 加载 URDF 模型
robot = URDF.load(urdf_path)

# 转换为 MJCF 模型
mjcf_model = mjcf.from_urdf_model(robot)

# 保存为 MJCF XML 文件
output_path = "../third_party/URDF_model/lite3_mjcf/lite3_mjcf.xml"
mjcf_model.save(output_path)
print(f"Saved MJCF to {output_path}")
