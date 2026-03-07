import pinocchio as pin
from pinocchio.utils import zero

# 加载模型
urdf_path = "openarm_bimanual.urdf"
model = pin.buildModelFromUrdf(urdf_path)
data = model.createData()

# 零位关节
q = zero(model.nq)
g = pin.computeGeneralizedGravity(model, data, q)

print("=== 零位重力补偿力矩分析 ===")
for idx, (name, torque) in enumerate(zip(model.names[1:], g)):  # names[0]是universe
    print(f"关节 {idx+1} ({name}): 力矩 = {torque:.4f} Nm")

print("\n说明：")
print("1. 如果某些关节的重力力矩很大，通常是因为该关节需要支撑其后所有连杆和末端执行器的重量。")
print("2. 机械臂在零位时，部分连杆可能处于悬臂状态，导致重力臂最大，力矩也最大。")
print("3. 检查URDF中各连杆的质量、质心位置、重力方向设置是否合理。")
print("4. 若重力方向设置为(0, 0, -9.81)，则重力沿Z轴向下。")
print("5. 若发现异常大，建议检查URDF的惯性参数和质量单位。")

# 可视化（可选，需要安装meshcat）
try:
    from pinocchio.visualize import MeshcatVisualizer
    viz = MeshcatVisualizer(model)
    viz.initViewer()
    viz.display(q)
    print("已在Meshcat中显示零位姿态。")
except ImportError:
    print("未安装meshcat，可跳过可视化。")