import matplotlib.pyplot as plt
import numpy as np

# 解析文件并提取位姿信息，只保留成功的轨迹
def parse_trajectory(file_path):
    trajectory = []

    with open(file_path, 'r') as f:
        for line in f:
            # 跳过无效行（例如空行或非数据行）
            if not line.strip():
                continue
            
            # 解析每一行数据
            parts = line.split()
            status = parts[0]  # "success" 或 "fail"
            
            # 只处理成功的轨迹数据
            if status != "success":
                continue
            
            timestamp = parts[1]  # 时间戳
            qw,qx, qy, qz = map(float, parts[5:9])  # 四元数旋转部分
            tx, ty, tz = map(float, parts[2:5])  # 位移向量部分
            
            # 添加位移向量 tx, ty, tz 到轨迹中
            trajectory.append([tx, ty, tz])

    return np.array(trajectory)

# 绘制轨迹
def plot_trajectory(trajectory):
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    
    # 画出轨迹
    ax.plot(trajectory[:, 0], trajectory[:, 1], trajectory[:, 2], label="Trajectory", color="b")

    # 设置轴标签
    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_zlabel("Z")

    # 设置标题
    ax.set_title("3D Trajectory (success only)")

    # 显示图形
    plt.legend()
    plt.show()

# 主程序
if __name__ == "__main__":
    # 替换为你的文件路径
    file_path = "D:/docker_test/catkin_ws/src/AirSlam/debug/relocalization.txt"
    
    # 解析轨迹数据
    trajectory = parse_trajectory(file_path)
    
    # 绘制轨迹
    plot_trajectory(trajectory)
