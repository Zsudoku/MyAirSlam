import pandas as pd

# 读取原始的txt文件
input_file = "D:/docker_test/catkin_ws/src/AirSlam/debug/relocalization.txt"  # 请替换为你的文件路径
output_file = "D:/docker_test/catkin_ws/src/AirSlam/debug/new_relocalization.txt"  # 新文件路径

# 读取数据
df = pd.read_csv(input_file, header=None, names=["result","image_path", "pose1", "pose2", "pose3", "pose4", "pose5", "pose6", "pose7"])

# 去除重复的行
df_unique = df.drop_duplicates(subset=["result","image_path", "pose1", "pose2", "pose3", "pose4", "pose5", "pose6", "pose7"])

# 保存为新的txt文件
df_unique.to_csv(output_file, index=False, header=False)

print(f"去重后的数据已保存到 {output_file}")
