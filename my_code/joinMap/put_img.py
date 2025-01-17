import os
import time
import shutil

def move_images_by_timestamp(src_folder, dst_folder, interval=1):
    """
    按时间戳排序将图像从 src_folder 移动到 dst_folder，目标文件夹始终只有一张图像。
    
    Args:
        src_folder (str): 源文件夹路径，存放带有时间戳命名的图像文件。
        dst_folder (str): 目标文件夹路径，存放当前图像。
        interval (int): 每张图像移动的间隔时间（秒）。
    """
    # 确保目标文件夹存在
    os.makedirs(dst_folder, exist_ok=True)

    # 获取源文件夹中的所有图像文件
    images = [f for f in os.listdir(src_folder) if f.lower().endswith(('.png', '.jpg', '.jpeg', '.bmp', '.tif', '.tiff'))]

    # 根据文件名（时间戳）排序
    images.sort(key=lambda x: x.split('.')[0])  # 假设时间戳是文件名的前部分

    for image in images:
        src_path = os.path.join(src_folder, image)
        dst_path = os.path.join(dst_folder, image)

        # 移动图像到目标文件夹
        shutil.move(src_path, dst_path)
        print(f"Moved {image} to {dst_folder}")

        # 确保目标文件夹中始终只有一张图像
        for file in os.listdir(dst_folder):
            if file != image:
                os.remove(os.path.join(dst_folder, file))

        # 等待指定间隔时间
        time.sleep(interval)

if __name__ == "__main__":
    # 设置源文件夹和目标文件夹路径
    source_folder = "/my_workspace/catkin_ws/src/AirSlam/dataroot/cam1/data"
    destination_folder = "/my_workspace/catkin_ws/src/AirSlam/dataroot/test"

    # 调用函数
    move_images_by_timestamp(source_folder, destination_folder, interval=0.25)
