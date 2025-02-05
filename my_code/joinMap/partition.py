import os
import cv2

def split_stereo_images(input_folder, output_left_folder, output_right_folder):
    # 确保输出文件夹存在，如果不存在则创建
    if not os.path.exists(output_left_folder):
        os.makedirs(output_left_folder)
    if not os.path.exists(output_right_folder):
        os.makedirs(output_right_folder)

    # 遍历输入文件夹中的所有文件
    for filename in os.listdir(input_folder):
        # 构建完整路径
        file_path = os.path.join(input_folder, filename)
        
        # 检查是否为文件且为图像文件（可以根据需要调整检查条件）
        if os.path.isfile(file_path) and filename.lower().endswith(('.png', '.jpg', '.jpeg', '.bmp', '.tiff')):
            # 读取图像
            image = cv2.imread(file_path)
            
            # 获取图像尺寸
            height, width = image.shape[:2]
            
            # 确认图像宽度为1280，高度为480
            if width != 2560 or height != 720:
                print(f"警告：图像 {filename} 分辨率不是1280x480，跳过此文件")
                continue
            
            # 分割图像
            left_image = image[:, :width//2]  # 左半部分
            right_image = image[:, width//2:]  # 右半部分
            
            # 构建输出路径
            left_output_path = os.path.join(output_left_folder, filename)
            right_output_path = os.path.join(output_right_folder, filename)
            
            # 保存分割后的图像
            cv2.imwrite(left_output_path, left_image)
            cv2.imwrite(right_output_path, right_image)
            
            print(f"成功处理图像: {filename}")

# 设置您的文件夹路径
input_folder = 'path_to_input_folder'  # 输入图像的文件夹路径
output_left_folder = 'path_to_output_left_folder'  # 输出左图像的文件夹路径
output_right_folder = 'path_to_output_right_folder'  # 输出右图像的文件夹路径

# 执行函数
split_stereo_images("C:/Users/JG/Pictures/room/room", 'C:/Users/JG/Pictures/room/left', 'C:/Users/JG/Pictures/room/right')