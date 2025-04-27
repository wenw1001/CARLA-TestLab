import cv2
import os
import imageio

def png_to_mp4(image_folder, output_mp4, fps=30):
    # 確保輸出的影片副檔名是 .mp4
    if not output_mp4.endswith(".mp4"):
        output_mp4 += ".mp4"
    
    # 取得所有 PNG 檔案並排序
    images = [img for img in os.listdir(image_folder) if img.endswith(".png")]
    images.sort()  # 確保按照名稱順序排列
    
    if not images:
        print("No PNG images found in the folder.")
        return
    
    # 讀取第一張圖片來獲取影片尺寸
    first_image = cv2.imread(os.path.join(image_folder, images[0]))
    height, width, _ = first_image.shape
    
    # 使用 imageio 來輸出 MP4 影片
    writer = imageio.get_writer(output_mp4, format='FFMPEG', mode='I', fps=fps, codec='libx264')
    
    for i, image_name in enumerate(images):
        print(f"process: {i+1}/{len(images)}")
        img_path = os.path.join(image_folder, image_name)
        img = cv2.imread(img_path)
        if img is None:
            print(f"Warning: Unable to read {image_name}")
            continue
        
        # 轉換為 RGB（OpenCV 預設是 BGR）
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        writer.append_data(img)
    
    writer.close()
    print(f"MP4 file saved as {output_mp4}")

# 使用範例
src_path = "Recording/Recording_4"
save_path = "Recording/Recording_4/output.mp4"
png_to_mp4(src_path, save_path, fps=5)
