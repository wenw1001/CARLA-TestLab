import os
import glob
# def increment_path(path, exist_ok=False):
#     if os.path.exists(path) and not exist_ok:
#         raise ValueError(f"Path '{path}' already exists.")
    
#     directory, filename = os.path.split(path)
#     print(f"directory:{directory}, filename:{filename}")

#     prefix, suffix = os.path.splitext(filename)
#     print(f"prefix: {prefix}, suffix:{suffix}")

#     i = 1
#     while os.path.exists(os.path.join(directory, f"{prefix}_{i}{suffix}")):
#         i += 1

#     new_path  = os.path.join(directory, f"{prefix}_{i}{suffix}")
#     print(f"new_path: {new_path}")

#     return new_path

def increment_path(root_path, prefix="recording_"):
    if not os.path.exists(root_path):
            os.makedirs(root_path)  # 如果根目錄不存在，則建立

    # 搜尋符合 pattern 的資料夾，例如 "recording_*"
    pattern = os.path.join(root_path, f"{prefix}*")
    existing_dirs = glob.glob(pattern)

    # 找出最大索引值
    max_index = 0
    for directory in existing_dirs:
        dirname = os.path.basename(directory)  # 取得目錄名稱
        try:
            index = int(dirname[len(prefix):])  # 解析數字部分
            max_index = max(max_index, index)
        except ValueError:
            continue  # 忽略無法解析的資料夾名稱

    # 產生新的資料夾名稱
    new_dirname = f"{prefix}{max_index + 1}"
    new_dirpath = os.path.join(root_path, new_dirname)

    # 建立新資料夾
    os.makedirs(new_dirpath)

    return new_dirpath
