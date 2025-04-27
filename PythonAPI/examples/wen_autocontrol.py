"""
This script is used for autocontrol the vehicle.
You have two options to switch between recording/replay mode.
If you want to record the path you are driving, set --agents to True.
If you want to replay the path, set --agents to False and --replay to True.
"""

# ==============================================================================
# -- find carla module ---------------------------------------------------------
# ==============================================================================


import glob
import os
import sys
import json

try:
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

# ==============================================================================
# -- imports -------------------------------------------------------------------
# ==============================================================================

import carla
import csv
import time
import argparse

sys.path.append('../carla')

from agents.navigation.behavior_agent import BehaviorAgent

def record_path(vehicle, filename=""):
    """ 記錄車輛的行駛位置點到 CSV """
    if not filename:
        print("error: 請輸入儲存路徑 (--save-path)")
        return
    with open(filename, mode="w", newline="") as file:
        writer = csv.writer(file)
        writer.writerow(["x", "y", "z", "pitch", "yaw", "roll"])  # 寫入標題

        while True:
            transform = vehicle.get_transform()
            location = transform.location
            rotation = transform.rotation
            yaw = transform.rotation.yaw  # 取得車輛的偏航角
            
            writer.writerow([location.x, location.y, location.z, rotation.pitch, rotation.yaw, rotation.roll])  # 記錄座標與偏航角
            print(f"Recorded: x={location.x}, y={location.y}, z={location.z}, pitch={rotation.pitch}, yaw={rotation.yaw}, roll={rotation.roll}")  # 顯示目前位置

            time.sleep(0.5)  # 每 0.5 秒記錄一次位置
            
            # # 檢查車輛是否到達終點
            # if agent.done():
            #     print("目的地已到達！")
            #     break

def load_path(filename=None):
    """ 讀取記錄的路徑點 """

    # 檢查檔案是否存在
    if not os.path.exists(filename):
        print(f"警告: 檔案 '{filename}' 不存在！請先記錄路徑後再執行此函式。")
        return []
    
    waypoints = []
    with open(filename, mode="r") as file:
        reader = csv.reader(file)
        next(reader)  # 跳過標題行
        for row in reader:
            x, y, z, pitch, yaw, roll = map(float, row)  # 解析 CSV 資料
            # x, y, z, yaw = map(float, row)  # 解析 CSV 資料
            transform = carla.Transform(carla.Location(x, y, z-0.01), carla.Rotation(pitch, yaw, roll))
            waypoints.append(transform)  # 存入 Transform 物件

    return waypoints

def replay_path(vehicle, waypoints):
    """ 讓車輛沿著記錄的路徑行駛 (包含方向角 yaw) """
    vehicle.apply_control(carla.VehicleControl(throttle=0.0, brake=0.0, hand_brake=True))
    # print("---- pressed R ---- \nstart recording ) ")
    # keyboard.write('R',delay=0)
    for waypoint in waypoints:
        vehicle.set_transform(waypoint)  # 直接使用 Transform 
        print(f"location: {waypoint}")
        time.sleep(1.5)  # 每 1.5 秒移動一次
    print("Path replay complete!")
    # print("---- pressed R ----")
    # keyboard.write('R',delay=0)
    vehicle.apply_control(carla.VehicleControl(throttle=0.0, brake=0.0, hand_brake=False))

def main(args):
    client = carla.Client('localhost', 2000)
    client.set_timeout(5.0)
    world = client.get_world()
    print(f"Args: {args}")

    # **找到玩家控制的車輛**
    ego_vehicle = None
    all_vehicles = world.get_actors().filter('vehicle.*')

    for vehicle in all_vehicles:
        print(f"vehicle id:{vehicle.id}, role_name:{vehicle.attributes.get('role_name')}")
        if vehicle.attributes.get('role_name') == 'hero':  # 預設玩家車輛名稱
            ego_vehicle = vehicle
            break

    if args.agents == True:
        print("Mode: agents mode")

        ###### 這段目前有問題，無法成功運作 ######
        # # 設定行為代理人
        # global agent  # 讓 agent 可被 record_path() 使用
        # agent = BehaviorAgent(ego_vehicle, behavior="normal")  # "normal"、"aggressive"、"cautious"
        
        # # 設定目標位置
        # # spawn_points = self.map.get_spawn_points()
        # # destination = spawn_points[20].location  # 目標位置
        # x = 284
        # y = -145
        # z = 1
        # destination = carla.Transform(carla.Location(x=x,y=y, z=z), carla.Rotation(yaw=180)).location # 目標位置
        # agent.set_destination(destination)
        ######################################

        # 啟動記錄線程
        print("start recording path")
        # print("Agent 的路徑長度:", len(agent._local_planner.waypoints_queue))
        recodring_path = args.recordingPath
        record_path(ego_vehicle, recodring_path)

    elif args.replay == True:
        print("Mode: replay mode")
        # 設定自動駕駛模組
        # vehicle.set_autopilot(True)  # 讓 CARLA 內建的 AI 控制車輛

        # 讀取記錄的路徑
        # map_path = "Recording/recorded_path_all2.csv"
        map_path = args.replayPath
        print(f"using path: {map_path}")
        waypoints = load_path(map_path)

        # 讓車輛重現行駛路徑
        replay_path(ego_vehicle, waypoints)

        print(f"Path:{map_path} completed!")

if __name__ == '__main__':
    argparser = argparse.ArgumentParser(
        description='CARLA Manual Control Client')
    argparser.add_argument(
        '--agents',
        action='store_true',
        default=False,
        help='recording path')
    argparser.add_argument(
        '-p', '--replay',
        action='store_true',
        default=True,
        help='replay and move vehicle')
    argparser.add_argument(
    '--recordingPath',
    type=str,
    default='Recording/recorded_path_all5.csv',  # 可以自己設一個預設儲存路徑
    help='path to save recording path')
    argparser.add_argument(
    '--replayPath',
    type=str,
    default='Recording/recorded_path_all2.csv',  # 可以自己設一個預設儲存路徑
    help='path to load replay path')

    args = argparser.parse_args()
    main(args)