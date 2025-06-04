"""A script to get nearby vehicle information in the simulation"""

import glob
import os
import sys
import time

try:
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

import carla

from carla import VehicleLightState as vls

import argparse
import logging
from numpy import random
import math


def get_actor_blueprints(world, filter, generation):
    bps = world.get_blueprint_library().filter(filter)

    if generation.lower() == "all":
        return bps

    # If the filter returns only one bp, we assume that this one needed
    # and therefore, we ignore the generation
    if len(bps) == 1:
        return bps

    try:
        int_generation = int(generation)
        # Check if generation is in available generations
        if int_generation in [1, 2, 3]:
            bps = [x for x in bps if int(x.get_attribute('generation')) == int_generation]
            return bps
        else:
            print("   Warning! Actor Generation is not valid. No actor will be spawned.")
            return []
    except:
        print("   Warning! Actor Generation is not valid. No actor will be spawned.")
        return []

def main():
    argparser = argparse.ArgumentParser(
        description=__doc__)
    argparser.add_argument(
        '--host',
        metavar='H',
        default='127.0.0.1',
        help='IP of the host server (default: 127.0.0.1)')
    argparser.add_argument(
        '-p', '--port',
        metavar='P',
        default=2000,
        type=int,
        help='TCP port to listen to (default: 2000)')
    argparser.add_argument(
        '-n', '--number-of-vehicles',
        metavar='N',
        default=30,
        type=int,
        help='Number of vehicles (default: 30)')
    argparser.add_argument(
        '-w', '--number-of-walkers',
        metavar='W',
        default=10,
        type=int,
        help='Number of walkers (default: 10)')
    argparser.add_argument(
        '--safe',
        action='store_true',
        help='Avoid spawning vehicles prone to accidents')
    argparser.add_argument(
        '--filterv',
        metavar='PATTERN',
        default='vehicle.*',
        help='Filter vehicle model (default: "vehicle.*")')
    argparser.add_argument(
        '--generationv',
        metavar='G',
        default='All',
        help='restrict to certain vehicle generation (values: "1","2","All" - default: "All")')
    argparser.add_argument(
        '--filterw',
        metavar='PATTERN',
        default='walker.pedestrian.*',
        help='Filter pedestrian type (default: "walker.pedestrian.*")')
    argparser.add_argument(
        '--generationw',
        metavar='G',
        default='2',
        help='restrict to certain pedestrian generation (values: "1","2","All" - default: "2")')
    argparser.add_argument(
        '--tm-port',
        metavar='P',
        default=8000,
        type=int,
        help='Port to communicate with TM (default: 8000)')
    argparser.add_argument(
        '--asynch',
        action='store_true',
        help='Activate asynchronous mode execution')
    argparser.add_argument(
        '--hybrid',
        action='store_true',
        help='Activate hybrid mode for Traffic Manager')
    argparser.add_argument(
        '-s', '--seed',
        metavar='S',
        type=int,
        help='Set random device seed and deterministic mode for Traffic Manager')
    argparser.add_argument(
        '--seedw',
        metavar='S',
        default=0,
        type=int,
        help='Set the seed for pedestrians module')
    argparser.add_argument(
        '--car-lights-on',
        action='store_true',
        default=False,
        help='Enable automatic car light management')
    argparser.add_argument(
        '--hero',
        action='store_true',
        default=False,
        help='Set one of the vehicles as hero')
    argparser.add_argument(
        '--respawn',
        action='store_true',
        default=False,
        help='Automatically respawn dormant vehicles (only in large maps)')
    argparser.add_argument(
        '--no-rendering',
        action='store_true',
        default=False,
        help='Activate no rendering mode')
    
    args = argparser.parse_args()

    vehicles_list = []
    traffic_list = []

    client = carla.Client(args.host, args.port)
    client.set_timeout(10.0)
    synchronous_master = False
    random.seed(args.seed if args.seed is not None else int(time.time()))

    try:
        world = client.get_world()
        settings = world.get_settings()

        # actor_list = world.get_actors()
        
        # number_of_actors = len(actor_list)
        # print(f"There are {number_of_actors} actor spawn: ")

        # for n, actor in enumerate(actor_list): # calculate numbers of vehicle
        #     if "vehicle" in actor.type_id:
        #         vehicles_list.append(actor) # append vehicle actor to vehicle_list
        #         print(f"find a vehicle (id:{actor.id})")
        #     if "traffic" in actor.type_id:
        #         traffic_list.append(actor)

        # print(f"\nVehicle num: {len(vehicles_list)}, Traffic num: {len(traffic_list)}\n")

        # **找到玩家控制的車輛**
        ego_vehicle = None
        all_vehicles = world.get_actors().filter('vehicle.*')

        for vehicle in all_vehicles:
            print(f"vehicle id:{vehicle.id}, role_name:{vehicle.attributes.get('role_name')}")
            if vehicle.attributes.get('role_name') == 'hero':  # 預設玩家車輛名稱
                ego_vehicle = vehicle
                # break
            else:
                vehicles_list.append(vehicle)

        if ego_vehicle is None:
            print("找不到玩家車輛！")
        else:
            while True:
                ego_location = ego_vehicle.get_location()
                ego_forward_vector = ego_vehicle.get_transform().get_forward_vector()  # 自車前向量
                print(f"ego_forward_vector={ego_forward_vector}")
                nearby_vehicles = []

                for vehicle in vehicles_list:
                    distance = ego_location.distance(vehicle.get_location())
                    vehicle_location = vehicle.get_location()
                    if distance < 100.0:  # 設定 100 公尺為偵測範圍
                        # 計算相對位置向量
                        relative_vector = carla.Vector3D(
                            vehicle_location.x - ego_location.x,
                            vehicle_location.y - ego_location.y,
                            vehicle_location.z - ego_location.z
                        )
                        
                        # 計算內積
                        dot_product = (
                            relative_vector.x * ego_forward_vector.x +
                            relative_vector.y * ego_forward_vector.y +
                            relative_vector.z * ego_forward_vector.z
                        )

                        # 計算向量長度（歐幾里得範數）
                        magnitude_ego = math.sqrt(ego_forward_vector.x**2 + ego_forward_vector.y**2 + ego_forward_vector.z**2)
                        magnitude_relative = math.sqrt(relative_vector.x**2 + relative_vector.y**2 + relative_vector.z**2)

                        # 避免除以零
                        if magnitude_ego > 0 and magnitude_relative > 0:
                            # 計算夾角（弧度轉角度）
                            angle_rad = math.acos(dot_product / (magnitude_ego * magnitude_relative))
                            angle_deg = math.degrees(angle_rad)  # 轉換為度數
                        else:
                            angle_deg = 0.0  # 若向量長度為零，則角度設為 0


                        print(f"id:{vehicle.id}, dot={round(dot_product,6)}, degree={round(angle_deg,4)}")
                        if angle_deg < 40:  # 只記錄前方的車輛
                            nearby_vehicles.append((vehicle, distance))

                if nearby_vehicles:
                    print(f"偵測到前方 {len(nearby_vehicles)} 輛停車車輛在 100 公尺內：")
                    for vehicle, distance in nearby_vehicles:
                        loc = vehicle.get_location()
                        print(f"  - 車輛 {vehicle.id} 位置: ({loc.x:.2f}, {loc.y:.2f}, {loc.z:.2f}), 距離: {distance:.4f} m")
                
                time.sleep(1)  # 每秒檢查一次

        ########################################################################################

        # vehicle_actor = vehicles_list[0]
        # print(f"-- vehicle id:{vehicle_actor.id} current information --")
        # print(f"acceleration: {vehicle_actor.get_acceleration()}")
        # print(f"angular_velocity: {vehicle_actor.get_angular_velocity}")
        # print(f"location: {vehicle_actor.get_location()}")
        # print(f"transform: {vehicle_actor.get_transform()}")
        # print(f"velocity: {vehicle_actor.get_velocity()}")
        # print(f"world: {vehicle_actor.get_world()}")
        # print(f"GNSS: {world.gnss_sensor.lat, world.gnss_sensor.lon}")
        # world.gnss_sensor.lat, world.gnss_sensor.lon
        
    
        ########################################################################################
        print('\npress Ctrl+C to exit.')

        while True:
            if not args.asynch and synchronous_master:
                world.tick()
            else:
                world.wait_for_tick()

    finally:

        if not args.asynch and synchronous_master:
            settings = world.get_settings()
            settings.synchronous_mode = False
            settings.no_rendering_mode = False
            settings.fixed_delta_seconds = None
            world.apply_settings(settings)
        

        time.sleep(0.5)


if __name__ == '__main__':

    try:
        main()
    except KeyboardInterrupt:
        pass
    finally:
        print('\ndone.')