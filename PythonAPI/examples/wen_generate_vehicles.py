"""A script to generate traffic(parking vehicles) in the simulation"""

import glob
import os
import sys
import time
import weakref

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
import random
import csv

def random_spawnpoints(num_sets, x_range=(80, 280)):
    x = random.uniform(x_range[0], x_range[1], num_sets)  # 在範圍內隨機取 x
    y = 0.01867 * x - 135.711  # 計算對應的 y 值
    # spawn_points = [carla.Transform(carla.Location(x, y, 1), carla.Rotation(yaw=0)) for x, y in zip(x, y)]
    # return spawn_points
    return list(zip(x, y))

def load_parking_points(filename="Recording/parking_points.csv"):
    """ 讀取記錄的路徑點 """

    # 檢查檔案是否存在
    if not os.path.exists(filename):
        print(f"警告: 檔案 '{filename}' 不存在！請先記錄路徑後再執行此函式。")
        return []
    
    parking_points = []
    with open(filename, mode="r") as file:
        reader = csv.reader(file)
        next(reader)  # 跳過標題行
        for row in reader:
            x, y, z, yaw = map(float, row)  # 解析 CSV 資料
            transform = carla.Transform(carla.Location(x, y, z+0.5), carla.Rotation(yaw=yaw))
            parking_points.append(transform)  # 存入 Transform 物件

    return parking_points


spawn_location = [[95, -134],[115, -133.6], [135,-133.2], [155, -132.8], [175, -132.4], [195,-132], [215,-131.6], [235,-131.2], [255, -131.2]
                  ]
# spawn_points = random_spawnpoints(15)

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

# ==============================================================================
# -- Vehicle ----------------------------------------------------------------
# ==============================================================================

class Vehicle(object):
    def __init__(self, carla_world, args):
        self.world = carla_world
        self.sync = args.sync
        self._actor_filter = args.filterv # default: "vehicle.*"
        self._actor_generation = args.generationv # default: "All"
        self.player = None
        ######### sensors #########
        self.gnss_sensor = None
        self.imu_sensor = None
        self.radar_sensor = None


    def generate_vehicle(self, location, rotation):
        blueprints = get_actor_blueprints(self.world, self._actor_filter, self._actor_generation)
        blueprint = random.choice(blueprints) # random vehicle style
        
        # Spawn the player.
        if self.player is not None:
            print("found an old play, destory it...")
            self.destroy()
        # way1: spawn at fixed point
        x = location[0]
        y = location[1]
        transform = carla.Transform(carla.Location(x=x, y=y), carla.Rotation(yaw=0))
        self.player = self.world.try_spawn_actor(blueprint, transform)
        while self.player is None:
            print(f" ({x}, {y})Spawn failed, adjusting position slightly...")
            x += 2  # 嘗試略微偏移位置
            transform = carla.Transform(carla.Location(x=x, y=y), carla.Rotation(yaw=0))
            self.player = self.world.try_spawn_actor(blueprint, transform)

        # # way2: spawn at random points
        # if not self.map.get_spawn_points():
        #     print('There are no spawn points available in your map/town.')
        #     print('Please add some Vehicle Spawn Point to your UE4 scene.')
        #     sys.exit(1)
        # spawn_points = self.map.get_spawn_points()
        # spawn_point = random.choice(spawn_points) if spawn_points else carla.Transform()
        # self.player = self.world.try_spawn_actor(blueprint, spawn_point)

        
        print(f"player spawned: {self.player}")
        print(f"transform: {transform}")
        print(f"spawn location: {self.player.get_transform()}")
        # Set up the sensors.
        # self.gnss_sensor = GnssSensor(self.player)
        # print(f"self.gnss_sensor = {self.gnss_sensor}")
        # self.imu_sensor = IMUSensor(self.player)

        if self.sync:
            self.world.tick()
        else:
            self.world.wait_for_tick()

    def toggle_radar(self):
        if self.radar_sensor is None:
            self.radar_sensor = RadarSensor(self.player)
        elif self.radar_sensor.sensor is not None:
            self.radar_sensor.sensor.destroy()
            self.radar_sensor = None

    def destroy(self):
        if self.radar_sensor is not None:
            self.toggle_radar()
        sensors = [
            self.gnss_sensor.sensor]#,
            #self.imu_sensor.sensor]
        for sensor in sensors:
            if sensor is not None:
                sensor.stop()
                sensor.destroy()
        if self.player is not None:
            self.player.destroy()


# ==============================================================================
# -- GnssSensor ----------------------------------------------------------------
# ==============================================================================


class GnssSensor(object):
    def __init__(self, parent_actor):
        self.sensor = None
        self._parent = parent_actor
        self.lat = 0.0
        self.lon = 0.0
        world = self._parent.get_world()
        bp = world.get_blueprint_library().find('sensor.other.gnss')
        self.sensor = world.spawn_actor(bp, carla.Transform(carla.Location(x=1.0, z=2.8)), attach_to=self._parent)
        # We need to pass the lambda a weak reference to self to avoid circular
        # reference.
        weak_self = weakref.ref(self)
        self.sensor.listen(lambda event: GnssSensor._on_gnss_event(weak_self, event))

    @staticmethod
    def _on_gnss_event(weak_self, event):
        self = weak_self()
        if not self:
            return
        self.lat = event.latitude
        self.lon = event.longitude

# ==============================================================================
# -- IMUSensor -----------------------------------------------------------------
# ==============================================================================


class IMUSensor(object):
    def __init__(self, parent_actor):
        self.sensor = None
        self._parent = parent_actor
        self.accelerometer = (0.0, 0.0, 0.0)
        self.gyroscope = (0.0, 0.0, 0.0)
        self.compass = 0.0
        world = self._parent.get_world()
        bp = world.get_blueprint_library().find('sensor.other.imu')
        self.sensor = world.spawn_actor(
            bp, carla.Transform(), attach_to=self._parent)
        # We need to pass the lambda a weak reference to self to avoid circular
        # reference.
        weak_self = weakref.ref(self)
        self.sensor.listen(
            lambda sensor_data: IMUSensor._IMU_callback(weak_self, sensor_data))

    @staticmethod
    def _IMU_callback(weak_self, sensor_data):
        self = weak_self()
        if not self:
            return
        limits = (-99.9, 99.9)
        self.accelerometer = (
            max(limits[0], min(limits[1], sensor_data.accelerometer.x)),
            max(limits[0], min(limits[1], sensor_data.accelerometer.y)),
            max(limits[0], min(limits[1], sensor_data.accelerometer.z)))
        self.gyroscope = (
            max(limits[0], min(limits[1], math.degrees(sensor_data.gyroscope.x))),
            max(limits[0], min(limits[1], math.degrees(sensor_data.gyroscope.y))),
            max(limits[0], min(limits[1], math.degrees(sensor_data.gyroscope.z))))
        self.compass = math.degrees(sensor_data.compass)


# ==============================================================================
# -- RadarSensor ---------------------------------------------------------------
# ==============================================================================


class RadarSensor(object):
    def __init__(self, parent_actor):
        self.sensor = None
        self._parent = parent_actor
        bound_x = 0.5 + self._parent.bounding_box.extent.x
        bound_y = 0.5 + self._parent.bounding_box.extent.y
        bound_z = 0.5 + self._parent.bounding_box.extent.z

        self.velocity_range = 7.5 # m/s
        world = self._parent.get_world()
        self.debug = world.debug
        bp = world.get_blueprint_library().find('sensor.other.radar')
        bp.set_attribute('horizontal_fov', str(35))
        bp.set_attribute('vertical_fov', str(20))
        self.sensor = world.spawn_actor(
            bp,
            carla.Transform(
                carla.Location(x=bound_x + 0.05, z=bound_z+0.05),
                carla.Rotation(pitch=5)),
            attach_to=self._parent)
        # We need a weak reference to self to avoid circular reference.
        weak_self = weakref.ref(self)
        self.sensor.listen(
            lambda radar_data: RadarSensor._Radar_callback(weak_self, radar_data))

    @staticmethod
    def _Radar_callback(weak_self, radar_data):
        self = weak_self()
        if not self:
            return
        # To get a numpy [[vel, altitude, azimuth, depth],...[,,,]]:
        # points = np.frombuffer(radar_data.raw_data, dtype=np.dtype('f4'))
        # points = np.reshape(points, (len(radar_data), 4))

        current_rot = radar_data.transform.rotation
        for detect in radar_data:
            azi = math.degrees(detect.azimuth)
            alt = math.degrees(detect.altitude)
            # The 0.25 adjusts a bit the distance so the dots can
            # be properly seen
            fw_vec = carla.Vector3D(x=detect.depth - 0.25)
            carla.Transform(
                carla.Location(),
                carla.Rotation(
                    pitch=current_rot.pitch + alt,
                    yaw=current_rot.yaw + azi,
                    roll=current_rot.roll)).transform(fw_vec)

            def clamp(min_v, max_v, value):
                return max(min_v, min(value, max_v))

            norm_velocity = detect.velocity / self.velocity_range # range [-1, 1]
            r = int(clamp(0.0, 1.0, 1.0 - norm_velocity) * 255.0)
            g = int(clamp(0.0, 1.0, 1.0 - abs(norm_velocity)) * 255.0)
            b = int(abs(clamp(- 1.0, 0.0, - 1.0 - norm_velocity)) * 255.0)
            self.debug.draw_point(
                radar_data.transform.location + fw_vec,
                size=0.075,
                life_time=0.06,
                persistent_lines=False,
                color=carla.Color(r, g, b))


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
    argparser.add_argument(
        '--sync',
        action='store_true',
        help='Activate synchronous mode execution')
    argparser.add_argument(
        '--destroy',
        action='store_true',
        default=False,
        help='Destroy vehicles')
    
    args = argparser.parse_args()

    ##############################################################################################

    vehicles_list = []
    traffic_list = []
    walkers_list = []
    all_id = []
    client = carla.Client(args.host, args.port)
    client.set_timeout(10.0)
    synchronous_master = False
    random.seed(args.seed if args.seed is not None else int(time.time()))

    try:
        world = client.get_world()
        settings = world.get_settings()
        vehicles_list = world.get_actors().filter('vehicle.*')
        print(f"vehicle number: {len(vehicles_list)}")
        ###########################################################################################
        # if args.destroy:
        #     print("Generating a vehicle...")
        #     vehicle = Vehicle(world, args)
        #     vehicle.generate_vehicle(spawn_location[0], rotation=0)
        #     print(f"Vehicle gnss: {vehicle.gnss_sensor}")
        #     print(f"vehicle number: {len(vehicles_list)}")

        # spawn_points = world.get_map().get_spawn_points()
        # number_of_spawn_points = len(spawn_points)
        # print("(wen) There are %d spawn points:",number_of_spawn_points)

        # for n, transform in enumerate(spawn_points[:3]):
        #     print("Generating a vehicle...")
        #     print(f"at location({transform})")
        #     vehicle = Vehicle(world, args)
        #     vehicle.generate_vehicle(transform, rotation=0)

        ###########################################################################################

        blueprint_list = ["tesla.cybertruck","ford.mustang", "volkswagen.t2", "ford.crown","mercedes.coupe","audi.a2",
                          "nissan.micra", "tesla.model3","nissan.patrol", "jeep.wrangler_rubicon", "dodge.charger_2020",
                            "audi.etron", "mini.cooper_s_2021", ]
        
        # if not blueprints:
        #     raise ValueError("Couldn't find any vehicles with the specified filters")
        
        # blueprint = blueprints[0]
        actor_list = world.get_actors()
        number_of_actors = len(actor_list)
        print(f"There are {number_of_actors} actor in this world. ")

        actorVehicle_list = world.get_actors().filter('vehicle.*')
        actorTraffic_list = world.get_actors().filter('traffic.*')
        vehicles_list = []
        traffic_list = []
        for n, actor in enumerate(actorVehicle_list): # calculate numbers of vehicle
            print(f"find a vehicle (id:{actor.id})")
            vehicles_list.append(actor)

        print(f"\nVehicle num: {len(actorVehicle_list)}, Traffic num: {len(traffic_list)}")
        if len(vehicles_list)<2:
            # spawn_points = random_spawnpoints(15)
            spawn_points = random.sample(load_parking_points(), 12)
            for i, location in enumerate(spawn_points): # make sure only one vehicle spawned
                # print(f"spawn_points: {location}")
                # x = location[0]
                # y = location[1]
                # z = 0.5
                # transform = carla.Transform(carla.Location(x=x, y=y, z=z), carla.Rotation(yaw=0))
                filterv = random.choice(blueprint_list)
                # print(f"filterv: {filterv}")
                # blueprints = get_actor_blueprints(world, "vehicle."+filterv, args.generationv) # fix blueprints
                blueprints = get_actor_blueprints(world, args.filterv, args.generationv)
                blueprints = [x for x in blueprints if x.get_attribute('base_type') == 'car']
                blueprint = random.choice(blueprints)
                vehicle_actor = world.try_spawn_actor(blueprint, location)
                while vehicle_actor is None:
                    print(f" ({x}, {y}, {z})Spawn failed, adjusting position slightly...")
                    z += 0.1  # 嘗試略微偏移位置
                    x += 0.5
                    transform = carla.Transform(carla.Location(x=x, y=y, z=z), carla.Rotation(yaw=0))
                    vehicle_actor = world.try_spawn_actor(blueprint, transform)
                # actor_gnss = GnssSensor(vehicle_actor)
                # print(actor_gnss.gnss_sensor.lat, actor_gnss.gnss_sensor.lon)

                world.tick()

                # **拉起手煞車，讓車輛保持靜止**
                vehicle_actor.apply_control(carla.VehicleControl(throttle=0.0, brake=0.0, hand_brake=True))

                vehicles_list.append(vehicle_actor)
                # print(f"Generate vehicle actor{i+1}: {vehicle_actor}")
                # print(f"type={vehicle_actor.type_id[8:]}")
                print(f"vehicle {i+1} (id:{vehicle_actor.id}, type={vehicle_actor.type_id[8:]}) spawned at \n{vehicle_actor.get_location()}")
        
        print('\nspawned %d vehicles and %d walkers, press Ctrl+C to exit.' % (len(vehicles_list), len(walkers_list)))

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
        
        if args.destroy:
            
            # for x in vehicles_list:
            #     print("Destroying sensors...")
                # attrs = vars(x)
                # print(', '.join("%s: %s" % item for item in attrs.items()))
                # sensors = [
                # x.gnss_sensor.sensor]#,
                # #x.imu_sensor.sensor]
                # for sensor in sensors:
                #     if sensor is not None:
                #         sensor.stop()
                #         sensor.destroy()
                # x.destroy()
            print('\ndestroying %d vehicles' % len(vehicles_list))
            for i in vehicles_list:
                print(f"id: {i.id}")
            gnss_list = world.get_actors().filter("sensor.other.gnss")
            client.apply_batch([carla.command.DestroyActor(x) for x in vehicles_list])
            client.apply_batch([carla.command.DestroyActor(x) for x in gnss_list])


        # # stop walker controllers (list is [controller, actor, controller, actor ...])
        # for i in range(0, len(all_id), 2):
        #     all_actors[i].stop()

        # print('\ndestroying %d walkers' % len(walkers_list))
        # client.apply_batch([carla.command.DestroyActor(x) for x in all_id])

        time.sleep(0.5)


if __name__ == '__main__':

    try:
        main()
    except KeyboardInterrupt:
        pass
    finally:
        print('\ndone.')