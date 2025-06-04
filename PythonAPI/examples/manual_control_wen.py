#!/usr/bin/env python

# Copyright (c) 2019 Computer Vision Center (CVC) at the Universitat Autonoma de
# Barcelona (UAB).
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

# Allows controlling a vehicle with a keyboard. For a simpler and more
# documented example, please take a look at tutorial.py.

"""
Welcome to CARLA manual control.

Use ARROWS or WASD keys for control.

    W            : throttle
    S            : brake
    A/D          : steer left/right
    Q            : toggle reverse
    Space        : hand-brake
    P            : toggle autopilot
    M            : toggle manual transmission
    ,/.          : gear up/down
    CTRL + W     : toggle constant velocity mode at 60 km/h

    L            : toggle next light type
    SHIFT + L    : toggle high beam
    Z/X          : toggle right/left blinker
    I            : toggle interior light

    TAB          : change sensor position
    ` or N       : next sensor
    [1-9]        : change to sensor [1-9]
    G            : toggle radar visualization
    C            : change weather (Shift+C reverse)
    Backspace    : change vehicle

    O            : open/close all doors of vehicle
    T            : toggle vehicle's telemetry

    V            : Select next map layer (Shift+V reverse)
    B            : Load current selected map layer (Shift+B to unload)

    R            : toggle recording images to disk

    CTRL + R     : toggle recording of simulation (replacing any previous)
    CTRL + P     : start replaying last recorded simulation
    CTRL + +     : increments the start time of the replay by 1 second (+SHIFT = 10 seconds)
    CTRL + -     : decrements the start time of the replay by 1 second (+SHIFT = 10 seconds)

    F1           : toggle HUD
    H/?          : toggle help
    ESC          : quit
"""

from __future__ import print_function


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

from carla import ColorConverter as cc

import argparse
import collections
import datetime
import logging
import math
import random
import re
import weakref
import cv2
import time
import threading
import queue
from concurrent.futures import ThreadPoolExecutor

from pathlib import Path
from utilsWen.general import increment_path

try:
    import pygame
    from pygame.locals import KMOD_CTRL
    from pygame.locals import KMOD_SHIFT
    from pygame.locals import K_0
    from pygame.locals import K_9
    from pygame.locals import K_BACKQUOTE
    from pygame.locals import K_BACKSPACE
    from pygame.locals import K_COMMA
    from pygame.locals import K_DOWN
    from pygame.locals import K_ESCAPE
    from pygame.locals import K_F1
    from pygame.locals import K_LEFT
    from pygame.locals import K_PERIOD
    from pygame.locals import K_RIGHT
    from pygame.locals import K_SLASH
    from pygame.locals import K_SPACE
    from pygame.locals import K_TAB
    from pygame.locals import K_UP
    from pygame.locals import K_a
    from pygame.locals import K_b
    from pygame.locals import K_c
    from pygame.locals import K_d
    from pygame.locals import K_f
    from pygame.locals import K_g
    from pygame.locals import K_h
    from pygame.locals import K_i
    from pygame.locals import K_l
    from pygame.locals import K_m
    from pygame.locals import K_n
    from pygame.locals import K_o
    from pygame.locals import K_p
    from pygame.locals import K_q
    from pygame.locals import K_r
    from pygame.locals import K_s
    from pygame.locals import K_t
    from pygame.locals import K_v
    from pygame.locals import K_w
    from pygame.locals import K_x
    from pygame.locals import K_z
    from pygame.locals import K_MINUS
    from pygame.locals import K_EQUALS
except ImportError:
    raise RuntimeError('cannot import pygame, make sure pygame package is installed')

try:
    import numpy as np
except ImportError:
    raise RuntimeError('cannot import numpy, make sure numpy package is installed')

VIEW_WIDTH = 1280 # 1920//2
VIEW_HEIGHT = 720 # 1080//2
VIEW_FOV = 90

BB_COLOR = (248, 64, 24)

# ==============================================================================
# -- Global functions ----------------------------------------------------------
# ==============================================================================
# save_path = increment_path(Path('Recording/Recording_1'), exist_ok=True)
# save_path = increment_path('Recording','Recording_')
# print(f"increment_path: {save_path}")
# a = Path("Recording/" + save_path)
# a.mkdir(parents=True, exist_ok=True)  # make dir
# print(f"Save path: {a}")
# save_path = 'Recording/Recording_5'

# spawn location:
fix_x = 105
fix_y = -137.5
fix_z = 1

def find_weather_presets():
    rgx = re.compile('.+?(?:(?<=[a-z])(?=[A-Z])|(?<=[A-Z])(?=[A-Z][a-z])|$)')
    name = lambda x: ' '.join(m.group(0) for m in rgx.finditer(x))
    presets = [x for x in dir(carla.WeatherParameters) if re.match('[A-Z].+', x)]
    return [(getattr(carla.WeatherParameters, x), name(x)) for x in presets]


def get_actor_display_name(actor, truncate=250):
    name = ' '.join(actor.type_id.replace('_', '.').title().split('.')[1:])
    return (name[:truncate - 1] + u'\u2026') if len(name) > truncate else name

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
# -- World ---------------------------------------------------------------------
# ==============================================================================


class World(object):
    def __init__(self, carla_world, hud, args):
        self.world = carla_world
        self.sync = args.sync
        self.actor_role_name = args.rolename
        try:
            self.map = self.world.get_map()
        except RuntimeError as error:
            print('RuntimeError: {}'.format(error))
            print('  The server could not send the OpenDRIVE (.xodr) file:')
            print('  Make sure it exists, has the same name of your town, and is correct.')
            sys.exit(1)
        self.hud = hud
        self.player = None
        self.collision_sensor = None
        self.lane_invasion_sensor = None
        self.gnss_sensor = None
        self.imu_sensor = None
        self.radar_sensor = None
        self.camera_manager = None
        self._weather_presets = find_weather_presets()
        self._weather_index = 0
        self._actor_filter = args.filter
        self._actor_generation = args.generation
        self._gamma = args.gamma
        self.restart()
        self.world.on_tick(hud.on_world_tick)
        self.recording_enabled = False
        self.recording_start = 0
        self.constant_velocity_enabled = False
        self.show_vehicle_telemetry = False
        self.doors_are_open = False
        self.current_map_layer = 0
        self.map_layer_names = [
            carla.MapLayer.NONE,
            carla.MapLayer.Buildings,
            carla.MapLayer.Decals,
            carla.MapLayer.Foliage,
            carla.MapLayer.Ground,
            carla.MapLayer.ParkedVehicles,
            carla.MapLayer.Particles,
            carla.MapLayer.Props,
            carla.MapLayer.StreetLights,
            carla.MapLayer.Walls,
            carla.MapLayer.All
        ]

    def restart(self):
        self.player_max_speed = 1.589
        self.player_max_speed_fast = 3.713
        # Keep same camera config if the camera manager exists.
        cam_index = self.camera_manager.index if self.camera_manager is not None else 0
        cam_pos_index = self.camera_manager.transform_index if self.camera_manager is not None else 0
        # Get a random blueprint.
        blueprint_list = get_actor_blueprints(self.world, "vehicle.dodge.charger_police_2020", self._actor_generation) # self._actor_filter
        if not blueprint_list:
            raise ValueError("Couldn't find any blueprints with the specified filters")
        blueprint = random.choice(blueprint_list)
        blueprint.set_attribute('role_name', self.actor_role_name)
        if blueprint.has_attribute('terramechanics'):
            blueprint.set_attribute('terramechanics', 'true')
        if blueprint.has_attribute('color'):
            color = random.choice(blueprint.get_attribute('color').recommended_values)
            blueprint.set_attribute('color', color)
        if blueprint.has_attribute('driver_id'):
            driver_id = random.choice(blueprint.get_attribute('driver_id').recommended_values)
            blueprint.set_attribute('driver_id', driver_id)
        if blueprint.has_attribute('is_invincible'):
            blueprint.set_attribute('is_invincible', 'true')
        # set the max speed
        if blueprint.has_attribute('speed'):
            self.player_max_speed = float(blueprint.get_attribute('speed').recommended_values[1])
            self.player_max_speed_fast = float(blueprint.get_attribute('speed').recommended_values[2])

        # Spawn the player.
        if self.player is not None:
            spawn_point = self.player.get_transform()
            spawn_point.location.z += 2.0
            spawn_point.rotation.roll = 0.0
            spawn_point.rotation.pitch = 0.0
            self.destroy()
            self.player = self.world.try_spawn_actor(blueprint, spawn_point)
            self.show_vehicle_telemetry = False
            self.modify_vehicle_physics(self.player)
        while self.player is None:
            if not self.map.get_spawn_points():
                print('There are no spawn points available in your map/town.')
                print('Please add some Vehicle Spawn Point to your UE4 scene.')
                sys.exit(1)
            spawn_points = self.map.get_spawn_points()
            spawn_point = random.choice(spawn_points) if spawn_points else carla.Transform()
            x = fix_x #105 # 107
            y = fix_y #-137.5 #-319.8
            z = fix_z #1

            fix_point = carla.Transform(carla.Location(x=x,y=y, z=z), carla.Rotation(yaw=0)) # (82.0, 0, yaw=270): songshin road, (117,-322,0): grass
            self.player = self.world.try_spawn_actor(blueprint, fix_point)

            
            while self.player is None:
                print(f" ({x}, {y})Spawn failed, adjusting position slightly...")
                x += 1  # 嘗試略微偏移位置
                transform = carla.Transform(carla.Location(x=x, y=y, z=z), carla.Rotation(yaw=0))
                self.player = self.world.try_spawn_actor(blueprint, transform)

            self.show_vehicle_telemetry = False
            self.modify_vehicle_physics(self.player)
            print(f"Spawn at location:({self.player.get_location()})")
        # Set up the sensors.
        self.collision_sensor = CollisionSensor(self.player, self.hud)
        self.lane_invasion_sensor = LaneInvasionSensor(self.player, self.hud)
        self.gnss_sensor = GnssSensor(self.player)
        self.imu_sensor = IMUSensor(self.player)
        self.camera_manager = CameraManager(self.player, self.gnss_sensor, self.hud, self._gamma)
        self.camera_manager.transform_index = cam_pos_index
        self.camera_manager.set_sensor(cam_index, notify=False)
        self.camera_manager.get_camera_params()
        actor_type = get_actor_display_name(self.player)
        self.hud.notification(actor_type)

        if self.sync:
            self.world.tick()
        else:
            self.world.wait_for_tick()

    def next_weather(self, reverse=False):
        self._weather_index += -1 if reverse else 1
        self._weather_index %= len(self._weather_presets)
        preset = self._weather_presets[self._weather_index]
        self.hud.notification('Weather: %s' % preset[1])
        self.player.get_world().set_weather(preset[0])

    def next_map_layer(self, reverse=False):
        self.current_map_layer += -1 if reverse else 1
        self.current_map_layer %= len(self.map_layer_names)
        selected = self.map_layer_names[self.current_map_layer]
        self.hud.notification('LayerMap selected: %s' % selected)

    def load_map_layer(self, unload=False):
        selected = self.map_layer_names[self.current_map_layer]
        if unload:
            self.hud.notification('Unloading map layer: %s' % selected)
            self.world.unload_map_layer(selected)
        else:
            self.hud.notification('Loading map layer: %s' % selected)
            self.world.load_map_layer(selected)

    def toggle_radar(self):
        if self.radar_sensor is None:
            self.radar_sensor = RadarSensor(self.player)
        elif self.radar_sensor.sensor is not None:
            self.radar_sensor.sensor.destroy()
            self.radar_sensor = None

    def modify_vehicle_physics(self, actor):
        #If actor is not a vehicle, we cannot use the physics control
        try:
            physics_control = actor.get_physics_control()
            physics_control.use_sweep_wheel_collision = True
            actor.apply_physics_control(physics_control)
        except Exception:
            pass

    def tick(self, clock):
        self.hud.tick(self, clock)

    def render(self, display):
        self.camera_manager.render(display)
        self.hud.render(display)

    def destroy_sensors(self):
        self.camera_manager.sensor.destroy()
        self.camera_manager.sensor = None
        self.camera_manager.index = None

    def destroy(self):
        if self.radar_sensor is not None:
            self.toggle_radar()
        sensors = [
            self.camera_manager.sensor,
            self.collision_sensor.sensor,
            self.lane_invasion_sensor.sensor,
            self.gnss_sensor.sensor,
            self.imu_sensor.sensor]
        for sensor in sensors:
            if sensor is not None:
                sensor.stop()
                sensor.destroy()
        if self.player is not None:
            self.player.destroy()


# ==============================================================================
# -- KeyboardControl -----------------------------------------------------------
# ==============================================================================


class KeyboardControl(object):
    """Class that handles keyboard input."""
    def __init__(self, world, start_in_autopilot):
        self._autopilot_enabled = start_in_autopilot
        self._ackermann_enabled = False
        self._ackermann_reverse = 1
        if isinstance(world.player, carla.Vehicle):
            self._control = carla.VehicleControl()
            self._ackermann_control = carla.VehicleAckermannControl()
            self._lights = carla.VehicleLightState.NONE
            world.player.set_autopilot(self._autopilot_enabled)
            world.player.set_light_state(self._lights)
        elif isinstance(world.player, carla.Walker):
            self._control = carla.WalkerControl()
            self._autopilot_enabled = False
            self._rotation = world.player.get_transform().rotation
        else:
            raise NotImplementedError("Actor type not supported")
        self._steer_cache = 0.0
        world.hud.notification("Press 'H' or '?' for help.", seconds=4.0)

    def parse_events(self, client, world, clock, sync_mode):
        if isinstance(self._control, carla.VehicleControl):
            current_lights = self._lights
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                return True
            elif event.type == pygame.KEYUP:
                if self._is_quit_shortcut(event.key):
                    return True
                elif event.key == K_BACKSPACE:
                    if self._autopilot_enabled:
                        world.player.set_autopilot(False)
                        world.restart()
                        world.player.set_autopilot(True)
                    else:
                        world.restart()
                elif event.key == K_F1:
                    world.hud.toggle_info()
                elif event.key == K_v and pygame.key.get_mods() & KMOD_SHIFT:
                    world.next_map_layer(reverse=True)
                elif event.key == K_v:
                    world.next_map_layer()
                elif event.key == K_b and pygame.key.get_mods() & KMOD_SHIFT:
                    world.load_map_layer(unload=True)
                elif event.key == K_b:
                    world.load_map_layer()
                elif event.key == K_h or (event.key == K_SLASH and pygame.key.get_mods() & KMOD_SHIFT):
                    world.hud.help.toggle()
                elif event.key == K_TAB:
                    world.camera_manager.toggle_camera()
                elif event.key == K_c and pygame.key.get_mods() & KMOD_SHIFT:
                    world.next_weather(reverse=True)
                elif event.key == K_c:
                    world.next_weather()
                elif event.key == K_g:
                    world.toggle_radar()
                elif event.key == K_BACKQUOTE:
                    world.camera_manager.next_sensor()
                elif event.key == K_n:
                    world.camera_manager.next_sensor()
                elif event.key == K_w and (pygame.key.get_mods() & KMOD_CTRL):
                    if world.constant_velocity_enabled:
                        world.player.disable_constant_velocity()
                        world.constant_velocity_enabled = False
                        world.hud.notification("Disabled Constant Velocity Mode")
                    else:
                        world.player.enable_constant_velocity(carla.Vector3D(17, 0, 0))
                        world.constant_velocity_enabled = True
                        world.hud.notification("Enabled Constant Velocity Mode at 60 km/h")
                elif event.key == K_o:
                    try:
                        if world.doors_are_open:
                            world.hud.notification("Closing Doors")
                            world.doors_are_open = False
                            world.player.close_door(carla.VehicleDoor.All)
                        else:
                            world.hud.notification("Opening doors")
                            world.doors_are_open = True
                            world.player.open_door(carla.VehicleDoor.All)
                    except Exception:
                        pass
                elif event.key == K_t:
                    if world.show_vehicle_telemetry:
                        world.player.show_debug_telemetry(False)
                        world.show_vehicle_telemetry = False
                        world.hud.notification("Disabled Vehicle Telemetry")
                    else:
                        try:
                            world.player.show_debug_telemetry(True)
                            world.show_vehicle_telemetry = True
                            world.hud.notification("Enabled Vehicle Telemetry")
                        except Exception:
                            pass
                elif event.key > K_0 and event.key <= K_9:
                    index_ctrl = 0
                    if pygame.key.get_mods() & KMOD_CTRL:
                        index_ctrl = 9
                    world.camera_manager.set_sensor(event.key - 1 - K_0 + index_ctrl)
                elif event.key == K_r and not (pygame.key.get_mods() & KMOD_CTRL):
                    world.camera_manager.toggle_recording()
                elif event.key == K_r and (pygame.key.get_mods() & KMOD_CTRL):
                    if (world.recording_enabled):
                        client.stop_recorder()
                        world.recording_enabled = False
                        world.hud.notification("Recorder is OFF")
                    else:
                        client.start_recorder("manual_recording.rec")
                        world.recording_enabled = True
                        world.hud.notification("Recorder is ON")
                elif event.key == K_p and (pygame.key.get_mods() & KMOD_CTRL):
                    # stop recorder
                    client.stop_recorder()
                    world.recording_enabled = False
                    # work around to fix camera at start of replaying
                    current_index = world.camera_manager.index
                    world.destroy_sensors()
                    # disable autopilot
                    self._autopilot_enabled = False
                    world.player.set_autopilot(self._autopilot_enabled)
                    world.hud.notification("Replaying file 'manual_recording.rec'")
                    # replayer
                    client.replay_file("manual_recording.rec", world.recording_start, 0, 0)
                    world.camera_manager.set_sensor(current_index)
                elif event.key == K_MINUS and (pygame.key.get_mods() & KMOD_CTRL):
                    if pygame.key.get_mods() & KMOD_SHIFT:
                        world.recording_start -= 10
                    else:
                        world.recording_start -= 1
                    world.hud.notification("Recording start time is %d" % (world.recording_start))
                elif event.key == K_EQUALS and (pygame.key.get_mods() & KMOD_CTRL):
                    if pygame.key.get_mods() & KMOD_SHIFT:
                        world.recording_start += 10
                    else:
                        world.recording_start += 1
                    world.hud.notification("Recording start time is %d" % (world.recording_start))
                if isinstance(self._control, carla.VehicleControl):
                    if event.key == K_f:
                        # Toggle ackermann controller
                        self._ackermann_enabled = not self._ackermann_enabled
                        world.hud.show_ackermann_info(self._ackermann_enabled)
                        world.hud.notification("Ackermann Controller %s" %
                                               ("Enabled" if self._ackermann_enabled else "Disabled"))
                    if event.key == K_q:
                        if not self._ackermann_enabled:
                            self._control.gear = 1 if self._control.reverse else -1
                        else:
                            self._ackermann_reverse *= -1
                            # Reset ackermann control
                            self._ackermann_control = carla.VehicleAckermannControl()
                    elif event.key == K_m:
                        self._control.manual_gear_shift = not self._control.manual_gear_shift
                        self._control.gear = world.player.get_control().gear
                        world.hud.notification('%s Transmission' %
                                               ('Manual' if self._control.manual_gear_shift else 'Automatic'))
                    elif self._control.manual_gear_shift and event.key == K_COMMA:
                        self._control.gear = max(-1, self._control.gear - 1)
                    elif self._control.manual_gear_shift and event.key == K_PERIOD:
                        self._control.gear = self._control.gear + 1
                    elif event.key == K_p and not pygame.key.get_mods() & KMOD_CTRL:
                        if not self._autopilot_enabled and not sync_mode:
                            print("WARNING: You are currently in asynchronous mode and could "
                                  "experience some issues with the traffic simulation")
                        self._autopilot_enabled = not self._autopilot_enabled
                        world.player.set_autopilot(self._autopilot_enabled)
                        world.hud.notification(
                            'Autopilot %s' % ('On' if self._autopilot_enabled else 'Off'))
                    elif event.key == K_l and pygame.key.get_mods() & KMOD_CTRL:
                        current_lights ^= carla.VehicleLightState.Special1
                    elif event.key == K_l and pygame.key.get_mods() & KMOD_SHIFT:
                        current_lights ^= carla.VehicleLightState.HighBeam
                    elif event.key == K_l:
                        # Use 'L' key to switch between lights:
                        # closed -> position -> low beam -> fog
                        if not self._lights & carla.VehicleLightState.Position:
                            world.hud.notification("Position lights")
                            current_lights |= carla.VehicleLightState.Position
                        else:
                            world.hud.notification("Low beam lights")
                            current_lights |= carla.VehicleLightState.LowBeam
                        if self._lights & carla.VehicleLightState.LowBeam:
                            world.hud.notification("Fog lights")
                            current_lights |= carla.VehicleLightState.Fog
                        if self._lights & carla.VehicleLightState.Fog:
                            world.hud.notification("Lights off")
                            current_lights ^= carla.VehicleLightState.Position
                            current_lights ^= carla.VehicleLightState.LowBeam
                            current_lights ^= carla.VehicleLightState.Fog
                    elif event.key == K_i:
                        current_lights ^= carla.VehicleLightState.Interior
                    elif event.key == K_z:
                        current_lights ^= carla.VehicleLightState.LeftBlinker
                    elif event.key == K_x:
                        current_lights ^= carla.VehicleLightState.RightBlinker

        if not self._autopilot_enabled:
            if isinstance(self._control, carla.VehicleControl):
                self._parse_vehicle_keys(pygame.key.get_pressed(), clock.get_time())
                self._control.reverse = self._control.gear < 0
                # Set automatic control-related vehicle lights
                if self._control.brake:
                    current_lights |= carla.VehicleLightState.Brake
                else: # Remove the Brake flag
                    current_lights &= ~carla.VehicleLightState.Brake
                if self._control.reverse:
                    current_lights |= carla.VehicleLightState.Reverse
                else: # Remove the Reverse flag
                    current_lights &= ~carla.VehicleLightState.Reverse
                if current_lights != self._lights: # Change the light state only if necessary
                    self._lights = current_lights
                    world.player.set_light_state(carla.VehicleLightState(self._lights))
                # Apply control
                if not self._ackermann_enabled:
                    world.player.apply_control(self._control)
                else:
                    world.player.apply_ackermann_control(self._ackermann_control)
                    # Update control to the last one applied by the ackermann controller.
                    self._control = world.player.get_control()
                    # Update hud with the newest ackermann control
                    world.hud.update_ackermann_control(self._ackermann_control)

            elif isinstance(self._control, carla.WalkerControl):
                self._parse_walker_keys(pygame.key.get_pressed(), clock.get_time(), world)
                world.player.apply_control(self._control)

    def _parse_vehicle_keys(self, keys, milliseconds):
        if keys[K_UP] or keys[K_w]:
            if not self._ackermann_enabled:
                self._control.throttle = min(self._control.throttle + 0.1, 1.00)
            else:
                self._ackermann_control.speed += round(milliseconds * 0.005, 2) * self._ackermann_reverse
        else:
            if not self._ackermann_enabled:
                self._control.throttle = 0.0

        if keys[K_DOWN] or keys[K_s]:
            if not self._ackermann_enabled:
                self._control.brake = min(self._control.brake + 0.2, 1)
            else:
                self._ackermann_control.speed -= min(abs(self._ackermann_control.speed), round(milliseconds * 0.005, 2)) * self._ackermann_reverse
                self._ackermann_control.speed = max(0, abs(self._ackermann_control.speed)) * self._ackermann_reverse
        else:
            if not self._ackermann_enabled:
                self._control.brake = 0

        steer_increment = 5e-4 * milliseconds
        if keys[K_LEFT] or keys[K_a]:
            if self._steer_cache > 0:
                self._steer_cache = 0
            else:
                self._steer_cache -= steer_increment
        elif keys[K_RIGHT] or keys[K_d]:
            if self._steer_cache < 0:
                self._steer_cache = 0
            else:
                self._steer_cache += steer_increment
        else:
            self._steer_cache = 0.0
        self._steer_cache = min(0.7, max(-0.7, self._steer_cache))
        if not self._ackermann_enabled:
            self._control.steer = round(self._steer_cache, 1)
            self._control.hand_brake = keys[K_SPACE]
        else:
            self._ackermann_control.steer = round(self._steer_cache, 1)

    def _parse_walker_keys(self, keys, milliseconds, world):
        self._control.speed = 0.0
        if keys[K_DOWN] or keys[K_s]:
            self._control.speed = 0.0
        if keys[K_LEFT] or keys[K_a]:
            self._control.speed = .01
            self._rotation.yaw -= 0.08 * milliseconds
        if keys[K_RIGHT] or keys[K_d]:
            self._control.speed = .01
            self._rotation.yaw += 0.08 * milliseconds
        if keys[K_UP] or keys[K_w]:
            self._control.speed = world.player_max_speed_fast if pygame.key.get_mods() & KMOD_SHIFT else world.player_max_speed
        self._control.jump = keys[K_SPACE]
        self._rotation.yaw = round(self._rotation.yaw, 1)
        self._control.direction = self._rotation.get_forward_vector()

    @staticmethod
    def _is_quit_shortcut(key):
        return (key == K_ESCAPE) or (key == K_q and pygame.key.get_mods() & KMOD_CTRL)


# ==============================================================================
# -- HUD -----------------------------------------------------------------------
# ==============================================================================


class HUD(object):
    def __init__(self, width, height):
        print(f"VIEW_WIDTH = {width}")
        print(f"VIEW_HEIGHT = {height}")
        self.dim = (width, height)
        font = pygame.font.Font(pygame.font.get_default_font(), 20)
        font_name = 'courier' if os.name == 'nt' else 'mono'
        fonts = [x for x in pygame.font.get_fonts() if font_name in x]
        default_font = 'ubuntumono'
        mono = default_font if default_font in fonts else fonts[0]
        mono = pygame.font.match_font(mono)
        self._font_mono = pygame.font.Font(mono, 12 if os.name == 'nt' else 14)
        self._notifications = FadingText(font, (width, 40), (0, height - 40))
        self.help = HelpText(pygame.font.Font(mono, 16), width, height)
        self.server_fps = 0
        self.frame = 0
        self.simulation_time = 0
        self._show_info = True
        self._info_text = []
        self._server_clock = pygame.time.Clock()

        self._show_ackermann_info = False
        self._ackermann_control = carla.VehicleAckermannControl()

    def on_world_tick(self, timestamp):
        self._server_clock.tick()
        self.server_fps = self._server_clock.get_fps()
        self.frame = timestamp.frame
        self.simulation_time = timestamp.elapsed_seconds

    def tick(self, world, clock):
        self._notifications.tick(world, clock)
        if not self._show_info:
            return
        t = world.player.get_transform()
        v = world.player.get_velocity()
        c = world.player.get_control()
        compass = world.imu_sensor.compass
        heading = 'N' if compass > 270.5 or compass < 89.5 else ''
        heading += 'S' if 90.5 < compass < 269.5 else ''
        heading += 'E' if 0.5 < compass < 179.5 else ''
        heading += 'W' if 180.5 < compass < 359.5 else ''
        colhist = world.collision_sensor.get_collision_history()
        collision = [colhist[x + self.frame - 200] for x in range(0, 200)]
        max_col = max(1.0, max(collision))
        collision = [x / max_col for x in collision]
        vehicles = world.world.get_actors().filter('vehicle.*')
        self._info_text = [
            'Server:  % 16.0f FPS' % self.server_fps,
            'Client:  % 16.0f FPS' % clock.get_fps(),
            '',
            'Vehicle: % 20s' % get_actor_display_name(world.player, truncate=20),
            'Map:     % 20s' % world.map.name.split('/')[-1],
            'Simulation time: % 12s' % datetime.timedelta(seconds=int(self.simulation_time)),
            '',
            'Speed:   % 15.0f km/h' % (3.6 * math.sqrt(v.x**2 + v.y**2 + v.z**2)),
            u'Compass:% 17.0f\N{DEGREE SIGN} % 2s' % (compass, heading),
            'Accelero: (%5.1f,%5.1f,%5.1f)' % (world.imu_sensor.accelerometer),
            'Gyroscop: (%5.1f,%5.1f,%5.1f)' % (world.imu_sensor.gyroscope),
            'Location:% 20s' % ('(% 5.1f, % 5.1f)' % (t.location.x, t.location.y)),
            'GNSS:% 24s' % ('(% 2.6f, % 3.6f)' % (world.gnss_sensor.lat, world.gnss_sensor.lon)),
            'Height:  % 18.0f m' % t.location.z,
            '']
        if isinstance(c, carla.VehicleControl):
            self._info_text += [
                ('Throttle:', c.throttle, 0.0, 1.0),
                ('Steer:', c.steer, -1.0, 1.0),
                ('Brake:', c.brake, 0.0, 1.0),
                ('Reverse:', c.reverse),
                ('Hand brake:', c.hand_brake),
                ('Manual:', c.manual_gear_shift),
                'Gear:        %s' % {-1: 'R', 0: 'N'}.get(c.gear, c.gear)]
            if self._show_ackermann_info:
                self._info_text += [
                    '',
                    'Ackermann Controller:',
                    '  Target speed: % 8.0f km/h' % (3.6*self._ackermann_control.speed),
                ]
        elif isinstance(c, carla.WalkerControl):
            self._info_text += [
                ('Speed:', c.speed, 0.0, 5.556),
                ('Jump:', c.jump)]
        self._info_text += [
            '',
            'Collision:',
            collision,
            '',
            'Number of vehicles: % 8d' % len(vehicles)]
        if len(vehicles) > 1:
            self._info_text += ['Nearby vehicles:']
            distance = lambda l: math.sqrt((l.x - t.location.x)**2 + (l.y - t.location.y)**2 + (l.z - t.location.z)**2)
            vehicles = [(distance(x.get_location()), x) for x in vehicles if x.id != world.player.id]
            for d, vehicle in sorted(vehicles, key=lambda vehicles: vehicles[0]):
                if d > 200.0:
                    break
                vehicle_type = get_actor_display_name(vehicle, truncate=22)
                self._info_text.append('% 4dm %s' % (d, vehicle_type))

    def show_ackermann_info(self, enabled):
        self._show_ackermann_info = enabled

    def update_ackermann_control(self, ackermann_control):
        self._ackermann_control = ackermann_control

    def toggle_info(self):
        self._show_info = not self._show_info

    def notification(self, text, seconds=2.0):
        self._notifications.set_text(text, seconds=seconds)

    def error(self, text):
        self._notifications.set_text('Error: %s' % text, (255, 0, 0))

    def render(self, display):
        if self._show_info:
            info_surface = pygame.Surface((220, self.dim[1]))
            info_surface.set_alpha(100)
            display.blit(info_surface, (0, 0))
            v_offset = 4
            bar_h_offset = 100
            bar_width = 106
            for item in self._info_text:
                if v_offset + 18 > self.dim[1]:
                    break
                if isinstance(item, list):
                    if len(item) > 1:
                        points = [(x + 8, v_offset + 8 + (1.0 - y) * 30) for x, y in enumerate(item)]
                        pygame.draw.lines(display, (255, 136, 0), False, points, 2)
                    item = None
                    v_offset += 18
                elif isinstance(item, tuple):
                    if isinstance(item[1], bool):
                        rect = pygame.Rect((bar_h_offset, v_offset + 8), (6, 6))
                        pygame.draw.rect(display, (255, 255, 255), rect, 0 if item[1] else 1)
                    else:
                        rect_border = pygame.Rect((bar_h_offset, v_offset + 8), (bar_width, 6))
                        pygame.draw.rect(display, (255, 255, 255), rect_border, 1)
                        f = (item[1] - item[2]) / (item[3] - item[2])
                        if item[2] < 0.0:
                            rect = pygame.Rect((bar_h_offset + f * (bar_width - 6), v_offset + 8), (6, 6))
                        else:
                            rect = pygame.Rect((bar_h_offset, v_offset + 8), (f * bar_width, 6))
                        pygame.draw.rect(display, (255, 255, 255), rect)
                    item = item[0]
                if item:  # At this point has to be a str.
                    surface = self._font_mono.render(item, True, (255, 255, 255))
                    display.blit(surface, (8, v_offset))
                v_offset += 18
        self._notifications.render(display)
        self.help.render(display)


# ==============================================================================
# -- FadingText ----------------------------------------------------------------
# ==============================================================================


class FadingText(object):
    def __init__(self, font, dim, pos):
        self.font = font
        self.dim = dim
        self.pos = pos
        self.seconds_left = 0
        self.surface = pygame.Surface(self.dim)

    def set_text(self, text, color=(255, 255, 255), seconds=2.0):
        text_texture = self.font.render(text, True, color)
        self.surface = pygame.Surface(self.dim)
        self.seconds_left = seconds
        self.surface.fill((0, 0, 0, 0))
        self.surface.blit(text_texture, (10, 11))

    def tick(self, _, clock):
        delta_seconds = 1e-3 * clock.get_time()
        self.seconds_left = max(0.0, self.seconds_left - delta_seconds)
        self.surface.set_alpha(500.0 * self.seconds_left)

    def render(self, display):
        display.blit(self.surface, self.pos)


# ==============================================================================
# -- HelpText ------------------------------------------------------------------
# ==============================================================================


class HelpText(object):
    """Helper class to handle text output using pygame"""
    def __init__(self, font, width, height):
        lines = __doc__.split('\n')
        self.font = font
        self.line_space = 18
        self.dim = (780, len(lines) * self.line_space + 12)
        self.pos = (0.5 * width - 0.5 * self.dim[0], 0.5 * height - 0.5 * self.dim[1])
        self.seconds_left = 0
        self.surface = pygame.Surface(self.dim)
        self.surface.fill((0, 0, 0, 0))
        for n, line in enumerate(lines):
            text_texture = self.font.render(line, True, (255, 255, 255))
            self.surface.blit(text_texture, (22, n * self.line_space))
            self._render = False
        self.surface.set_alpha(220)

    def toggle(self):
        self._render = not self._render

    def render(self, display):
        if self._render:
            display.blit(self.surface, self.pos)


# ==============================================================================
# -- CollisionSensor -----------------------------------------------------------
# ==============================================================================


class CollisionSensor(object):
    def __init__(self, parent_actor, hud):
        self.sensor = None
        self.history = []
        self._parent = parent_actor
        self.hud = hud
        world = self._parent.get_world()
        bp = world.get_blueprint_library().find('sensor.other.collision')
        self.sensor = world.spawn_actor(bp, carla.Transform(), attach_to=self._parent)
        # We need to pass the lambda a weak reference to self to avoid circular
        # reference.
        weak_self = weakref.ref(self)
        self.sensor.listen(lambda event: CollisionSensor._on_collision(weak_self, event))

    def get_collision_history(self):
        history = collections.defaultdict(int)
        for frame, intensity in self.history:
            history[frame] += intensity
        return history

    @staticmethod
    def _on_collision(weak_self, event):
        self = weak_self()
        if not self:
            return
        actor_type = get_actor_display_name(event.other_actor)
        self.hud.notification('Collision with %r' % actor_type)
        impulse = event.normal_impulse
        intensity = math.sqrt(impulse.x**2 + impulse.y**2 + impulse.z**2)
        self.history.append((event.frame, intensity))
        if len(self.history) > 4000:
            self.history.pop(0)


# ==============================================================================
# -- LaneInvasionSensor --------------------------------------------------------
# ==============================================================================


class LaneInvasionSensor(object):
    def __init__(self, parent_actor, hud):
        self.sensor = None

        # If the spawn object is not a vehicle, we cannot use the Lane Invasion Sensor
        if parent_actor.type_id.startswith("vehicle."):
            self._parent = parent_actor
            self.hud = hud
            world = self._parent.get_world()
            bp = world.get_blueprint_library().find('sensor.other.lane_invasion')
            self.sensor = world.spawn_actor(bp, carla.Transform(), attach_to=self._parent)
            # We need to pass the lambda a weak reference to self to avoid circular
            # reference.
            weak_self = weakref.ref(self)
            self.sensor.listen(lambda event: LaneInvasionSensor._on_invasion(weak_self, event))

    @staticmethod
    def _on_invasion(weak_self, event):
        self = weak_self()
        if not self:
            return
        lane_types = set(x.type for x in event.crossed_lane_markings)
        text = ['%r' % str(x).split()[-1] for x in lane_types]
        self.hud.notification('Crossed line %s' % ' and '.join(text))


# ==============================================================================
# -- GnssSensor ----------------------------------------------------------------
# ==============================================================================


class GnssSensor(object):
    def __init__(self, parent_actor):
        print("-- Gnss Sensor init --")
        print(f"parent_actor: {parent_actor}")
        self.sensor = None
        self._parent = parent_actor
        self.lat = 0.0
        self.lon = 0.0
        world = self._parent.get_world()
        bp = world.get_blueprint_library().find('sensor.other.gnss')
        self.sensor = world.spawn_actor(bp, carla.Transform(carla.Location(x=1.0, z=2.8)), attach_to=self._parent)
        # self.sensors.append(['sensor.other.gnss', None, 'GNSS', {}, self.sensor])
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
    
    # def get_gnss_data(self):
    #     """獲取最新的 GNSS 經緯度數據"""
    #     gnss_data = {"lat": self.lat, "lon": self.lon}
    #     return gnss_data

# ==============================================================================
# -- IMUSensor -----------//2

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

# ==============================================================================
# -- CameraManager -------------------------------------------------------------
# ==============================================================================


class CameraManager(object):
    def __init__(self, parent_actor, gnss_sensor, hud, gamma_correction):
        self.sensor = None
        self.surface = None
        self._parent = parent_actor
        self.gnss_sensor = gnss_sensor
        self.hud = hud
        self.recording = False
        self._last_location = None
        self.K = self.get_camera_intrinsic(hud.dim[0], hud.dim[1])
        self.image_queue = queue.Queue() # 沒用到
        self.executor = ThreadPoolExecutor(max_workers=4)  # 可依處理需求調整
        self.write_lock = threading.Lock()  # 新增一把鎖
        bound_x = 0.5 + self._parent.bounding_box.extent.x
        bound_y = 0.5 + self._parent.bounding_box.extent.y
        bound_z = 0.5 + self._parent.bounding_box.extent.z
        Attachment = carla.AttachmentType

        if not self._parent.type_id.startswith("walker.pedestrian"):
            self._camera_transforms = [
                (carla.Transform(carla.Location(x=-2.0*bound_x, y=+0.0*bound_y, z=2.0*bound_z), carla.Rotation(pitch=8.0)), Attachment.SpringArmGhost),
                (carla.Transform(carla.Location(x=+0.8*bound_x, y=+0.0*bound_y, z=1.3*bound_z)), Attachment.Rigid),
                (carla.Transform(carla.Location(x=+1.9*bound_x, y=+1.0*bound_y, z=1.2*bound_z)), Attachment.SpringArmGhost),
                (carla.Transform(carla.Location(x=-2.8*bound_x, y=+0.0*bound_y, z=4.6*bound_z), carla.Rotation(pitch=6.0)), Attachment.SpringArmGhost),
                (carla.Transform(carla.Location(x=-1.0, y=-1.0*bound_y, z=0.4*bound_z)), Attachment.Rigid)]
        else:
            self._camera_transforms = [
                (carla.Transform(carla.Location(x=-2.5, z=0.0), carla.Rotation(pitch=-8.0)), Attachment.SpringArmGhost),
                (carla.Transform(carla.Location(x=1.6, z=1.7)), Attachment.Rigid),
                (carla.Transform(carla.Location(x=2.5, y=0.5, z=0.0), carla.Rotation(pitch=-8.0)), Attachment.SpringArmGhost),
                (carla.Transform(carla.Location(x=-4.0, z=2.0), carla.Rotation(pitch=6.0)), Attachment.SpringArmGhost),
                (carla.Transform(carla.Location(x=0, y=-2.5, z=-0.0), carla.Rotation(yaw=90.0)), Attachment.Rigid)]

        self.transform_index = 1
        self.sensors = [
            ['sensor.camera.rgb', cc.Raw, 'Camera RGB', {}],
            ['sensor.camera.depth', cc.Raw, 'Camera Depth (Raw)', {}],
            ['sensor.camera.depth', cc.Depth, 'Camera Depth (Gray Scale)', {}],
            ['sensor.camera.depth', cc.LogarithmicDepth, 'Camera Depth (Logarithmic Gray Scale)', {}],
            ['sensor.camera.semantic_segmentation', cc.Raw, 'Camera Semantic Segmentation (Raw)', {}],
            ['sensor.camera.semantic_segmentation', cc.CityScapesPalette, 'Camera Semantic Segmentation (CityScapes Palette)', {}],
            ['sensor.camera.instance_segmentation', cc.CityScapesPalette, 'Camera Instance Segmentation (CityScapes Palette)', {}],
            ['sensor.camera.instance_segmentation', cc.Raw, 'Camera Instance Segmentation (Raw)', {}],
            ['sensor.lidar.ray_cast', None, 'Lidar (Ray-Cast)', {'range': '50'}],
            ['sensor.camera.dvs', cc.Raw, 'Dynamic Vision Sensor', {}],
            ['sensor.camera.rgb', cc.Raw, 'Camera RGB Distorted',
                {'lens_circle_multiplier': '3.0',
                'lens_circle_falloff': '3.0',
                'chromatic_aberration_intensity': '0.5',
                'chromatic_aberration_offset': '0'}],
            ['sensor.camera.optical_flow', cc.Raw, 'Optical Flow', {}],
            ['sensor.camera.normals', cc.Raw, 'Camera Normals', {}],
        ]
        world = self._parent.get_world()
        bp_library = world.get_blueprint_library()
        for item in self.sensors:
            bp = bp_library.find(item[0])
            if item[0].startswith('sensor.camera'):
                bp.set_attribute('image_size_x', str(hud.dim[0]))
                bp.set_attribute('image_size_y', str(hud.dim[1]))
                if bp.has_attribute('gamma'):
                    bp.set_attribute('gamma', str(gamma_correction))
                for attr_name, attr_value in item[3].items():
                    bp.set_attribute(attr_name, attr_value)
            elif item[0].startswith('sensor.lidar'):
                self.lidar_range = 50

                for attr_name, attr_value in item[3].items():
                    bp.set_attribute(attr_name, attr_value)
                    if attr_name == 'range':
                        self.lidar_range = float(attr_value)

            item.append(bp)
        self.index = None

    def toggle_camera(self):
        self.transform_index = (self.transform_index + 1) % len(self._camera_transforms)
        self.set_sensor(self.index, notify=False, force_respawn=True)
        self.get_camera_params()

    def set_sensor(self, index, notify=True, force_respawn=False):
        index = index % len(self.sensors)
        needs_respawn = True if self.index is None else \
            (force_respawn or (self.sensors[index][2] != self.sensors[self.index][2]))
        if needs_respawn:
            if self.sensor is not None:
                self.sensor.destroy()
                self.surface = None
            self.sensor = self._parent.get_world().spawn_actor(
                self.sensors[index][-1],
                self._camera_transforms[self.transform_index][0],
                attach_to=self._parent,
                attachment_type=self._camera_transforms[self.transform_index][1])
            # We need to pass the lambda a weak reference to self to avoid
            # circular reference.
            weak_self = weakref.ref(self)
            self.sensor.listen(lambda image: CameraManager._parse_image(weak_self, image))

            calibration = np.identity(3)
            calibration[0, 2] = self.hud.dim[0] / 2.0
            calibration[1, 2] = self.hud.dim[1] / 2.0
            calibration[0, 0] = calibration[1, 1] = self.hud.dim[0] / (2.0 * np.tan(VIEW_FOV * np.pi / 360.0))
            self.sensor.calibration = calibration
        if notify:
            self.hud.notification(self.sensors[index][2])
        self.index = index

    def next_sensor(self):
        self.set_sensor(self.index + 1)

    def toggle_recording(self):
        self.recording = not self.recording
        if self.recording:
            global save_path
            save_path = increment_path('Recording','Recording_')
            print(f"save path: {save_path}")
        else:
            # global save_path
            print(f"結束錄影，影像存於: {save_path}")
        self.hud.notification('Recording %s' % ('On' if self.recording else 'Off'))

    def render(self, display):
        if self.surface is not None:
            display.blit(self.surface, (0, 0))
        
    @staticmethod
    def _parse_image(weak_self, image):
        self = weak_self()
        if not self:
            return

        # 取得當前車輛位置
        start_time = time.time()  # 開始計時
        current_location = self._parent.get_transform().location

        # 如果前一次位置存在，且差距很小（例如小於 0.1 公尺），就跳過這張影像
        if self._last_location is not None:
            dx = current_location.x - self._last_location.x
            dy = current_location.y - self._last_location.y
            dz = current_location.z - self._last_location.z
            dist_sq = dx*dx + dy*dy + dz*dz
            if dist_sq < 0.01:  # 代表幾乎沒移動
                # print("車輛沒移動")
                return

        # 更新位置記錄
        self._last_location = current_location
        # print("更新位置")

        # print(f"self.sensors: {self.sensors}")
        if self.sensors[self.index][0].startswith('sensor.lidar'): # 處理 LIDAR 數據
            points = np.frombuffer(image.raw_data, dtype=np.dtype('f4'))
            points = np.reshape(points, (int(points.shape[0] / 4), 4))
            lidar_data = np.array(points[:, :2])
            lidar_data *= min(self.hud.dim) / (2.0 * self.lidar_range)
            lidar_data += (0.5 * self.hud.dim[0], 0.5 * self.hud.dim[1])
            lidar_data = np.fabs(lidar_data)  # pylint: disable=E1111
            lidar_data = lidar_data.astype(np.int32)
            lidar_data = np.reshape(lidar_data, (-1, 2))
            lidar_img_size = (self.hud.dim[0], self.hud.dim[1], 3)
            lidar_img = np.zeros((lidar_img_size), dtype=np.uint8)
            lidar_img[tuple(lidar_data.T)] = (255, 255, 255)
            self.surface = pygame.surfarray.make_surface(lidar_img)
        elif self.sensors[self.index][0].startswith('sensor.camera.dvs'): # 處理 DVS (Dynamic Vision Sensor) 影像
            # Example of converting the raw_data from a carla.DVSEventArray
            # sensor into a NumPy array and using it as an image
            dvs_events = np.frombuffer(image.raw_data, dtype=np.dtype([
                ('x', np.uint16), ('y', np.uint16), ('t', np.int64), ('pol', np.bool)]))
            dvs_img = np.zeros((image.height, image.width, 3), dtype=np.uint8)
            # Blue is positive, red is negative
            dvs_img[dvs_events[:]['y'], dvs_events[:]['x'], dvs_events[:]['pol'] * 2] = 255
            self.surface = pygame.surfarray.make_surface(dvs_img.swapaxes(0, 1))
        elif self.sensors[self.index][0].startswith('sensor.camera.optical_flow'): # 處理光流影像
            image = image.get_color_coded_flow()
            array = np.frombuffer(image.raw_data, dtype=np.dtype("uint8"))
            array = np.reshape(array, (image.height, image.width, 4))
            array = array[:, :, :3]
            array = array[:, :, ::-1]
            self.surface = pygame.surfarray.make_surface(array.swapaxes(0, 1))
        else: # 處理一般影像
            image.convert(self.sensors[self.index][1])
            array = np.frombuffer(image.raw_data, dtype=np.dtype("uint8"))
            array = np.reshape(array, (image.height, image.width, 4))
            array = array[:, :, :3]
            array = array[:, :, ::-1]
            self.surface = pygame.surfarray.make_surface(array.swapaxes(0, 1))
        # 結束計時（計算車輛移動）
        end_time = time.time()
        duration = end_time - start_time
        # print(f"處理基本影像費時：{duration:.10f} 秒")
        if self.recording:
            # self.image_queue.put(image)
            self.executor.submit(self.save_vehicle_info, image)
            # t = threading.Thread(target=self.save_vehicle_info()) # 建立新的執行緒
            # t.start() # 啟用執行緒

    def save_vehicle_info(self, image): #　self._parent，　self.gnss_sensor，　all_vehicles，　self.sensor
        start_time = time.time()  # 開始計時
        img = np.reshape(np.copy(image.raw_data), (image.height, image.width, 4))
         # 🚗 嘗試獲取 world
        try:
            world = self._parent.get_world()  # 確保 _parent 存在
        except AttributeError:
            print("----- get_world() error -----")
            world = image.actor.get_world()  # 從影像物件獲取世界

        # ✅ 使用 snapshot 來對齊影像時間點
        snapshot = world.get_snapshot()

        if snapshot.frame != image.frame:
            print(f"[⚠️警告] Snapshot 與影像 frame 不一致: snapshot={snapshot.frame}, image={image.frame}")

        # 📌 獲取當前車輛位置
        ego_vehicle = None
        all_vehicles = world.get_actors().filter('vehicle.*') # 獲取所有車輛

        for vehicle in all_vehicles:
            if vehicle.attributes.get('role_name') == 'hero':  # 預設玩家車輛名稱
                ego_vehicle = vehicle
                break
        # ego_vehicle = self._parent
        vehicle_transform = ego_vehicle.get_transform()  # 位置與旋轉
        vehicle_location = vehicle_transform.location  # 取得座標
        vehicle_rotation = vehicle_transform.rotation # 取得旋轉
        vehicle_velocity = ego_vehicle.get_velocity()  # 取得速度向量
        speed = (3.6 * (vehicle_velocity.x**2 + vehicle_velocity.y**2 + vehicle_velocity.z**2) ** 0.5)  # 轉換為 km/h

        # 🚗 取得 GNSS 資訊（如果有 GNSS Sensor）
        if self.gnss_sensor:
            # gnss_data = self.gnss_sensor.get_gnss_data()
            gnss_location = {"latitude": self.gnss_sensor.lat, "longitude": self.gnss_sensor.lon} # json
        else:
            gnss_location = None 

        # 📌 記錄附近車輛資訊
        ego_location = ego_vehicle.get_location()
        ego_forward_vector = ego_vehicle.get_transform().get_forward_vector()  # 自車前向量
        nearby_vehicles = []  # 存儲附近車輛資料
        bounding_boxes = []
        world_2_camera = np.array(self._camera_transforms[self.index][0].get_inverse_matrix())
        all_vehicles_start_time = time.time()  # 開始計時
        for vehicle in all_vehicles:
            if vehicle.id == ego_vehicle.id:
                continue  # 跳過自己
            distance = ego_location.distance(vehicle.get_location())
            vehicle_location = vehicle.get_location()
            if distance < 100.0:  # 只記錄 100 公尺內的車輛
                angle_deg_start_time = time.time()  # 開始計時
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
                
                # 結束計時
                end_time = time.time()
                duration = end_time - angle_deg_start_time
                print(f" 角度計算 {duration:.5f} 秒/車")

                # print(f"id:{vehicle.id}, dot={round(dot_product,6)}, degree={round(angle_deg,4)}")
                if angle_deg < 40:
                    ############## 3D BBOX #################
                    _3dbbox_start_time = time.time()  # 開始計時
                    bbox = self._create_bb_points(vehicle)
                    cords_x_y_z = self._vehicle_to_sensor(self, bbox, vehicle, self.sensor)[:3, :]
                    cords_y_minus_z_x = np.concatenate([cords_x_y_z[1, :], -cords_x_y_z[2, :], cords_x_y_z[0, :]])
                    bbox = np.transpose(np.dot(self.sensor.calibration, cords_y_minus_z_x))
                    camera_bbox = np.concatenate([bbox[:, 0] / bbox[:, 2], bbox[:, 1] / bbox[:, 2], bbox[:, 2]], axis=1)
                    bounding_boxes.append(camera_bbox)
                    # 結束計時
                    end_time = time.time()
                    duration = end_time - _3dbbox_start_time
                    print(f" 3D BBOX {duration:.5f} 秒/車")
                    
                    ############## 2D BBOX #################
                    _2dbbox_start_time = time.time()  # 開始計時
                    bb = vehicle.bounding_box
                    
                    # forward_vec = vehicle.get_transform().get_forward_vector()
                    # ray = vehicle.get_transform().location - vehicle.get_transform().location

                    # if forward_vec.dot(ray) > 0:
                    if angle_deg < 40:
                        x_max = -10000
                        x_min = 10000
                        y_max = -10000
                        y_min = 10000
                        p1 = self.get_image_point(bb.location, self.K, world_2_camera)
                        verts = [v for v in bb.get_world_vertices(vehicle.get_transform())]
                        # print(f"verts:{verts}")
                        for v_idx, vert in enumerate(verts):
                            p = self.get_image_point(vert, self.K, world_2_camera)
                            # print(f"vert[{v_idx}]:{p}")
                            # Find the rightmost vertex
                            if p[0] > x_max:
                                x_max = p[0]
                            # Find the leftmost vertex
                            if p[0] < x_min:
                                x_min = p[0]
                            # Find the highest vertex
                            if p[1] > y_max:
                                y_max = p[1]
                            # Find the lowest  vertex
                            if p[1] < y_min:
                                y_min = p[1]
                        # cv2.line(img, (int(x_min),int(y_min)), (int(x_max),int(y_min)), (0,0,255, 255), 1)
                        # cv2.line(img, (int(x_min),int(y_max)), (int(x_max),int(y_max)), (0,0,255, 255), 1)
                        # cv2.line(img, (int(x_min),int(y_min)), (int(x_min),int(y_max)), (0,0,255, 255), 1)
                        # cv2.line(img, (int(x_max),int(y_min)), (int(x_max),int(y_max)), (0,0,255, 255), 1)
                        
                        # print(f"2Dbbox: {(x_min, y_min, x_max, y_max)}")
                        # print("="*40)
                            
                    # 結束計時
                    end_time = time.time()
                    duration = end_time - _2dbbox_start_time
                    print(f" 2D BBOX {duration:.5f} 秒/車")
                    ############################################
                    ############## kitti label #################
                    kitti_start_time = time.time()  # 開始計時
                    label = self.get_simple_kitti_label(self, vehicle, self.sensor)
                    end_time = time.time()
                    duration = end_time - kitti_start_time
                    print(f" kitti label {duration:.5f} 秒/車")
                    ############################################

                    nearby_vehicles.append({
                        "id": vehicle.id,
                        "location": {"x": vehicle_location.x, "y": vehicle_location.y, "z": vehicle_location.z},
                        "distance_m": round(distance, 6),
                        "angle": round(angle_deg, 4), 
                        "3Dbbox": camera_bbox.tolist(),
                        "2Dbbox": (x_min, y_min, x_max, y_max),
                        "label": label
                    })
        # 結束計時
        end_time = time.time()
        duration = end_time - all_vehicles_start_time
        print(f" 計算所有車輛共花費 {duration:.5f} 秒")
        # self.draw_bounding_boxes(bounding_boxes)    
        # 🚗 設定影像名稱
        global save_path
        image_filename = f"{image.frame:08d}.png"
        json_file = save_path + "/vehicle_data.json"

        # 🚗 建立 JSON 資料結構
        data = {
            "frame": image.frame,
            "image_file": image_filename,
            # "speed_kmh": round(speed, 2),
            "location": {"x": vehicle_location.x, "y": vehicle_location.y, "z": vehicle_location.z},
            "rotation": {"pitch": vehicle_rotation.pitch, "yaw": vehicle_rotation.yaw, "roll": vehicle_rotation.roll},
            "gnss": gnss_location,
            "nearby_vehicles": nearby_vehicles
        }

        image.save_to_disk(save_path+f"/{image.frame:08d}.png") # 將影像保存到磁碟中
        # img.save_to_disk(save_path+f"/{image.frame:08d}.png")
        print(f"image saved at: {save_path+f'/{image.frame:08d}.png'}",end="")
        # 結束計時
        end_time = time.time()
        duration = end_time - start_time
        print(f" 資料處理 {duration:.3f} 秒")
        
        with self.write_lock: # 使用鎖，避免多執行緒同時讀寫
            # 🚗 讀取舊的 JSON 資料（如果檔案存在）
            if os.path.exists(json_file):
                with open(json_file, "r") as f:
                    try:
                        all_data = json.load(f)
                    except json.JSONDecodeError:
                        all_data = []
            else:
                all_data = []

            # 🚗 新增當前幀的數據
            all_data.append(data)

            # 🚗 儲存到 JSON 檔案
            # json_file = save_path + f"/{image.frame:08d}.json"
            with open(json_file, "w") as f:
                json.dump(all_data, f, indent=4)
                print(f"(json done: {image.frame:08d})",end="")
            # 結束計時
            end_time = time.time()
            duration = end_time - start_time
            print(f" 一張影像共費時 {duration:.3f} 秒")

            

    @staticmethod
    def get_image_point(loc, K, w2c):
        # Calculate 2D projection of 3D coordinate

        # Format the input coordinate (loc is a carla.Position object)
        point = np.array([loc.x, loc.y, loc.z, 1])
        # transform to camera coordinates
        point_camera = np.dot(w2c, point)

        # New we must change from UE4's coordinate system to an "standard"
        # (x, y ,z) -> (y, -z, x)
        # and we remove the fourth componebonent also
        point_camera = [point_camera[1], -point_camera[2], point_camera[0]]

        # now project 3D->2D using the camera matrix
        point_img = np.dot(K, point_camera)
        # normalize
        point_img[0] /= point_img[2]
        point_img[1] /= point_img[2]

        return point_img[0:2]

    def world_to_pixel(self, location):
        """
        世界座標轉成像素座標
        """
        # transform = self.camera_sensor.get_transform()
        transform, tt1 = self._camera_transforms[1] # 往前看的車內攝像頭
        world_2_camera = np.linalg.inv(self.carla_transform_to_matrix(transform))

        point = np.array([location.x, location.y, location.z, 1])
        camera_coords = world_2_camera.dot(point)

        if camera_coords[2] <= 0:
            return None

        pixel_coords = self.K.dot(camera_coords[:3])
        pixel_coords /= pixel_coords[2]

        x = int(pixel_coords[0])
        y = int(pixel_coords[1])
        print(f"像素座標: ({x},{y})")
        if 0 <= x < self.hud.dim[0] and 0 <= y < self.hud.dim[1]:
            return (x, y)
        else:
            print("座標不合理")
            return None

    def carla_transform_to_matrix(self, transform):
        """
        將CARLA的Transform轉成4x4變換矩陣
        """
        # print(f"====== transform =======")
        # print(f"{transform}")
        # rotation = transform.rotation
        # location = transform.location

        # c_y = np.cos(np.radians(rotation.yaw))
        # s_y = np.sin(np.radians(rotation.yaw))
        # c_p = np.cos(np.radians(rotation.pitch))
        # s_p = np.sin(np.radians(rotation.pitch))
        # c_r = np.cos(np.radians(rotation.roll))
        # s_r = np.sin(np.radians(rotation.roll))

        # matrix = np.identity(4)
        # matrix[0, 0] = c_p * c_y
        # matrix[0, 1] = c_y * s_p * s_r - s_y * c_r
        # matrix[0, 2] = -c_y * s_p * c_r - s_y * s_r
        # matrix[0, 3] = location.x
        # matrix[1, 0] = c_p * s_y
        # matrix[1, 1] = s_y * s_p * s_r + c_y * c_r
        # matrix[1, 2] = -s_y * s_p * c_r + c_y * s_r
        # matrix[1, 3] = location.y
        # matrix[2, 0] = s_p
        # matrix[2, 1] = -c_p * s_r
        # matrix[2, 2] = c_p * c_r
        # matrix[2, 3] = location.z

        rotation = transform.rotation
        location = transform.location
        c_y = np.cos(np.radians(rotation.yaw))
        s_y = np.sin(np.radians(rotation.yaw))
        c_r = np.cos(np.radians(rotation.roll))
        s_r = np.sin(np.radians(rotation.roll))
        c_p = np.cos(np.radians(rotation.pitch))
        s_p = np.sin(np.radians(rotation.pitch))
        matrix = np.matrix(np.identity(4))
        matrix[0, 3] = location.x
        matrix[1, 3] = location.y
        matrix[2, 3] = location.z
        matrix[0, 0] = c_p * c_y
        matrix[0, 1] = c_y * s_p * s_r - s_y * c_r
        matrix[0, 2] = -c_y * s_p * c_r - s_y * s_r
        matrix[1, 0] = s_y * c_p
        matrix[1, 1] = s_y * s_p * s_r + c_y * c_r
        matrix[1, 2] = -s_y * s_p * c_r + c_y * s_r
        matrix[2, 0] = s_p
        matrix[2, 1] = -c_p * s_r
        matrix[2, 2] = c_p * c_r
        return matrix

    def draw_bounding_boxes(self, bounding_boxes):
        """
        Draws bounding boxes on pygame display.
        """

        bb_surface = self.surface # pygame.Surface((VIEW_WIDTH, VIEW_HEIGHT))
        bb_surface.set_colorkey((0, 0, 0))
        for bbox in bounding_boxes:
            points = [(int(bbox[i, 0]), int(bbox[i, 1])) for i in range(8)]
            # draw lines
            # base
            pygame.draw.line(bb_surface, BB_COLOR, points[0], points[1])
            pygame.draw.line(bb_surface, BB_COLOR, points[0], points[1])
            pygame.draw.line(bb_surface, BB_COLOR, points[1], points[2])
            pygame.draw.line(bb_surface, BB_COLOR, points[2], points[3])
            pygame.draw.line(bb_surface, BB_COLOR, points[3], points[0])
            # top
            pygame.draw.line(bb_surface, BB_COLOR, points[4], points[5])
            pygame.draw.line(bb_surface, BB_COLOR, points[5], points[6])
            pygame.draw.line(bb_surface, BB_COLOR, points[6], points[7])
            pygame.draw.line(bb_surface, BB_COLOR, points[7], points[4])
            # base-top
            pygame.draw.line(bb_surface, BB_COLOR, points[0], points[4])
            pygame.draw.line(bb_surface, BB_COLOR, points[1], points[5])
            pygame.draw.line(bb_surface, BB_COLOR, points[2], points[6])
            pygame.draw.line(bb_surface, BB_COLOR, points[3], points[7])
        # display.blit(bb_surface, (0, 0))

    @staticmethod
    def get_bounding_box(self, vehicle, camera):
        """
        Returns 3D bounding box for a vehicle based on camera view.
        """

        bb_cords = self._create_bb_points(vehicle)
        cords_x_y_z = self._vehicle_to_sensor(self, bb_cords, vehicle, camera)[:3, :]
        cords_y_minus_z_x = np.concatenate([cords_x_y_z[1, :], -cords_x_y_z[2, :], cords_x_y_z[0, :]])
        bbox = np.transpose(np.dot(camera.calibration, cords_y_minus_z_x))
        camera_bbox = np.concatenate([bbox[:, 0] / bbox[:, 2], bbox[:, 1] / bbox[:, 2], bbox[:, 2]], axis=1)
        return camera_bbox

    @staticmethod
    def _create_bb_points(vehicle):
        """
        Returns 3D bounding box for a vehicle.
        """

        cords = np.zeros((8, 4))
        extent = vehicle.bounding_box.extent
        cords[0, :] = np.array([extent.x, extent.y, -extent.z, 1])
        cords[1, :] = np.array([-extent.x, extent.y, -extent.z, 1])
        cords[2, :] = np.array([-extent.x, -extent.y, -extent.z, 1])
        cords[3, :] = np.array([extent.x, -extent.y, -extent.z, 1])
        cords[4, :] = np.array([extent.x, extent.y, extent.z, 1])
        cords[5, :] = np.array([-extent.x, extent.y, extent.z, 1])
        cords[6, :] = np.array([-extent.x, -extent.y, extent.z, 1])
        cords[7, :] = np.array([extent.x, -extent.y, extent.z, 1])
        return cords

    @staticmethod
    def _vehicle_to_sensor(self, cords, vehicle, sensor):
        """
        Transforms coordinates of a vehicle bounding box to sensor.
        """

        world_cord = self._vehicle_to_world(self, cords, vehicle)
        sensor_cord = self._world_to_sensor(self, world_cord, sensor)
        return sensor_cord

    @staticmethod
    def _vehicle_to_world(self, cords, vehicle):
        """
        Transforms coordinates of a vehicle bounding box to world.
        """

        bb_transform = carla.Transform(vehicle.bounding_box.location)
        bb_vehicle_matrix = self.get_matrix(bb_transform)
        vehicle_world_matrix = self.get_matrix(vehicle.get_transform())
        bb_world_matrix = np.dot(vehicle_world_matrix, bb_vehicle_matrix)
        world_cords = np.dot(bb_world_matrix, np.transpose(cords))
        return world_cords

    @staticmethod
    def _world_to_sensor(self, cords, sensor):
        """
        Transforms world coordinates to sensor.
        """

        sensor_world_matrix = self.get_matrix(sensor.get_transform())
        world_sensor_matrix = np.linalg.inv(sensor_world_matrix)
        sensor_cords = np.dot(world_sensor_matrix, cords)
        return sensor_cords

    @staticmethod
    def get_matrix(transform):
        """
        Creates matrix from carla transform.
        """

        rotation = transform.rotation
        location = transform.location
        c_y = np.cos(np.radians(rotation.yaw))
        s_y = np.sin(np.radians(rotation.yaw))
        c_r = np.cos(np.radians(rotation.roll))
        s_r = np.sin(np.radians(rotation.roll))
        c_p = np.cos(np.radians(rotation.pitch))
        s_p = np.sin(np.radians(rotation.pitch))
        matrix = np.matrix(np.identity(4))
        matrix[0, 3] = location.x
        matrix[1, 3] = location.y
        matrix[2, 3] = location.z
        matrix[0, 0] = c_p * c_y
        matrix[0, 1] = c_y * s_p * s_r - s_y * c_r
        matrix[0, 2] = -c_y * s_p * c_r - s_y * s_r
        matrix[1, 0] = s_y * c_p
        matrix[1, 1] = s_y * s_p * s_r + c_y * c_r
        matrix[1, 2] = -s_y * s_p * c_r + c_y * s_r
        matrix[2, 0] = s_p
        matrix[2, 1] = -c_p * s_r
        matrix[2, 2] = c_p * c_r
        return matrix


    def get_camera_intrinsic(self, width, height, fov=90.0):
        """
        生成相機內部參數矩陣
        """
        f_x = width / (2.0 * np.tan(fov * np.pi / 360.0))
        f_y = f_x  # 假設 pixel aspect ratio 為1
        c_x = width / 2.0
        c_y = height / 2.0

        K = np.array([
            [f_x,   0, c_x],
            [0,   f_y, c_y],
            [0,     0,   1]
        ])
        return K

    def get_attribute_value(self, attr):
        if str(attr.type) == "Int":
            # print(f"{attr.type} = 'int'")
            return attr.as_int()
        elif str(attr.type) == "Float":
            # print(f"{attr.type} = 'float'")
            return attr.as_float()
        elif str(attr.type) == "Bool":
            # print(f"{attr.type} = 'bool'")
            return attr.as_bool()
        else:
            # print(f"{attr.type} = 'string'")
            return str(attr)

    def get_camera_params(self, cam_para_path="camera_params.txt", kitti_path="calib.txt"):
        # global save_path
        save_dir = '/mnt/data1/rvl/UnrealEngine_4.26/Engine/Binaries/Linux/carla/PythonAPI/examples/Recording/'
        save_path = save_dir + cam_para_path
        kitti_path = save_dir + kitti_path
        # 取得目前 transform 和 blueprint
        transform, attachment = self._camera_transforms[self.transform_index]
        bp = self.sensors[self.index][-1] if self.index is not None else None
        print(f"BP: {bp}")

        # 儲存文字
        lines = []

        # 外部參數（Extrinsic）
        lines.append("=== 相機外部參數（Extrinsic） ===")
        lines.append(f"Location: x={transform.location.x:.4f}, y={transform.location.y:.4f}, z={transform.location.z:.4f}")
        lines.append(f"Rotation: pitch={transform.rotation.pitch:.4f}, yaw={transform.rotation.yaw:.4f}, roll={transform.rotation.roll:.4f}")

        # 內部參數（Intrinsic）
        lines.append("\n=== 相機內部參數（Intrinsic / Blueprint 設定） ===")
        if bp is not None:
            for attr in bp:
                # print(f"attr: {attr}")
                value = self.get_attribute_value(attr)
                lines.append(f"{attr.id}: {value}")
        else:
            lines.append("（尚未初始化 blueprint，請先啟用相機）")

        # 存為一般 txt 檔
        with open(save_path, "w", encoding="utf-8") as f:
            for line in lines:
                f.write(line + "\n")
        print(f"📄 相機參數已儲存到 {save_path}")

        # 🔽 儲存為 KITTI 格式的 calib.txt
        if bp is not None:
            image_w = float(self.get_attribute_value(bp.get_attribute("image_size_x")))
            image_h = float(self.get_attribute_value(bp.get_attribute("image_size_y")))
            fov = float(self.get_attribute_value(bp.get_attribute('fov')))
            focal_length = image_w / (2.0 * math.tan(fov * math.pi / 360.0))
            cx = image_w / 2.0
            cy = image_h / 2.0

            # 主相機的投影矩陣
            P = np.array([
                [focal_length, 0,  cx, 0],
                [0,  focal_length, cy, 0],
                [0,  0,  1,  0]
            ]).flatten()

            # 寫入完整 calib.txt
            with open(kitti_path, "w") as f:
                for i in range(4):  # P0 ~ P3
                    f.write(f"P{i}: " + " ".join([f"{v:.4f}" for v in P]) + "\n")
                # 其他必須欄位（填入單位矩陣或 0）
                f.write("Tr_velo_to_cam: " + " ".join(["0.0000"] * 12) + "\n")
                f.write("R0_rect: " + " ".join(["1.0000" if i % 4 == 0 else "0.0000" for i in range(9)]) + "\n")

            print(f"📄 KITTI 格式參數已完整儲存到 {kitti_path}")

    @staticmethod
    def get_bottom_center_2d(self, vehicle, camera):
        """
        回傳 3D bounding box 底部中心點在影像中的投影座標 (x, y)
        """
        # 建立 3D bounding box 的 8 個角點
        bb_cords = self._create_bb_points(vehicle)

        # 計算底部四個點的平均，即底部中心點 (車體座標)
        bottom_center = np.mean(bb_cords[0:4, :], axis=0).reshape((4, 1))  # shape = (4, 1)

        # 轉換到底座點的感測器座標 (3x1)
        cords = self._vehicle_to_sensor(bottom_center.T, vehicle, camera)[:3, :]

        # 換軸順序符合內參矩陣：Y, -Z, X（shape = (3, 1)）
        cords_yzx = np.array([
            cords[1][0],         # y
            -cords[2][0],        # -z
            cords[0][0]          # x
        ]).reshape(3, 1)         # shape = (3,1)

        # 使用相機內參做投影 (3x3) x (3x1) = (3x1)
        projected = np.dot(camera.calibration, cords_yzx)

        # 正規化為像素座標
        x = int(projected[0][0] / projected[2][0])
        y = int(projected[1][0] / projected[2][0])

        return [x, y]

    @staticmethod
    def get_simple_kitti_label(self, vehicle, camera):
        """
        生成一個簡化的KITTI格式標籤，直接使用CARLA的函數

        Args:
            vehicle: CARLA車輛對象
            camera: CARLA相機對象

        Returns:
            kitti_label: KITTI格式的標籤字符串
        """
        # 獲取車輛的邊界框和變換
        bb = vehicle.bounding_box
        vehicle_transform = vehicle.get_transform()
        camera_transform = camera.get_transform()

        # 獲取車輛的類別
        vehicle_type = vehicle.type_id.split('.')[-2]
        obj_type = 'Car'
        if 'bicycle' in vehicle_type:
            obj_type = 'Cyclist'
        elif 'motorcycle' in vehicle_type:
            obj_type = 'Cyclist'
        elif 'pedestrian' in vehicle_type:
            obj_type = 'Pedestrian'        
        kitti_label = f"{obj_type} "

        # 遮擋
        truncated = 0
        occluded = 0
        alpha = 0
        kitti_label += f"{truncated} {occluded} {alpha} "

        # 2d bounding box 左上 ＆ 右下
        kitti_label += f"{0} {0} {0} {0} "
        
        # 車輛尺寸 單位公尺
        height = bb.extent.z * 2  # 高度
        width = bb.extent.y * 2   # 寬度
        length = bb.extent.x * 2  # 長度
        kitti_label += f"{height:.2f} {width:.2f} {length:.2f} "

        # 3d bbox 的部份改為八個點位置，以像素為單位
        """ 
        bbox_3d = ClientSideBoundingBoxes.get_bounding_box(vehicle, camera)
        points = [(int(bbox_3d[i, 0]), int(bbox_3d[i, 1])) for i in range(8)]

        # 添加3D邊界框的8個投影點
        for j in range(8):
            kitti_label += f"{points[j]}"
        kitti_label += " "        
        """ 
        
        # 獲取車輛中心點在車輛坐標系中的位置（通常是(0,0,0)）
        vehicle_center = np.array([0, 0, 0, 1]).reshape(4, 1)
        # 將車輛中心從車輛坐標系轉到相機坐標系        
        # ---車輛到世界坐標系的變換矩陣
        vehicle_to_world = self.get_matrix(vehicle_transform)
        vehicle_center_world = np.dot(vehicle_to_world, vehicle_center)
        # ---世界到相機的變換矩陣
        world_to_camera = np.linalg.inv(self.get_matrix(camera_transform))                
        vehicle_center_camera = np.dot(world_to_camera, vehicle_center_world)

        # 提取x, y, z（注意：CARLA和KITTI的座標系有所不同，需要轉換），單位為 meter
        # CARLA: X(前), Y(右), Z(上)
        # KITTI相機坐標系: X(右), Y(下), Z(前)
        tx = vehicle_center_camera[1][0]  # CARLA Y -> KITTI X
        ty = -vehicle_center_camera[2][0]  # 負的CARLA Z -> KITTI Y
        tz = vehicle_center_camera[0][0]  # CARLA X -> KITTI Z
        
        # 檢查物體是否在相機前方
        if tz <= 0:
            print(f"[{vehicle.id}] 物體在相機後方: tz={tz}")
            return None        
        # 添加相機座標
        kitti_label += f"{tx:.5f} {ty:.5f} {tz:.5f} "

        ''' 05/31
        # 計算車輛在相機坐標系中的 rotation_y
        # KITTI rotation_y 定義：物體相對於相機坐標系 Y 軸的旋轉角度
        # 範圍：[-π, π]，正值表示逆時針旋轉

        # 方法1：使用車輛的旋轉矩陣計算
        vehicle_rotation = vehicle_transform.rotation
        vehicle_yaw_world = np.radians(vehicle_rotation.yaw)  # 轉換為弧度

        # 獲取相機的旋轉
        camera_rotation = camera_transform.rotation
        camera_yaw_world = np.radians(camera_rotation.yaw)

        # 計算車輛相對於相機的 yaw 角度（在世界坐標系中）
        relative_yaw = vehicle_yaw_world - camera_yaw_world

        # 將角度標準化到 [-π, π] 範圍
        relative_yaw = ((relative_yaw + np.pi) % (2 * np.pi)) - np.pi

        # 由於 CARLA 和 KITTI 坐標系的差異，需要進行轉換
        # CARLA: yaw 是繞 Z 軸的旋轉
        # KITTI: rotation_y 是繞 Y 軸的旋轉，且坐標系已轉換
        # 考慮坐標系轉換：CARLA Z軸朝上，KITTI Y軸朝下
        rotation_y = -relative_yaw + np.pi/2

        # 標準化到 [-π, π] 範圍
        rotation_y = ((rotation_y + np.pi) % (2 * np.pi)) - np.pi
        kitti_label += f"{rotation_y:.5f} "
        '''
        # 方法2：更精確的向量計算方法（推薦使用）
        # 獲取車輛前向向量並轉換到相機坐標系
        vehicle_forward = vehicle_transform.get_forward_vector()
        vehicle_forward_world = np.array([vehicle_forward.x, vehicle_forward.y, vehicle_forward.z, 0])

        # 轉換到相機坐標系
        vehicle_forward_camera = np.dot(world_to_camera, vehicle_forward_world)

        # 應用 CARLA 到 KITTI 的坐標系轉換
        # CARLA: X(前), Y(右), Z(上) -> KITTI: X(右), Y(下), Z(前)
        forward_x_kitti = vehicle_forward_camera[1]  # CARLA Y -> KITTI X
        forward_z_kitti = vehicle_forward_camera[0]  # CARLA X -> KITTI Z

        # 修正：KITTI rotation_y 的定義
        # KITTI 中 rotation_y = 0 表示物體朝向 +Z 方向（相機前方）
        # 但車輛的"前方"在 KITTI 中通常是車頭朝向，需要修正 90 度偏差
        # 這是因為 CARLA 車輛模型和 KITTI 標註約定的差異

        # 計算在 KITTI 相機坐標系中的 rotation_y
        if abs(forward_z_kitti) < 1e-6:
            # 當 Z 分量接近 0 時，物體朝向與相機光軸垂直
            rotation_y = np.pi/2 if forward_x_kitti > 0 else -np.pi/2
        else:
            # 使用 atan2 計算角度
            rotation_y = np.arctan2(forward_x_kitti, forward_z_kitti)

        # 修正 90 度偏差 - 這是關鍵修正
        # 減去 π/2 來校正 CARLA 車輛模型與 KITTI 標註約定的差異
        rotation_y = rotation_y - np.pi/2

        # 標準化角度到 [-π, π] 範圍
        rotation_y = ((rotation_y + np.pi) % (2 * np.pi)) - np.pi
        kitti_label += f"{rotation_y:.5f} "

        # 中心點位置 image pixel
        center_2d = self.get_bottom_center_2d(self, vehicle, camera)
        print("影像中底部中心點位置:", center_2d)
        #kitti_label += f"{center_2d[0]} {center_2d[1]}"
        
        """ 
        # 輸出相機矩陣
        camera_bp = self.world.get_blueprint_library().find('sensor.camera.rgb')
        camera_transform = carla.Transform(carla.Location(x=1.5, z=2.4))
        image_width = 960
        image_height = 540 
        P2 = BasicSynchronousClient.get_kitti_p2_matrix(camera_bp, camera_transform, image_width, image_height)
        # print(P2)
        """ 

        # txt 輸出： 類別、長、寬、高、x、y、z、旋轉角度、3D_bbox 底部中心點座標
        return kitti_label



# ==============================================================================
# -- game_loop() ---------------------------------------------------------------
# ==============================================================================


def game_loop(args):
    pygame.init()
    pygame.font.init()
    world = None
    original_settings = None

    try:
        client = carla.Client(args.host, args.port)
        client.set_timeout(2000.0)

        sim_world = client.get_world()
        if args.sync:
            original_settings = sim_world.get_settings()
            settings = sim_world.get_settings()
            if not settings.synchronous_mode:
                settings.synchronous_mode = True
                settings.fixed_delta_seconds = 0.05
            sim_world.apply_settings(settings)

            traffic_manager = client.get_trafficmanager()
            traffic_manager.set_synchronous_mode(True)

        if args.autopilot and not sim_world.get_settings().synchronous_mode:
            print("WARNING: You are currently in asynchronous mode and could "
                  "experience some issues with the traffic simulation")

        display = pygame.display.set_mode(
            (args.width, args.height),
            pygame.HWSURFACE | pygame.DOUBLEBUF)
        display.fill((0,0,0))
        pygame.display.flip()

        hud = HUD(args.width, args.height)
        world = World(sim_world, hud, args)
        controller = KeyboardControl(world, args.autopilot)

        if args.sync:
            sim_world.tick()
        else:
            sim_world.wait_for_tick()

        clock = pygame.time.Clock()
        while True:
            if args.sync:
                sim_world.tick()
            clock.tick_busy_loop(60)
            if controller.parse_events(client, world, clock, args.sync):
                return
            world.tick(clock)
            world.render(display)
            pygame.display.flip()

    finally:

        if original_settings:
            sim_world.apply_settings(original_settings)

        if (world and world.recording_enabled):
            client.stop_recorder()

        if world is not None:
            world.destroy()

        pygame.quit()


# ==============================================================================
# -- main() --------------------------------------------------------------------
# ==============================================================================


def main():
    argparser = argparse.ArgumentParser(
        description='CARLA Manual Control Client')
    argparser.add_argument(
        '-v', '--verbose',
        action='store_true',
        dest='debug',
        help='print debug information')
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
        '-a', '--autopilot',
        action='store_true',
        help='enable autopilot')
    argparser.add_argument(
        '--res',
        metavar='WIDTHxHEIGHT',
        default='1280x720',
        help='window resolution (default: 1280x720)')
    argparser.add_argument(
        '--filter',
        metavar='PATTERN',
        default='vehicle.*',
        help='actor filter (default: "vehicle.*")')
    argparser.add_argument(
        '--generation',
        metavar='G',
        default='2',
        help='restrict to certain actor generation (values: "1","2","All" - default: "2")')
    argparser.add_argument(
        '--rolename',
        metavar='NAME',
        default='hero',
        help='actor role name (default: "hero")')
    argparser.add_argument(
        '--gamma',
        default=2.2,
        type=float,
        help='Gamma correction of the camera (default: 2.2)')
    argparser.add_argument(
        '--sync',
        action='store_true',
        help='Activate synchronous mode execution')
    args = argparser.parse_args()

    args.width, args.height = [int(x) for x in args.res.split('x')]

    log_level = logging.DEBUG if args.debug else logging.INFO
    logging.basicConfig(format='%(levelname)s: %(message)s', level=log_level)

    logging.info('listening to server %s:%s', args.host, args.port)

    print(__doc__)

    try:

        game_loop(args)

    except KeyboardInterrupt:
        print('\nCancelled by user. Bye!')


if __name__ == '__main__':

    main()
