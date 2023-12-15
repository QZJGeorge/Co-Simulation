#!/usr/bin/env python

# Copyright (c) 2020 Computer Vision Center (CVC) at the Universitat Autonoma de
# Barcelona (UAB).
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.
"""
Script to integrate CARLA and SUMO simulations
"""

# ==================================================================================================
# -- imports ---------------------------------------------------------------------------------------
# ==================================================================================================

import argparse
import logging
import time

import redis
import json

# ==================================================================================================
# -- find carla module -----------------------------------------------------------------------------
# ==================================================================================================

import glob
import os
import sys

import carla

try:
    sys.path.append(
        glob.glob('../../PythonAPI/carla/dist/carla-*%d.%d-%s.egg' %
                  (sys.version_info.major, sys.version_info.minor,
                   'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

# ==================================================================================================
# -- find traci module -----------------------------------------------------------------------------
# ==================================================================================================

# if 'SUMO_HOME' in os.environ:
#     sys.path.append(os.path.join(os.environ['SUMO_HOME'], 'tools'))
# else:
#     sys.exit("please declare environment variable 'SUMO_HOME'")

# ==================================================================================================
# -- sumo integration imports ----------------------------------------------------------------------
# ==================================================================================================

from sumo_integration.bridge_helper import BridgeHelper  # pylint: disable=wrong-import-position
from sumo_integration.carla_simulation import CarlaSimulation  # pylint: disable=wrong-import-position
from sumo_integration.constants import INVALID_ACTOR_ID  # pylint: disable=wrong-import-position

# ==================================================================================================
# -- synchronization_loop --------------------------------------------------------------------------
# ==================================================================================================


class SimulationSynchronization(object):
    """
    SimulationSynchronization class is responsible for the synchronization of sumo and carla
    simulations.
    """
    def __init__(self,
                 carla_simulation,
                 tls_manager='none',
                 sync_vehicle_color=False,
                 sync_vehicle_lights=False):

        self.carla = carla_simulation

        self.tls_manager = tls_manager
        self.sync_vehicle_color = sync_vehicle_color
        self.sync_vehicle_lights = sync_vehicle_lights

        self.redis = redis.Redis(host='localhost', port=6379, db=0)

        # if tls_manager == 'carla':
        #     self.sumo.switch_off_traffic_lights()
        # elif tls_manager == 'sumo':
        #     self.carla.switch_off_traffic_lights()

        # Mapped actor ids.
        self.sumo2carla_ids = {}  # Contains only actors controlled by sumo.

        BridgeHelper.blueprint_library = self.carla.world.get_blueprint_library()
        # BridgeHelper.offset = self.sumo.get_net_offset()
        BridgeHelper.offset = [102.89, 281.25]

        # Configuring carla simulation in sync mode.
        settings = self.carla.world.get_settings()
        settings.synchronous_mode = True
        settings.fixed_delta_seconds = self.carla.step_length
        self.carla.world.apply_settings(settings)

        traffic_manager = self.carla.client.get_trafficmanager()
        traffic_manager.set_synchronous_mode(True)

    def tick(self):
        """
        Tick to simulation synchronization
        """

        invalid_sumo_ids = []

        # reads sumo context from redis.
        sumo_context_json = self.redis.get('sumo_context')
        if sumo_context_json is not None:
            sumo_context_dict = json.loads(sumo_context_json)
        else:
            # Destroying synchronized actors.
            for carla_actor_id in self.sumo2carla_ids.values():
                self.carla.destroy_actor(carla_actor_id)
            print("No data found for sumo_context, destroying all actors.")
            time.sleep(2)
            return

        # iterates over sumo actors and updates them in carla.
        for sumo_actor_id, sumo_actor_value in sumo_context_dict.items():
            sumo_actor_type_id = sumo_actor_value['type_id']
            sumo_actor_vclass_value = sumo_actor_value['vclass']
            sumo_actor_lights = sumo_actor_value['lights']

            sumo_actor_color_list = sumo_actor_value['color']
            sumo_actor_color_tuple = tuple(sumo_actor_color_list)

            sumo_actor_transform = carla.Transform()
            sumo_actor_transform.location.x = sumo_actor_value['location']['x']
            sumo_actor_transform.location.y = sumo_actor_value['location']['y']
            sumo_actor_transform.location.z = sumo_actor_value['location']['z']
            sumo_actor_transform.rotation.pitch = sumo_actor_value['rotation']['pitch']
            sumo_actor_transform.rotation.yaw = sumo_actor_value['rotation']['yaw']
            sumo_actor_transform.rotation.roll = sumo_actor_value['rotation']['roll']

            sumo_actor_extent = carla.Vector3D()
            sumo_actor_extent.x = sumo_actor_value['extent']['x']
            sumo_actor_extent.y = sumo_actor_value['extent']['y']
            sumo_actor_extent.z = sumo_actor_value['extent']['z']

            carla_transform = BridgeHelper.get_carla_transform(sumo_actor_transform, sumo_actor_extent)
            
            carla_lights = None
            if self.sync_vehicle_lights:
                carla_lights = BridgeHelper.get_carla_lights_state(carla_actor.get_light_state(), sumo_actor_lights)
                
            # Creating carla actor if it does not exist.
            if sumo_actor_id not in self.sumo2carla_ids:
                carla_blueprint = BridgeHelper.get_carla_blueprint_from_sumo_redis(
                    sumo_actor_type_id, sumo_actor_color_tuple, sumo_actor_vclass_value)

                if carla_blueprint is not None:
                    carla_actor_id = self.carla.spawn_actor(carla_blueprint, carla_transform)
                    if carla_actor_id != INVALID_ACTOR_ID:
                        self.sumo2carla_ids[sumo_actor_id] = carla_actor_id
                    else:
                        invalid_sumo_ids.append(sumo_actor_id)

            # Updating carla actor if it exists.
            else:
                carla_actor_id = self.sumo2carla_ids[sumo_actor_id]
                carla_actor = self.carla.get_actor(carla_actor_id)
                self.carla.synchronize_vehicle(carla_actor_id, carla_transform, carla_lights)

            # Create a list to store items to be removed
            to_be_removed = []

            # Iterate over sumo2carla_ids dictionary
            for sumo_actor_id in self.sumo2carla_ids:
                if sumo_actor_id not in sumo_context_dict:
                    # Don't remove item from dictionary yet, just note it down
                    to_be_removed.append(sumo_actor_id)

                    print("Destroy actor: ", sumo_actor_id)

            # Now remove noted items from the dictionary
            for sumo_actor_id in to_be_removed:
                self.carla.destroy_actor(self.sumo2carla_ids.pop(sumo_actor_id))
                
        # Updates traffic lights in carla based on sumo information.
        # if self.tls_manager == 'sumo':
        #     common_landmarks = self.sumo.traffic_light_ids & self.carla.traffic_light_ids
        #     for landmark_id in common_landmarks:
        #         sumo_tl_state = self.sumo.get_traffic_light_state(landmark_id)
        #         carla_tl_state = BridgeHelper.get_carla_traffic_light_state(sumo_tl_state)
        #         self.carla.synchronize_traffic_light(landmark_id, carla_tl_state)

        self.redis.set('invalid_sumo_ids', json.dumps(invalid_sumo_ids))
        self.carla.tick()

    def close(self):
        """
        Cleans synchronization.
        """
        # Configuring carla simulation in async mode.
        settings = self.carla.world.get_settings()
        settings.synchronous_mode = False
        settings.fixed_delta_seconds = None
        self.carla.world.apply_settings(settings)

        # Destroying synchronized actors.
        for carla_actor_id in self.sumo2carla_ids.values():
            self.carla.destroy_actor(carla_actor_id)

        # Closing carla client.
        self.carla.close()


def synchronization_loop(args):
    """
    Entry point for sumo-carla co-simulation.
    """
    carla_simulation = CarlaSimulation(args.carla_host, args.carla_port, args.step_length)

    synchronization = SimulationSynchronization(carla_simulation, args.tls_manager,
                                                args.sync_vehicle_color, args.sync_vehicle_lights)
    try:
        while True:
            start = time.time()

            synchronization.tick()

            end = time.time()
            elapsed = end - start
            if elapsed < args.step_length:
                time.sleep(args.step_length - elapsed)

    except KeyboardInterrupt:
        logging.info('Cancelled by user.')

    finally:
        logging.info('Cleaning synchronization')
        synchronization.close()


if __name__ == '__main__':
    argparser = argparse.ArgumentParser(description=__doc__)
    argparser.add_argument('--sumo_cfg_file',
                           default = 'map/sumo_map/mcity.sumocfg', 
                           type=str, 
                           help='sumo configuration file')
    argparser.add_argument('--carla-host',
                           metavar='H',
                           default='127.0.0.1',
                           help='IP of the carla host server (default: 127.0.0.1)')
    argparser.add_argument('--carla-port',
                           metavar='P',
                           default=2000,
                           type=int,
                           help='TCP port to listen to (default: 2000)')
    argparser.add_argument('--sumo-host',
                           metavar='H',
                           default=None,
                           help='IP of the sumo host server (default: 127.0.0.1)')
    argparser.add_argument('--sumo-port',
                           metavar='P',
                           default=None,
                           type=int,
                           help='TCP port to listen to (default: 8813)')
    argparser.add_argument('--sumo-gui',
                           default=True,
                           type=bool,
                           help='run the gui version of sumo')
    argparser.add_argument('--step-length',
                           default=0.05,
                           type=float,
                           help='set fixed delta seconds (default: 0.05s)')
    argparser.add_argument('--client-order',
                           metavar='TRACI_CLIENT_ORDER',
                           default=1,
                           type=int,
                           help='client order number for the co-simulation TraCI connection (default: 1)')
    argparser.add_argument('--sync-vehicle-lights',
                           action='store_true',
                           help='synchronize vehicle lights state (default: False)')
    argparser.add_argument('--sync-vehicle-color',
                           action='store_true',
                           help='synchronize vehicle color (default: False)')
    argparser.add_argument('--sync-vehicle-all',
                           action='store_true',
                           help='synchronize all vehicle properties (default: False)')
    argparser.add_argument('--tls-manager',
                           type=str,
                           choices=['none', 'sumo', 'carla'],
                           help="select traffic light manager (default: none)",
                           default='none')
    argparser.add_argument('--debug', action='store_true', help='enable debug messages')
    arguments = argparser.parse_args()

    if arguments.sync_vehicle_all is True:
        arguments.sync_vehicle_lights = True
        arguments.sync_vehicle_color = True

    if arguments.debug:
        logging.basicConfig(format='%(levelname)s: %(message)s', level=logging.DEBUG)
    else:
        logging.basicConfig(format='%(levelname)s: %(message)s', level=logging.INFO)

    synchronization_loop(arguments)
