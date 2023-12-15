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

import carla

# ==================================================================================================
# -- find carla module -----------------------------------------------------------------------------
# ==================================================================================================

import glob
import os
import sys

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
from sumo_integration.sumo_simulation import SumoSimulation  # pylint: disable=wrong-import-position

# ==================================================================================================
# -- synchronization_loop --------------------------------------------------------------------------
# ==================================================================================================


class SimulationSynchronization(object):
    """
    SimulationSynchronization class is responsible for the synchronization of sumo and carla
    simulations.
    """
    def __init__(self,
                 sumo_simulation,
                 tls_manager='none',
                 sync_vehicle_color=False,
                 sync_vehicle_lights=False):

        self.sumo = sumo_simulation
        self.ego_id = None

        self.tls_manager = tls_manager
        self.sync_vehicle_color = sync_vehicle_color
        self.sync_vehicle_lights = sync_vehicle_lights

        self.redis = redis.Redis(host='localhost', port=6379, db=0)

        # if tls_manager == 'carla':
        #     self.sumo.switch_off_traffic_lights()
        # elif tls_manager == 'sumo':
        #     self.carla.switch_off_traffic_lights()

        # Mapped actor ids.
        self.sumo2carla_ids = set()

        BridgeHelper.offset = self.sumo.get_net_offset()

    def tick(self):
        """
        Tick to simulation synchronization
        """
        # -----------------
        # sumo-->carla sync
        # -----------------
        self.sumo.tick()

        # Spawning new sumo actors (i.e, not controlled by carla).
        sumo_spawned_actors = self.sumo.spawned_actors
        
        for sumo_actor_id in sumo_spawned_actors:
            # To avoid loop generation
            if 'carla' not in sumo_actor_id:
                self.sumo2carla_ids.add(sumo_actor_id)
                self.sumo.subscribe(sumo_actor_id)

        # Destroying sumo arrived actors in carla.
        for sumo_actor_id in self.sumo.destroyed_actors:
            if sumo_actor_id in self.sumo2carla_ids:
                self.sumo2carla_ids.remove(sumo_actor_id)

        # remove invalid sumo actors from carla
        invalid_sumo_ids = self.redis.get('invalid_sumo_ids')
        if invalid_sumo_ids is not None:
            invalid_sumo_ids = json.loads(invalid_sumo_ids)
            for sumo_actor_id in invalid_sumo_ids:
                if sumo_actor_id in self.sumo2carla_ids:
                    self.sumo2carla_ids.remove(sumo_actor_id)
                    self.sumo.unsubscribe(sumo_actor_id)
        
        # update ego vehicle
        ego_transform = self.redis.get('ego_transform')
        if ego_transform is not None:
            ego_transform = json.loads(ego_transform)

            if self.ego_id is None:
                red = '255, 0, 0, 255'
                self.ego_id = self.sumo.spawn_actor(type_id='IDM_waymo_motion', color=red)
            else:
                sumo_transform = carla.Transform()
                sumo_transform.location.x = ego_transform['location']['x']
                sumo_transform.location.y = ego_transform['location']['y']
                sumo_transform.location.z = ego_transform['location']['z']
                sumo_transform.rotation.roll = ego_transform['rotation']['roll']
                sumo_transform.rotation.pitch = ego_transform['rotation']['pitch']
                sumo_transform.rotation.yaw = ego_transform['rotation']['yaw']

                self.sumo.synchronize_vehicle(self.ego_id, sumo_transform, signals=None)
        
        sumo_context = {}

        for sumo_actor_id in self.sumo2carla_ids:

            sumo_actor = self.sumo.get_actor(sumo_actor_id)

            location = {}
            rotation = {}
            extent = {}

            # apply offset (may not be accurate)
            location["x"] = sumo_actor.transform.location.x + 97.0
            location["y"] = sumo_actor.transform.location.y + 122.0
            location["z"] = sumo_actor.transform.location.z - 34.5

            rotation["roll"] = sumo_actor.transform.rotation.roll
            rotation["pitch"] = sumo_actor.transform.rotation.pitch
            rotation["yaw"] = sumo_actor.transform.rotation.yaw

            extent["x"] = sumo_actor.extent.x
            extent["y"] = sumo_actor.extent.y
            extent["z"] = sumo_actor.extent.z

            actor = {}

            actor["location"] = location
            actor["rotation"] = rotation
            actor["extent"] = extent
            actor["type_id"] = sumo_actor.type_id
            actor["vclass"] = sumo_actor.vclass.value
            actor["color"] = sumo_actor.color
            actor["lights"] = sumo_actor.signals

            sumo_context[sumo_actor_id] = actor

        sumo_context_json = json.dumps(sumo_context)
        self.redis.set('sumo_context', sumo_context_json)

        # # Updates traffic lights in carla based on sumo information.
        # if self.tls_manager == 'sumo':
        #     common_landmarks = self.sumo.traffic_light_ids & self.carla.traffic_light_ids
        #     for landmark_id in common_landmarks:
        #         sumo_tl_state = self.sumo.get_traffic_light_state(landmark_id)
        #         carla_tl_state = BridgeHelper.get_carla_traffic_light_state(sumo_tl_state)
        #         self.carla.synchronize_traffic_light(landmark_id, carla_tl_state)

    def close(self):
        """
        Cleans synchronization.
        """
        for sumo_actor_id in self.sumo2carla_ids:
            self.sumo.destroy_actor(sumo_actor_id)

        # Closing sumo client.
        self.sumo.close()
        self.redis.flushall()


def synchronization_loop(args):
    """
    Entry point for sumo-carla co-simulation.
    """
    sumo_simulation = SumoSimulation(args.sumo_cfg_file, args.step_length, args.sumo_host,
                                     args.sumo_port, args.sumo_gui, args.client_order)

    synchronization = SimulationSynchronization(sumo_simulation, args.tls_manager,
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
