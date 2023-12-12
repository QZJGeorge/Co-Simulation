# -*- coding: utf-8 -*-
import json
import os
import sys

import typer
from typer import Option

try:
    if os.name == 'posix':
        import libsumo as traci

        print('libsumo imported!')
    else:
        import traci

        print('traci imported!')
except ImportError:
    import traci

DEBUG_MODE = bool(getattr(sys, 'gettrace', None) and getattr(sys, 'gettrace')())


def _run(sumo_conf: str = Option('sumo_map/mcity.sumocfg', '--config', help='SUMO config file.'),
         output_dir: str = Option('sumo_outputs/optimization', '--output-dir', help='Output directory.'),
         iter_idx: int = Option(..., '--iter-idx', help='Number of iterations.'),
         end_time: int = Option(50400, '--end-time', help='Simulation end time.')):
    return run_sim(sumo_conf, output_dir, iter_idx, end_time)


def run_sim(sumo_conf, output_dir, iter_idx, end_time):
    sim_output_dir = f"{output_dir}/iter_{iter_idx:03d}/"
    if not os.path.exists(sim_output_dir):
        os.makedirs(sim_output_dir)
    cmd = ["sumo",
           "-c", sumo_conf,
           "--output-prefix", f'../{sim_output_dir}']
    print(f'Start running sim  {" ".join(cmd)}')

    traci.start(cmd)
    while traci.simulation.getTime() < end_time:
        traci.simulationStep()
    traci.close()

    print('Sim finished.')
    return sim_output_dir


if __name__ == '__main__':
    typer.run(_run)
