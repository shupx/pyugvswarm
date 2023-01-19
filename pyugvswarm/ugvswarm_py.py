import argparse
import atexit

import numpy as np
import os # added by spx

from . import genericJoystick

# Building the parser in a separate function allows sphinx-argparse to
# auto-generate the documentation for the command-line flags.
def build_argparser(parent_parsers=[]):
    parser = argparse.ArgumentParser(
        # description='ugvswarm python wrapper',
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
        parents=parent_parsers
    )
    parser.add_argument("--sim", help="Run using simulation.", action="store_true")

    sim_group = parser.add_argument_group("Simulation-only", "")
    sim_group.add_argument("--vis", help="Visualization backend.", choices=['mpl', 'vispy', 'null'], default="mpl")
    sim_group.add_argument("--dt", help="Duration of seconds between rendered visualization frames.", type=float, default=0.1)
    sim_group.add_argument("--writecsv", help="Enable CSV output(Not working!).", action="store_true")
    sim_group.add_argument("--disturbance", help="Simulate Gaussian-distributed disturbance when using cmdVelocityWorld.", type=float, default=0.0)
    sim_group.add_argument("--maxvel", help="Limit simulated velocity (meters/sec).", type=float, default=np.inf)
    sim_group.add_argument("--video", help="Video output path.", type=str)    

    return parser


class UGVswarm:
    def __init__(self, ugvs_yaml=None, parent_parser=None, args=None):
        if parent_parser is not None:
            parents = [parent_parser]
        else:
            parents = []
        parser = build_argparser(parents)
        if isinstance(args, str):
            args = args.split()
        args, unknown = parser.parse_known_args(args)

        if ugvs_yaml is None:
            ## absolute path, modified by spx ##
            folder = os.path.dirname(os.path.abspath(__file__)) # absolute path of this folder 
            folder = os.path.dirname(folder) # up folder
            ugvs_yaml = folder + "/config/ugvs.yaml"
            # ugvs_yaml = "./config/ugvs.yaml"
        if ugvs_yaml.endswith(".yaml"):
            print('[UGVswarm] load ugvs_yaml: {}'.format(ugvs_yaml))
            ugvs_yaml = open(ugvs_yaml, 'r').read()

        if args.sim:
            from .ugvSim import TimeHelper, UGVServer
            self.timeHelper = TimeHelper(args.vis, args.dt, args.writecsv, disturbanceSize=args.disturbance, maxVel=args.maxvel, videopath=args.video)
            self.allugvs = UGVServer(self.timeHelper, ugvs_yaml)
            atexit.register(self.timeHelper._atexit)
        else:
            from .ugv import TimeHelper, UGVServer
            self.allugvs = UGVServer(ugvs_yaml)
            self.timeHelper = TimeHelper()
            if args.writecsv:
                print("WARNING: writecsv argument ignored! This is only available in simulation.")
            if args.video:
                print("WARNING: video argument ignored! This is only available in simulation.")

        self.input = genericJoystick.Joystick(self.timeHelper)
