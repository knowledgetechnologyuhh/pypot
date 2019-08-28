import json
import logging
import time as sys_time
from collections import OrderedDict
from functools import partial

from pyrep import PyRep  # for some reason this solves vrep Error: signal 11
from pyrep.backend.vrepConst import sim_simulation_advancing

import pypot.utils.pypot_time as pypot_time

from ..robot import Robot
from ..robot.config import make_alias, motor_from_confignode
from ..robot.sensor import ObjectTracker
from .controller import (PyRepController, VrepCollisionDetector,
                         VrepCollisionTracker, VrepObjectTracker)
from .io import PyRepIO

logger = logging.getLogger(__name__)


class vrep_time():
    def __init__(self, vrep_io):
        self.io = vrep_io
        self.disabled = False

    def get_time(self, trial=0):
        t = self.io.get_simulation_current_time()

        return t

    def sleep(self, t):
        if self.io.get_simulation_state() < sim_simulation_advancing:
            sys_time.sleep(t)
        else:
            t0 = self.get_time()
            while (self.get_time() - t0) < t:
                if (self.get_time() < t0 or self.io.get_simulation_state() <
                        sim_simulation_advancing):
                    break
                sys_time.sleep(0.01)


def from_vrep(config, scene, start=False, headless=False, responsive_ui=False,
              tracked_objects=[], tracked_collisions=[], id=None,
              shared_vrep_io=None):
    # TODO edit copied docstring
    """ Launches V-REP instance with given scene and creates robot from it.

    :param config: robot configuration (either the path to the json or directly the dictionary)
    :type config: str or dict
    :param str scene: path to the V-REP scene to load and start
    :param bool start: automatically start simulation after initialization
    :param bool headless: launch vrep in headless mode
    :param bool responsive_ui: enables ui interaction
    :param list tracked_objects: list of V-REP dummy object to track
    :param list tracked_collisions: list of V-REP collision to track
    :param int id: robot id in simulator (useful when using a scene with multiple robots)
    :param vrep_io: use an already connected VrepIO (useful when using a scene with multiple robots)
    :type vrep_io: :class:`~pypot.pyrep.io.PyRepIO`

    This function tries to connect to a V-REP instance and expects to find motors with names corresponding as the ones found in the config.

    .. note:: The :class:`~pypot.robot.robot.Robot` returned will also provide a convenience reset_simulation method which resets the simulation and the robot position to its intial stance.

    .. note:: Using the same configuration, you should be able to switch from a real to a simulated robot just by switching from :func:`~pypot.robot.config.from_config` to :func:`~pypot.vrep.from_vrep`.
        For instance::

            import json

            with open('my_config.json') as f:
                config = json.load(f)

            from pypot.robot import from_config
            from pypot.pyrep import from_vrep

            real_robot = from_config(config)
            simulated_robot = from_vrep(config, 'poppy.ttt')

    """
    # pyrep_instance = PyRep()
    if shared_vrep_io is None:
        vrep_io = PyRepIO(scene, start, headless, responsive_ui)
    else:
        vrep_io = shared_vrep_io

    vreptime = vrep_time(vrep_io)
    pypot_time.time = vreptime.get_time
    pypot_time.sleep = vreptime.sleep

    if isinstance(config, str):
        with open(config) as f:
            config = json.load(f, object_pairs_hook=OrderedDict)

    motors = [motor_from_confignode(config, name)
              for name in config['motors'].keys()]

    vc = PyRepController(vrep_io, motors, id=id)

    sensor_controllers = []

    if tracked_objects:
        sensors = [ObjectTracker(name) for name in tracked_objects]
        vot = VrepObjectTracker(vrep_io, sensors)
        sensor_controllers.append(vot)

    if tracked_collisions:
        sensors = [VrepCollisionDetector(name) for name in tracked_collisions]
        vct = VrepCollisionTracker(vrep_io, sensors)
        sensor_controllers.append(vct)

    robot = Robot(motor_controllers=[vc],
                  sensor_controllers=sensor_controllers)

    for m in robot.motors:
        m.goto_behavior = 'minjerk'

    init_pos = {m: m.goal_position for m in robot.motors}

    make_alias(config, robot)

    def start_simu():
        vc.start()
        if tracked_objects:
            vot.start()
        if tracked_collisions:
            vct.start()
        vrep_io.start_simulation()
        for m, p in init_pos.items():
            m.goal_position = p

    def stop_simu():
        vrep_io.stop_simulation()
        vc.stop()
        if tracked_objects:
            vot.stop()
        if tracked_collisions:
            vct.stop()

    def reset_simu():
        stop_simu()
        sys_time.sleep(0.5)
        start_simu()

    def step_simu():
        vrep_io.simulation_step()

    robot.start_simulation = start_simu
    robot.stop_simulation = stop_simu
    robot.reset_simulation = reset_simu
    robot.simulation_step = step_simu

    def current_simulation_time(robot):
        return robot._controllers[0].io.get_simulation_current_time()
    Robot.current_simulation_time = property(
        lambda robot: current_simulation_time(robot))

    def get_object_position(robot, object, relative_to_object=None):
        return vrep_io.get_object_position(object, relative_to_object)
    Robot.get_object_position = partial(get_object_position, robot)

    def get_object_orientation(robot, object, relative_to_object=None):
        return vrep_io.get_object_orientation(object, relative_to_object)
    Robot.get_object_orientation = partial(get_object_orientation, robot)

    return robot
