import json
import logging
import time as sys_time
from collections import OrderedDict
from functools import partial

from pyrep import PyRep  # for some reason this solves vrep Error: signal 11
from pyrep.backend.simConst import sim_simulation_advancing

import pypot.utils.pypot_time as pypot_time

from ..robot import Robot
from ..robot.config import make_alias, motor_from_confignode
from ..robot.sensor import ObjectTracker
from .controller import (
    PyRepController,
    CoppeliaSimCollisionDetector,
    CoppeliaSimCollisionTracker,
    CoppeliaSimObjectTracker,
)
from .io import PyRepIO

logger = logging.getLogger(__name__)


class coppelia_sim_time:
    def __init__(self, pyrep_io):
        self.io = pyrep_io
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
                if (
                    self.get_time() < t0
                    or self.io.get_simulation_state() < sim_simulation_advancing
                ):
                    break
                sys_time.sleep(0.01)


def from_pyrep(
    config,
    scene,
    start=False,
    headless=False,
    responsive_ui=False,
    tracked_objects=[],
    tracked_collisions=[],
    id=None,
    shared_pyrep_io=None,
):
    """ Launches PyRep instance with given scene and creates robot from it.

    :param config: robot configuration (either the path to the json or directly the dictionary)
    :type config: str or dict
    :param str scene: path to the CoppeliaSim scene to load and start
    :param bool start: automatically start simulation after initialization
    :param bool headless: launch CoppeliaSim in headless mode
    :param bool responsive_ui: enables ui interaction
    :param list tracked_objects: list of CoppeliaSim dummy object to track
    :param list tracked_collisions: list of CoppeliaSim collision to track
    :param int id: robot id in simulator (useful when using a scene with multiple robots)
    :param shared_pyrep_io: use an already connected PyRepIO (useful when using a scene with multiple robots)
    :type shared_pyrep_io: :class:`~pypot.pyrep.io.PyRepIO`

    This function launches a CoppeliaSim instance and expects to find motors with names corresponding as the ones found in the config.

    .. note:: The :class:`~pypot.robot.robot.Robot` returned will also provide a convenience reset_simulation method which resets the simulation and the robot position to its intial stance.

    .. note:: Using the same configuration, you should be able to switch from a real to a simulated robot just by switching from :func:`~pypot.robot.config.from_config` to :func:`~pypot.pyrep.from_pyrep`.
        For instance::

            import json

            with open('my_config.json') as f:
                config = json.load(f)

            from pypot.robot import from_config
            from pypot.pyrep import from_pyrep

            real_robot = from_config(config)
            simulated_robot = from_pyrep(config, 'poppy.ttt')

    """
    # pyrep_instance = PyRep()
    if shared_pyrep_io is None:
        pyrep_io = PyRepIO(scene, start, headless, responsive_ui)
    else:
        pyrep_io = shared_pyrep_io

    pyreptime = coppelia_sim_time(pyrep_io)
    pypot_time.time = pyreptime.get_time
    pypot_time.sleep = pyreptime.sleep

    if isinstance(config, str):
        with open(config) as f:
            config = json.load(f, object_pairs_hook=OrderedDict)

    motors = [motor_from_confignode(config, name) for name in config["motors"].keys()]

    vc = PyRepController(pyrep_io, motors, id=id)

    sensor_controllers = []

    if tracked_objects:
        sensors = [ObjectTracker(name) for name in tracked_objects]
        vot = CoppeliaSimObjectTracker(pyrep_io, sensors)
        sensor_controllers.append(vot)

    if tracked_collisions:
        sensors = [CoppeliaSimCollisionDetector(name) for name in tracked_collisions]
        vct = CoppeliaSimCollisionTracker(pyrep_io, sensors)
        sensor_controllers.append(vct)

    robot = Robot(motor_controllers=[vc], sensor_controllers=sensor_controllers)

    for m in robot.motors:
        m.goto_behavior = "minjerk"

    init_pos = {m: m.goal_position for m in robot.motors}

    make_alias(config, robot)

    def start_simu():
        vc.start()
        if tracked_objects:
            vot.start()
        if tracked_collisions:
            vct.start()
        pyrep_io.start_simulation()
        for m, p in init_pos.items():
            m.goal_position = p

    def stop_simu():
        pyrep_io.stop_simulation()
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
        pyrep_io.simulation_step()

    robot.start_simulation = start_simu
    robot.stop_simulation = stop_simu
    robot.reset_simulation = reset_simu
    robot.simulation_step = step_simu

    def current_simulation_time(robot):
        return robot._controllers[0].io.get_simulation_current_time()

    Robot.current_simulation_time = property(
        lambda robot: current_simulation_time(robot)
    )

    def get_object_position(robot, object, relative_to_object=None):
        return pyrep_io.get_object_position(object, relative_to_object)

    Robot.get_object_position = partial(get_object_position, robot)

    def get_object_orientation(robot, object, relative_to_object=None):
        return pyrep_io.get_object_orientation(object, relative_to_object)

    Robot.get_object_orientation = partial(get_object_orientation, robot)

    return robot
