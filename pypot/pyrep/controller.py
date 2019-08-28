from collections import deque

from numpy import deg2rad, rad2deg

from ..dynamixel.conversion import torque_max
from ..robot.controller import MotorsController, SensorsController
from ..robot.sensor import Sensor


class PyRepController(MotorsController):
    """ V-REP motors controller using PyRep. """

    def __init__(self, vrep_io, motors, sync_freq=50., id=None):
        """
        :param vrep_io: vrep io instance
        :type vrep_io: :class:`~pypot.vrep.io.VrepIO`
        :param str scene: path to the V-REP scene file to start
        :param list motors: list of motors attached to the controller
        :param float sync_freq: synchronization frequency
        :param int id: robot id in simulator (useful when using a scene with multiple robots)

        """
        MotorsController.__init__(self, vrep_io, motors, sync_freq)

        self.id = id

        # if scene is not None:
        #     vrep_io.load_scene(scene, start=True)

    def setup(self):
        """
        Setups the controller by reading/setting position for all motors.
        """
        self._init_vrep_streaming()

        # Init lifo for temperature spoofing
        for m in self.motors:
            m.__dict__['_load_fifo'] = deque(200 * [1], maxlen=200)
        self.update()

    def update(self):
        """ Synchronization update loop.

        At each update all motor position are read from vrep and set to the
        motors. The motors target position are also send to v-rep.

        """
        for m in self.motors:
            tmax = torque_max[m.model]

            # Read values from V-REP and set them to the Motor
            p = round(
                rad2deg(self.io.get_motor_position(motor_name=self._motor_name(m))), 1)
            m.__dict__['present_position'] = p

            try:
                l = 100. * \
                    self.io.get_motor_force(motor_name=self._motor_name(m)) / \
                    tmax
            except RuntimeError:
                l = 100.
            m.__dict__['present_load'] = l

            m.__dict__['_load_fifo'].append(abs(l))
            m.__dict__['present_temperature'] = 25 + \
                round(2.5 * sum(m.__dict__['_load_fifo']
                                ) / len(m.__dict__['_load_fifo']), 1)

            lower, upper = self.io.get_motor_limits(self._motor_name(m))
            m.__dict__['lower_limit'] = lower
            m.__dict__['upper_limit'] = upper

            # Send new values from Motor to V-REP
            p = deg2rad(round(m.__dict__['goal_position'], 1))
            self.io.set_motor_position(
                motor_name=self._motor_name(m), position=p)

            t = m.__dict__['torque_limit'] * tmax / 100.

            if m.__dict__['compliant']:
                t = 0.

            self.io.set_motor_force(motor_name=self._motor_name(m), force=t)

    def _init_vrep_streaming(self):
        # Initialize motor positions
        # get initial position
        pos = [self.io.get_motor_position(
            self._motor_name(m)) for m in self.motors]

        # update goal positions
        for m, p in zip(self.motors, pos):
            self.io.set_motor_position(self._motor_name(m), p)
            m.__dict__['goal_position'] = rad2deg(p)

        for m in self.motors:
            self.io.set_motor_force(self._motor_name(m), torque_max[m.model])
            m.__dict__['torque_limit'] = 100.
            m.__dict__['compliant'] = False

    def _motor_name(self, m):
        if self.id is None:
            return m.name
        else:
            return '{}{}'.format(m.name, self.id)


class VrepObjectTracker(SensorsController):

    """ Tracks the 3D position and orientation of a V-REP object. """

    def setup(self):
        """ Forces a first update to trigger V-REP streaming. """
        self.update()

    def update(self):
        """ Updates the position and orientation of the tracked objects. """
        for s in self.sensors:
            s.position = self.io.get_object_position(object_name=s.name)
            s.orientation = self.io.get_object_orientation(object_name=s.name)


class VrepCollisionDetector(Sensor):

    def __init__(self, name):
        Sensor.__init__(self, name)

        self._colliding = False

    @property
    def colliding(self):
        return self._colliding

    @colliding.setter
    def colliding(self, new_state):
        self._colliding = new_state


class VrepCollisionTracker(SensorsController):

    """ Tracks collision state. """

    def setup(self):
        """ Forces a first update to trigger V-REP streaming. """
        self.update()

    def update(self):
        """ Update the state of the collision detectors. """

        for s in self.sensors:
            s.colliding = self.io.get_collision_state(collision_name=s.name)
