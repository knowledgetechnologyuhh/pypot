import os

from pyrep import PyRep
from pyrep.backend import vrep
from pyrep.backend._v_rep_cffi import lib
from pyrep.backend.vrepConst import sim_simulation_advancing
from pyrep.const import ObjectType, PrimitiveShape
from pyrep.objects.joint import Joint
from pyrep.objects.shape import Shape

from ..robot.io import AbstractIO


class PyRepIO(AbstractIO):

    """ This class is used to get/set values from/to a V-REP scene.

        It is based on PyRep (http://www.coppeliarobotics.com/helpFiles/en/PyRep.htm).

    """

    def __init__(self, scene="", start=False, headless=False,
                 responsive_ui=False, blocking=False):
        """ Starts the connection with the V-REP remote API server.

        :param str scene: path to a V-REP scene file
        :param bool start: whether to start the scene after loading it
        """
        self._closed = False
        self._collision_handles = {}
        self._objects = {}

        self.pyrep = PyRep()

        self.pyrep.launch(scene, headless, responsive_ui, blocking)

        if start:
            self.start_simulation()

    def close(self):
        if not self._closed:
            self.pyrep.shutdown()
            self._closed = True

    def load_scene(self, scene_path, start=False):
        """ Loads a scene on the V-REP server.

        :param str scene_path: path to a V-REP scene file
        :param bool start: whether to directly start the simulation after
                           loading the scene

        .. note:: It is assumed that the scene file is always available on the
                  server side.

        """
        self.stop_simulation()

        if not os.path.exists(scene_path):
            raise IOError("No such file or directory: '{}'".format(scene_path))

        vrep.simLoadScene(scene_path)

        if start:
            self.start_simulation()

    def start_simulation(self):
        """ Starts the simulation. """
        self.pyrep.start()

    def restart_simulation(self):
        """ Re-starts the simulation. """
        self.stop_simulation()
        self.start_simulation()

    def stop_simulation(self):
        """ Stops the simulation. """
        self.pyrep.stop()

    def pause_simulation(self):
        """ Pauses the simulation. """
        vrep.simPauseSimulation()

    def resume_simulation(self):
        """ Resumes the simulation. """
        self.start_simulation()

    def set_simulation_timestep(self, dt):
        """Sets the simulation time step.

        :param dt: The time step value.
        """
        self.pyrep.set_simulation_timestep(dt)

    def get_simulation_state(self):
        """Retrieves current simulation state"""
        return lib.simGetSimulationState()

    def simulation_step(self):
        """Execute the next simulation step.

        If the physics simulation is not running, then this will only update
        the UI.
        """
        self.pyrep.step()

    def ui_step(self):
        """Update the UI.

        This will not execute the next simulation step, even if the physics
        simulation is running.
        This is only applicable when PyRep was launched without a responsive UI.
        """
        self.pyrep.step_ui()

    def get_motor_position(self, motor_name):
        """ Gets the motor current position. """
        joint = self.get_object(motor_name)
        return joint.get_joint_position()

    def set_motor_position(self, motor_name, position):
        """ Sets the motor target position. """
        joint = self.get_object(motor_name)
        joint.set_joint_target_position(position)

    def get_motor_force(self, motor_name):
        """ Retrieves the force or torque applied to a joint along/about its
        active axis. """
        joint = self.get_object(motor_name)
        return joint.get_joint_force()

    def set_motor_force(self, motor_name, force):
        """  Sets the maximum force or torque that a joint can exert. """
        joint = self.get_object(motor_name)
        joint.set_joint_force(force)

    def get_motor_limits(self, motor_name):
        """ Retrieves joint limits """
        joint = self.get_object(motor_name)
        _, interval = joint.get_joint_interval()
        return interval[0], sum(interval)

    def get_object_position(self, object_name, relative_to_object=None):
        """ Gets the object position. """
        scene_object = self.get_object(object_name)
        if relative_to_object is not None:
            relative_to_object = self.get_object(relative_to_object)

        return scene_object.get_position(relative_to_object)

    def set_object_position(self, object_name, position=[0, 0, 0]):
        """ Sets the object position. """
        scene_object = self.get_object(object_name)
        scene_object.set_position(position)

    def get_object_orientation(self, object_name, relative_to_object=None):
        """ Gets the object orientation. """
        scene_object = self.get_object(object_name)
        if relative_to_object is not None:
            relative_to_object = self.get_object(relative_to_object)

        return scene_object.get_orientation(relative_to_object)

    def get_object_handle(self, name):
        """ Gets the vrep object handle. """
        scene_object = self.get_object(name)

        return scene_object.get_handle()

    def get_collision_state(self, collision_name):
        """ Gets the collision state. """
        return vrep.simReadCollision(self.get_collision_handle(collision_name))

    def get_collision_handle(self, collision):
        """ Gets a vrep collisions handle. """
        if collision not in self._collision_handles:
            h = vrep.simGetCollisionHandle(collision)
            self._collision_handles[collision] = h

        return self._collision_handles[collision]

    def get_simulation_current_time(self):
        """ Gets the simulation current time. """
        try:
            return lib.simGetSimulationTime()
        except VrepIOErrors:
            return 0.0

    def add_cube(self, name, size, mass=1., backface_culling=False,
                 visible_edges=False, smooth=False, respondable=True,
                 static=False, renderable=True, position=None,
                 orientation=None, color=None):
        """
        Add Cube

        :param name: Name of the created object.
        :param size: A list of the x, y, z dimensions.
        :param mass: A float representing the mass of the object.
        :param backface_culling: If backface culling is enabled.
        :param visible_edges: If the object will have visible edges.
        :param smooth: If the shape appears smooth.
        :param respondable: Shape is responsible.
        :param static: If the shape is static.
        :param renderable: If the shape is renderable.
        :param position: The x, y, z position.
        :param orientation: The z, y, z orientation (in radians).
        :param color: The r, g, b values of the shape.
        """
        self._create_pure_shape(name, PrimitiveShape.CUBOID, size, mass,
                                backface_culling, visible_edges, smooth,
                                respondable, static, renderable, position,
                                orientation, color)

    def add_sphere(self, name, size, mass=1., backface_culling=False,
                   visible_edges=False, smooth=False, respondable=True,
                   static=False, renderable=True, position=None,
                   orientation=None, color=None):
        """
        Add Sphere

        :param name: Name of the created object.
        :param size: A list of the x, y, z dimensions.
        :param mass: A float representing the mass of the object.
        :param backface_culling: If backface culling is enabled.
        :param visible_edges: If the object will have visible edges.
        :param smooth: If the shape appears smooth.
        :param respondable: Shape is responsible.
        :param static: If the shape is static.
        :param renderable: If the shape is renderable.
        :param position: The x, y, z position.
        :param orientation: The z, y, z orientation (in radians).
        :param color: The r, g, b values of the shape.
        """
        self._create_pure_shape(name, PrimitiveShape.SPHERE, size, mass,
                                backface_culling, visible_edges, smooth,
                                respondable, static, renderable, position,
                                orientation, color)

    def add_cylinder(self, name, size, mass=1., backface_culling=False,
                     visible_edges=False, smooth=False, respondable=True,
                     static=False, renderable=True, position=None,
                     orientation=None, color=None):
        """
        Add Cylinder

        :param name: Name of the created object.
        :param size: A list of the x, y, z dimensions.
        :param mass: A float representing the mass of the object.
        :param backface_culling: If backface culling is enabled.
        :param visible_edges: If the object will have visible edges.
        :param smooth: If the shape appears smooth.
        :param respondable: Shape is responsible.
        :param static: If the shape is static.
        :param renderable: If the shape is renderable.
        :param position: The x, y, z position.
        :param orientation: The z, y, z orientation (in radians).
        :param color: The r, g, b values of the shape.
        """
        self._create_pure_shape(name, PrimitiveShape.CYLINDER, size, mass,
                                backface_culling, visible_edges, smooth,
                                respondable, static, renderable, position,
                                orientation, color)

    def add_cone(self, name, size, mass=1., backface_culling=False,
                 visible_edges=False, smooth=False, respondable=True,
                 static=False, renderable=True, position=None,
                 orientation=None, color=None):
        """
        Add Cone

        :param name: Name of the created object.
        :param size: A list of the x, y, z dimensions.
        :param mass: A float representing the mass of the object.
        :param backface_culling: If backface culling is enabled.
        :param visible_edges: If the object will have visible edges.
        :param smooth: If the shape appears smooth.
        :param respondable: Shape is responsible.
        :param static: If the shape is static.
        :param renderable: If the shape is renderable.
        :param position: The x, y, z position.
        :param orientation: The z, y, z orientation (in radians).
        :param color: The r, g, b values of the shape.
        """
        self._create_pure_shape(name, PrimitiveShape.CONE, size, mass,
                                backface_culling, visible_edges, smooth,
                                respondable, static, renderable, position,
                                orientation, color)

    def change_object_name(self, obj, new_name):
        """ Change object name """
        old_name = obj.get_name()
        if old_name in self._objects:
            self._objects.pop(old_name)
        obj.set_name(new_name)

    def _create_pure_shape(self, name, primitive_type, size, mass=1.,
                           backface_culling=False, visible_edges=False,
                           smooth=False, respondable=True, static=False,
                           renderable=True, position=None, orientation=None,
                           color=None):
        """
        Create Pure Shape

        :param name: Name of the created object.
        :param primitive_type: The type of primitive to shape. One of:
            PrimitiveShape.CUBOID
            PrimitiveShape.SPHERE
            PrimitiveShape.CYLINDER
            PrimitiveShape.CONE
        :param size: A list of the x, y, z dimensions.
        :param mass: A float representing the mass of the object.
        :param backface_culling: If backface culling is enabled.
        :param visible_edges: If the object will have visible edges.
        :param smooth: If the shape appears smooth.
        :param respondable: Shape is responsible.
        :param static: If the shape is static.
        :param renderable: If the shape is renderable.
        :param position: The x, y, z position.
        :param orientation: The z, y, z orientation (in radians).
        :param color: The r, g, b values of the shape.
        """
        obj = Shape.create(primitive_type, size, mass, backface_culling,
                           visible_edges, smooth, respondable, static,
                           renderable, position, orientation, color)
        self.change_object_name(obj, name)

    def _create_object(self, name):
        """Creates pyrep object for the correct type"""
        # TODO implement other types
        handle = vrep.simGetObjectHandle(name)
        o_type = vrep.simGetObjectType(handle)
        if ObjectType(o_type) == ObjectType.JOINT:
            return Joint(handle)
        elif ObjectType(o_type) == ObjectType.SHAPE:
            return Shape(handle)
        else:
            return None

    def get_object(self, name):
        """ Gets vrep scene object"""
        if name not in self._objects:
            self._objects[name] = self._create_object(name)
        return self._objects[name]

    # TODO integrate other pyrep functions


class VrepIOErrors(Exception):
    pass
