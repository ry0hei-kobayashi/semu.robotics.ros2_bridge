# semu.robotics.ros2_bridge -- ROS 2 action/topic bridge for Isaac Sim articulations
#
# Upstream: https://github.com/Toni-SM/semu.robotics.ros2_bridge (MIT License, Toni-SM)
# This fork (branch isaac5.1-humble) is a port to Isaac Sim 5.1 / Python 3.11 /
# ROS 2 Humble. It is no longer a Kit extension: the controllers are plain Python
# classes created by the host application. See docs/CHANGELOG.md for the list of
# changes relative to upstream.

from typing import List, Any

import math
import time
import json
import asyncio
import threading

import omni
import carb
import omni.kit
from pxr import Usd, Gf, PhysxSchema
from omni.isaac.dynamic_control import _dynamic_control
# isaacsim.* is the Isaac Sim >= 4.5 namespace. omni.isaac.dynamic_control is
# deprecated since 4.5 but still shipped (extsDeprecated) in 5.1; the dynamic
# control interface itself is injected by the caller (see __init__ of the
# controllers), this module only needs its constants.
from isaacsim.core.utils.stage import get_stage_units

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.action import FollowJointTrajectory
from control_msgs.action import GripperCommand
from control_msgs.msg import JointTrajectoryControllerState

# add_on_msgs (get/set prim attribute services) is built by tools/build.sh and shipped in
# packages/. It is optional (hosts that only use the action servers may not have it) and
# it is imported lazily: as a Kit extension this module is imported by scripts/extension.py
# before on_startup() has added packages/ to sys.path.
GetPrims = GetPrimAttributes = GetPrimAttribute = SetPrimAttribute = None


def _load_add_on_msgs() -> bool:
    """Import the add_on_msgs services (idempotent). Returns True when available."""
    global GetPrims, GetPrimAttributes, GetPrimAttribute, SetPrimAttribute
    if GetPrims is not None:
        return True
    try:
        from add_on_msgs.srv import GetPrims as _p, GetPrimAttributes as _as, GetPrimAttribute as _a, SetPrimAttribute as _s
    except ImportError:
        return False
    GetPrims, GetPrimAttributes, GetPrimAttribute, SetPrimAttribute = _p, _as, _a, _s
    return True


def acquire_ros2_bridge_interface(ext_id: str = "") -> 'Ros2Bridge':
    """Acquire the Ros2Bridge interface (extension entry point)

    :param ext_id: The extension id
    :type ext_id: str

    :returns: The Ros2Bridge interface
    :rtype: Ros2Bridge
    """
    if not rclpy.ok():
        rclpy.init()
    bridge = Ros2Bridge()

    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(bridge)
    threading.Thread(target=executor.spin, daemon=True).start()

    return bridge


def release_ros2_bridge_interface(bridge: 'Ros2Bridge') -> None:
    """Release the Ros2Bridge interface

    :param bridge: The Ros2Bridge interface
    :type bridge: Ros2Bridge
    """
    bridge.shutdown()


def _setting(name: str, default):
    """Read /exts/semu.robotics.ros2_bridge/<name> from carb settings, with a default
    for hosts that use this module without the extension (settings not populated)."""
    try:
        value = carb.settings.get_settings().get("/exts/semu.robotics.ros2_bridge/{}".format(name))
    except Exception:
        value = None
    return default if value is None else value


class Ros2Bridge(Node):
    """ROS 2 node that owns the bridge components when the module is used as a Kit extension.

    Upstream discovered the components from ``semu.usd.schemas`` prims on the stage.
    This port has no schema dependency: the components are listed in the extension
    setting ``/exts/semu.robotics.ros2_bridge/components`` (JSON, see config/extension.toml)
    and are (re)created every time the timeline starts playing.
    """
    def __init__(self) -> None:
        self._components = []
        self._node_name = _setting("nodeName", "SemuRos2Bridge")

        super().__init__(self._node_name)

        # omni objects and interfaces
        self._usd_context = omni.usd.get_context()
        self._timeline = omni.timeline.get_timeline_interface()
        self._physx_interface = omni.physx.acquire_physx_interface()
        self._dci = _dynamic_control.acquire_dynamic_control_interface()

        # events
        self._timeline_event = self._timeline.get_timeline_event_stream().create_subscription_to_pop(self._on_timeline_event)
        self._physx_event = self._physx_interface.subscribe_physics_step_events(self._on_physics_event)

    def shutdown(self) -> None:
        """Shutdown the Ros2Bridge interface
        """
        self._timeline_event = None
        self._physx_event = None

        self._stop_components()
        self.destroy_node()

    def _component_specs(self) -> list:
        """Parse the ``components`` setting (JSON list of dicts)."""
        raw = _setting("components", "[]")
        try:
            specs = json.loads(raw) if isinstance(raw, str) else list(raw)
        except (TypeError, ValueError) as exc:
            print("[Error][semu.robotics.ros2_bridge] Ros2Bridge: invalid 'components' setting ({}): {}".format(exc, raw))
            return []
        return [spec for spec in specs if isinstance(spec, dict)]

    def _stop_components(self) -> None:
        """Stop all components
        """
        for component in self._components:
            component.stop()

    def _reload_components(self) -> None:
        """Recreate and start all components from the ``components`` setting
        """
        self._stop_components()
        self._components = []
        for spec in self._component_specs():
            kind = spec.get("type", "")
            try:
                if kind == "Attribute":
                    component = RosAttribute(self, self._dci)
                    component.start(prims_srv=spec.get("prims_srv", "/get_prims"),
                                    get_attribute_srv=spec.get("get_attribute_srv", "/get_prim_attribute"),
                                    attributes_srv=spec.get("attributes_srv", "/get_prim_attributes"),
                                    set_attribute_srv=spec.get("set_attribute_srv", "/set_prim_attribute"))
                elif kind == "FollowJointTrajectory":
                    component = RosControlFollowJointTrajectory(self, self._dci)
                    component.start(spec["articulation"], spec["action"])
                elif kind == "GripperCommand":
                    component = RosControllerGripperCommand(self, self._dci)
                    component.start(spec["articulation"], spec["action"])
                else:
                    print("[Warning][semu.robotics.ros2_bridge] Ros2Bridge: unknown component type '{}'".format(kind))
                    continue
            except KeyError as exc:
                print("[Error][semu.robotics.ros2_bridge] Ros2Bridge: component {} is missing key {}".format(spec, exc))
                continue
            self._components.append(component)

    def _on_timeline_event(self, event: 'carb.events._events.IEvent') -> None:
        """Handle the timeline event

        :param event: Event
        :type event: carb.events._events.IEvent
        """
        if event.type == int(omni.timeline.TimelineEventType.PLAY):
            self._reload_components()
            print("[Info][semu.robotics.ros2_bridge] Ros2Bridge: components reloaded ({})".format(len(self._components)))
        elif event.type in (int(omni.timeline.TimelineEventType.STOP), int(omni.timeline.TimelineEventType.PAUSE)):
            self._stop_components()
            print("[Info][semu.robotics.ros2_bridge] Ros2Bridge: components stopped")

    def _on_physics_event(self, step: float) -> None:
        """Handle the physics event

        :param step: The physics step
        :type step: float
        """
        for component in self._components:
            component.step(step)


class RosController:
    def __init__(self, node: Node) -> None:
        """Base class for RosController

        :param node: ROS2 node
        :type node: Node
        """
        self._node = node        
        self.started = False
    
    def start(self) -> None:
        """Start the component
        """
        raise NotImplementedError

    def stop(self) -> None:
        """Stop the component
        """
        print("[Info][semu.robotics.ros2_bridge] RosController: stopping")
        self.started = False

    def step(self, dt: float) -> None:
        """Physics update step

        :param dt: The physics delta time
        :type dt: float
        """
        raise NotImplementedError


class RosAttribute(RosController):
    def __init__(self, node: Node, dci: 'omni.isaac.dynamic_control.DynamicControl') -> None:
        """RosAttribute interface: ROS 2 services to list prims and get/set prim attributes

        :param node: The ROS node
        :type node: rclpy.node.Node
        :param dci: The dynamic control interface
        :type dci: omni.isaac.dynamic_control.DynamicControl
        """
        super().__init__(node)
        if not _load_add_on_msgs():
            raise ImportError("add_on_msgs is not importable: build it with tools/build.sh and add packages/ to sys.path")

        self._dci = dci
        self._usd_context = omni.usd.get_context()

        self._srv_prims = None
        self._srv_attributes = None
        self._srv_getter = None
        self._srv_setter = None

        self._value = None
        self._attribute = None

        self._event = threading.Event()
        self._event.set()

        self.__event_timeout = float(_setting("eventTimeout", 5.0))
        self.__set_attribute_using_asyncio = bool(_setting("setAttributeUsingAsyncio", False))
        print("[Info][semu.robotics.ros2_bridge] RosAttribute: asyncio: {}".format(self.__set_attribute_using_asyncio))
        print("[Info][semu.robotics.ros2_bridge] RosAttribute: event timeout: {}".format(self.__event_timeout))

    async def _set_attribute(self, attribute: 'Usd.Attribute', attribute_value: Any) -> None:
        """Set the attribute value using asyncio

        :param attribute: The prim's attribute to set
        :type attribute: pxr.Usd.Attribute
        :param attribute_value: The attribute value
        :type attribute_value: Any
        """
        attribute.Set(attribute_value)

    def _process_setter_request(self,
                                request: 'SetPrimAttribute.Request',
                                response: 'SetPrimAttribute.Response') -> 'SetPrimAttribute.Response':
        """Process the setter request

        :param request: The service request
        :type request: SetPrimAttribute.Request
        :param response: The service response
        :type response: SetPrimAttribute.Response

        :return: The service response
        :rtype: SetPrimAttribute.Response
        """
        response.success = False
        stage = self._usd_context.get_stage()
        # get prim
        if stage.GetPrimAtPath(request.path).IsValid():
            prim = stage.GetPrimAtPath(request.path)
            if request.attribute and prim.HasAttribute(request.attribute):
                # attribute
                attribute = prim.GetAttribute(request.attribute)
                attribute_type = type(attribute.Get()).__name__
                # value
                try:
                    value = json.loads(request.value)
                    attribute_value = None
                except json.JSONDecodeError:
                    print("[Error][semu.robotics.ros2_bridge] RosAttribute: invalid value: {}".format(request.value))
                    response.success = False
                    response.message = "Invalid value '{}'".format(request.value)
                    return response
                # parse data
                try:
                    if attribute_type in ['Vec2d', 'Vec2f', 'Vec2h', 'Vec2i']:
                        attribute_value = type(attribute.Get())(value)
                    elif attribute_type in ['Vec3d', 'Vec3f', 'Vec3h', 'Vec3i']:
                        attribute_value = type(attribute.Get())(value)
                    elif attribute_type in ['Vec4d', 'Vec4f', 'Vec4h', 'Vec4i']:
                        attribute_value = type(attribute.Get())(value)
                    elif attribute_type in ['Quatd', 'Quatf', 'Quath']:
                        attribute_value = type(attribute.Get())(*value)
                    elif attribute_type in ['Matrix4d', 'Matrix4f']:
                        attribute_value = type(attribute.Get())(value)
                    elif attribute_type.startswith('Vec') and attribute_type.endswith('Array'):
                        attribute_value = type(attribute.Get())(value)
                    elif attribute_type.startswith('Matrix') and attribute_type.endswith('Array'):
                        if attribute_type.endswith("dArray"):
                            attribute_value = type(attribute.Get())([Gf.Matrix2d(v) for v in value])
                        elif attribute_type.endswith("fArray"):
                            attribute_value = type(attribute.Get())([Gf.Matrix2f(v) for v in value])
                    elif attribute_type.startswith('Quat') and attribute_type.endswith('Array'):
                        if attribute_type.endswith("dArray"):
                            attribute_value = type(attribute.Get())([Gf.Quatd(*v) for v in value])
                        elif attribute_type.endswith("fArray"):
                            attribute_value = type(attribute.Get())([Gf.Quatf(*v) for v in value])
                        elif attribute_type.endswith("hArray"):
                            attribute_value = type(attribute.Get())([Gf.Quath(*v) for v in value])
                    elif attribute_type.endswith('Array'):
                        attribute_value = type(attribute.Get())(value)
                    elif attribute_type in ['AssetPath']:
                        attribute_value = type(attribute.Get())(value)
                    elif attribute_type in ['NoneType']:
                        pass
                    else:
                        attribute_value = type(attribute.Get())(value)

                    # set attribute
                    if attribute_value is not None:
                        # set attribute using asyncio
                        if self.__set_attribute_using_asyncio:
                            try:
                                loop = asyncio.get_event_loop()
                            except Exception:
                                loop = asyncio.new_event_loop()
                            asyncio.set_event_loop(loop)
                            future = asyncio.ensure_future(self._set_attribute(attribute, attribute_value))
                            loop.run_until_complete(future)
                            response.success = True
                        # set attribute in the physics step
                        else:
                            self._attribute = attribute
                            self._value = attribute_value

                            self._event.clear()
                            response.success = self._event.wait(self.__event_timeout)
                            if not response.success:
                                response.message = "The timeout ({} s) for setting the attribute value has been reached" \
                                    .format(self.__event_timeout)

                except Exception as e:
                    print("[Error][semu.robotics.ros2_bridge] RosAttribute: srv {} request for {} ({}: {}): {}" \
                        .format(self._srv_setter.srv_name, request.path, request.attribute, value, e))
                    response.success = False
                    response.message = str(e)
            else:
                response.message = "Prim has not attribute {}".format(request.attribute)
        else:
            response.message = "Invalid prim ({})".format(request.path)
        return response

    def _process_getter_request(self,
                                request: 'GetPrimAttribute.Request',
                                response: 'GetPrimAttribute.Response') -> 'GetPrimAttribute.Response':
        """Process the getter request

        :param request: The service request
        :type request: GetPrimAttribute.Request
        :param response: The service response
        :type response: GetPrimAttribute.Response

        :return: The service response
        :rtype: GetPrimAttribute.Response
        """
        response.success = False
        stage = self._usd_context.get_stage()
        # get prim
        if stage.GetPrimAtPath(request.path).IsValid():
            prim = stage.GetPrimAtPath(request.path)
            if request.attribute and prim.HasAttribute(request.attribute):
                attribute = prim.GetAttribute(request.attribute)
                response.type = type(attribute.Get()).__name__
                # parse data
                response.success = True
                if response.type in ['Vec2d', 'Vec2f', 'Vec2h', 'Vec2i']:
                    data = attribute.Get()
                    response.value = json.dumps([data[i] for i in range(2)])
                elif response.type in ['Vec3d', 'Vec3f', 'Vec3h', 'Vec3i']:
                    data = attribute.Get()
                    response.value = json.dumps([data[i] for i in range(3)])
                elif response.type in ['Vec4d', 'Vec4f', 'Vec4h', 'Vec4i']:
                    data = attribute.Get()
                    response.value = json.dumps([data[i] for i in range(4)])
                elif response.type in ['Quatd', 'Quatf', 'Quath']:
                    data = attribute.Get()
                    response.value = json.dumps([data.real, data.imaginary[0], data.imaginary[1], data.imaginary[2]])
                elif response.type in ['Matrix4d', 'Matrix4f']:
                    data = attribute.Get()
                    response.value = json.dumps([[data.GetRow(i)[j] for j in range(data.dimension[1])] \
                        for i in range(data.dimension[0])])
                elif response.type.startswith('Vec') and response.type.endswith('Array'):
                    data = attribute.Get()
                    response.value = json.dumps([[d[i] for i in range(len(d))] for d in data])
                elif response.type.startswith('Matrix') and response.type.endswith('Array'):
                    data = attribute.Get()
                    response.value = json.dumps([[[d.GetRow(i)[j] for j in range(d.dimension[1])] \
                        for i in range(d.dimension[0])] for d in data])
                elif response.type.startswith('Quat') and response.type.endswith('Array'):
                    data = attribute.Get()
                    response.value = json.dumps([[d.real, d.imaginary[0], d.imaginary[1], d.imaginary[2]] for d in data])
                elif response.type.endswith('Array'):
                    try:
                        response.value = json.dumps(list(attribute.Get()))
                    except Exception:
                        print("[Warning][semu.robotics.ros2_bridge] RosAttribute: Unknown attribute type {}" \
                            .format(type(attribute.Get())))
                        response.success = False
                        response.message = "Unknown type {}".format(type(attribute.Get()))
                elif response.type in ['AssetPath']:
                    response.value = json.dumps(str(attribute.Get().path))
                else:
                    try:
                        response.value = json.dumps(attribute.Get())
                    except Exception:
                        print("[Warning][semu.robotics.ros2_bridge] RosAttribute: Unknown {}: {}" \
                            .format(type(attribute.Get()), attribute.Get()))
                        response.success = False
                        response.message = "Unknown type {}".format(type(attribute.Get()))
            else:
                response.message = "Prim has not attribute {}".format(request.attribute)
        else:
            response.message = "Invalid prim ({})".format(request.path)
        return response

    def _process_attributes_request(self,
                                    request: 'GetPrimAttributes.Request',
                                    response: 'GetPrimAttributes.Response') -> 'GetPrimAttributes.Response':
        """Process the 'get all attributes' request

        :param request: The service request
        :type request: GetPrimAttributes.Request
        :param response: The service response
        :type response: GetPrimAttributes.Response

        :return: The service response
        :rtype: GetPrimAttributes.Response
        """
        response.success = False
        stage = self._usd_context.get_stage()
        # get prim
        if stage.GetPrimAtPath(request.path).IsValid():
            prim = stage.GetPrimAtPath(request.path)
            for attribute in prim.GetAttributes():
                if attribute.GetNamespace():
                    response.names.append("{}:{}".format(attribute.GetNamespace(), attribute.GetBaseName()))
                else:
                    response.names.append(attribute.GetBaseName())
                response.displays.append(attribute.GetDisplayName())
                response.types.append(type(attribute.Get()).__name__)
                response.success = True
        else:
            response.message = "Invalid prim ({})".format(request.path)
        return response

    def _process_prims_request(self, request: 'GetPrims.Request', response: 'GetPrims.Response') -> 'GetPrims.Response':
        """Process the 'get all prims' request

        :param request: The service request
        :type request: GetPrims.Request
        :param response: The service response
        :type response: GetPrims.Response

        :return: The service response
        :rtype: GetPrims.Response
        """
        response.success = False
        stage = self._usd_context.get_stage()
        # get prims
        if not request.path or stage.GetPrimAtPath(request.path).IsValid():
            path = request.path if request.path else "/"
            for prim in Usd.PrimRange.AllPrims(stage.GetPrimAtPath(path)):
                response.paths.append(str(prim.GetPath()))
                response.types.append(prim.GetTypeName())
                response.success = True
        else:
            response.message = "Invalid search path ({})".format(request.path)
        return response

    def start(self,
              prims_srv: str = "/get_prims",
              get_attribute_srv: str = "/get_prim_attribute",
              attributes_srv: str = "/get_prim_attributes",
              set_attribute_srv: str = "/set_prim_attribute") -> None:
        """Start the services

        :param prims_srv: Service name for GetPrims
        :param get_attribute_srv: Service name for GetPrimAttribute
        :param attributes_srv: Service name for GetPrimAttributes
        :param set_attribute_srv: Service name for SetPrimAttribute
        """
        print("[Info][semu.robotics.ros2_bridge] RosAttribute: starting")

        self._srv_prims = self._node.create_service(GetPrims, prims_srv, self._process_prims_request)
        print("[Info][semu.robotics.ros2_bridge] RosAttribute: register srv: {}".format(self._srv_prims.srv_name))

        self._srv_getter = self._node.create_service(GetPrimAttribute, get_attribute_srv, self._process_getter_request)
        print("[Info][semu.robotics.ros2_bridge] RosAttribute: register srv: {}".format(self._srv_getter.srv_name))

        self._srv_attributes = self._node.create_service(GetPrimAttributes, attributes_srv, self._process_attributes_request)
        print("[Info][semu.robotics.ros2_bridge] RosAttribute: register srv: {}".format(self._srv_attributes.srv_name))

        self._srv_setter = self._node.create_service(SetPrimAttribute, set_attribute_srv, self._process_setter_request)
        print("[Info][semu.robotics.ros2_bridge] RosAttribute: register srv: {}".format(self._srv_setter.srv_name))

        self.started = True

    def stop(self) -> None:
        """Stop the services
        """
        for name in ("_srv_prims", "_srv_getter", "_srv_attributes", "_srv_setter"):
            srv = getattr(self, name)
            if srv is not None:
                print("[Info][semu.robotics.ros2_bridge] RosAttribute: unregister srv: {}".format(srv.srv_name))
                self._node.destroy_service(srv)
                setattr(self, name, None)
        super().stop()

    def step(self, dt: float) -> None:
        """Physics update step: apply the pending attribute value on the simulation thread

        :param dt: The physics delta time
        :type dt: float
        """
        if not self.started:
            return
        if self.__set_attribute_using_asyncio:
            return
        if self._dci.is_simulating():
            if not self._event.is_set():
                if self._attribute is not None:
                    self._attribute.Set(self._value)
                self._event.set()


class RosControlFollowJointTrajectory(RosController):
    # Joints this controller manages. None = publish all articulation joints
    # (legacy behavior). Subclasses override with a list to scope the
    # /controller_state msg to the joints the consumer expects (pumas
    # arm_node segfaults when actual.positions oversized).
    controlled_joints = None

    def __init__(self,
                 node: Node,
                 _dci: 'omni.isaac.dynamic_control.DynamicControl') -> None:
        """FollowJointTrajectory interface
        
        :param node: The ROS node
        :type node: rclpy.node.Node
        :param dci: The dynamic control interface
        :type dci: omni.isaac.dynamic_control.DynamicControl
        """
        super().__init__(node)

        self.dci = _dci

        self._articulation = _dynamic_control.INVALID_HANDLE
        self._joints = {}

        self._action_server = None

        self._action_dt = 0.05
        self._action_goal = None
        self._action_goal_handle = None
        self._action_start_time = None
        self._action_point_index = 1

        # feedback / result
        self._action_result_message = None
        self._action_feedback_message = FollowJointTrajectory.Feedback()

        # publishers for controller state: legacy /<c>/state and the
        # ros2_control standard /<c>/controller_state (pumas's arm_controller
        # / viewpoint_controller subscribe to the latter)
        self._state_pub = None
        self._controller_state_pub = None

        # topic-interface subscription for /<c>/joint_trajectory (mirrors
        # ros2_control's JointTrajectoryController)
        self._traj_sub = None

        # Topic msgs are received on the ROS executor thread; the actual goal
        # installation (which calls DCI for current joint state) is deferred
        # to step() on the main thread to avoid racing with the physics step.
        self._pending_topic_msg = None
        self._pending_lock = threading.Lock()

        # Pad `time_from_start == 0` to this many seconds so single-shot
        # trajectories (e.g. pumas initial-pose publish) have time to
        # physically reach the target before the trajectory ends.
        self._zero_time_pad_sec = 1.0

    def start(self, _articulation_path, _action_topic_name) -> None:
        """Start the action server
        """
        print("[Info][semu.robotics.ros2_bridge] RosControlFollowJointTrajectory: starting")

        # get attributes and relationships
        self.articulation_path = _articulation_path
        self.action_topic_name = _action_topic_name

        # start action server
        self._action_server = ActionServer(self._node,
                                           FollowJointTrajectory,
                                           self.action_topic_name,
                                           execute_callback=self._on_execute,
                                           goal_callback=self._on_goal,
                                           cancel_callback=self._on_cancel,
                                           handle_accepted_callback=self._on_handle_accepted)
        print("[Info][semu.robotics.ros2_bridge] RosControlFollowJointTrajectory: register action {}" \
            .format(self.action_topic_name))

        # Controller state publishers — both topic names so topic-only clients
        # (pumas) and legacy consumers see the same payload.
        state_topic = self.action_topic_name.replace('/follow_joint_trajectory', '/state')
        self._state_pub = self._node.create_publisher(
            JointTrajectoryControllerState,
            state_topic,
            10,
        )
        controller_state_topic = self.action_topic_name.replace('/follow_joint_trajectory', '/controller_state')
        self._controller_state_pub = self._node.create_publisher(
            JointTrajectoryControllerState,
            controller_state_topic,
            10,
        )

        # Topic interface that mirrors ros2_control's JointTrajectoryController
        traj_topic = self.action_topic_name.replace('/follow_joint_trajectory', '/joint_trajectory')
        self._traj_sub = self._node.create_subscription(
            JointTrajectory,
            traj_topic,
            self._on_trajectory_topic,
            10,
        )
        print("[Info][semu.robotics.ros2_bridge] RosControlFollowJointTrajectory: subscribe topic {}" \
            .format(traj_topic))
        print("[Info][semu.robotics.ros2_bridge] RosControlFollowJointTrajectory: publish state on {} and {}" \
            .format(state_topic, controller_state_topic))

        self.started = True

    def stop(self) -> None:
        """Stop the action server
        """
        super().stop()
        self._articulation = _dynamic_control.INVALID_HANDLE
        # destroy action server
        if self._action_server is not None:
            print("[Info][semu.robotics.ros2_bridge] RosControlFollowJointTrajectory: destroy action server: {}" \
                .format(self.action_topic_name))
            # self._action_server.destroy()
            self._action_server = None
        # destroy topic subscription
        if self._traj_sub is not None:
            try:
                self._node.destroy_subscription(self._traj_sub)
            except Exception:
                pass
            self._traj_sub = None
        self._action_goal_handle = None
        self._action_goal = None

    def _duration_to_seconds(self, duration: Duration) -> float:
        """Convert a ROS2 Duration to seconds

        :param duration: The ROS2 Duration
        :type duration: Duration

        :return: The duration in seconds
        :rtype: float
        """
        return Duration.from_msg(duration).nanoseconds / 1e9

    def _init_articulation(self) -> None:
        """Initialize the articulation and register joints
        """
        # get articulation
        self._articulation = self.dci.get_articulation(self.articulation_path)
        if self._articulation == _dynamic_control.INVALID_HANDLE:
            print("[Warning][semu.robotics.ros2_bridge] RosControlFollowJointTrajectory: {} is not an articulation".format(self.articulation_path))
            return

        dof_props = self.dci.get_articulation_dof_properties(self._articulation)
        if dof_props is None:
            return

        upper_limits = dof_props["upper"]
        lower_limits = dof_props["lower"]
        has_limits = dof_props["hasLimits"]

        # get joints
        for i in range(self.dci.get_articulation_dof_count(self._articulation)):
            dof_ptr = self.dci.get_articulation_dof(self._articulation, i)
            if dof_ptr != _dynamic_control.DofType.DOF_NONE:
                dof_name = self.dci.get_dof_name(dof_ptr)
                if dof_name not in self._joints:
                    _joint = self.dci.find_articulation_joint(self._articulation, dof_name)
                    self._joints[dof_name] = {"joint": _joint,
                                              "type": self.dci.get_joint_type(_joint),
                                              "dof": self.dci.find_articulation_dof(self._articulation, dof_name),
                                              "lower": lower_limits[i],
                                              "upper": upper_limits[i],
                                              "has_limits": has_limits[i]}

        if not self._joints:
            print("[Warning][semu.robotics.ros2_bridge] RosControlFollowJointTrajectory: no joints found in {}".format(self.articulation_path))
            self.started = False

    def _set_joint_position(self, name: str, target_position: float) -> None:
        """Set the target position of a joint in the articulation

        :param name: The joint name
        :type name: str
        :param target_position: The target position
        :type target_position: float
        """
        # Planner/filter bugs must never inject NaN into PhysX: one non-finite DOF
        # target invalidates the whole articulation (and all TF derived from it).
        # Goals are validated on receipt; this is the last line of defense.
        if not math.isfinite(float(target_position)):
            key = (name, repr(target_position))
            warned = getattr(self, '_invalid_target_warned', set())
            if key not in warned:
                warned.add(key)
                self._invalid_target_warned = warned
                print("[Warning][semu.robotics.ros2_bridge] {}: ignored non-finite target {}={}"
                      .format(self.action_topic_name, name, target_position), flush=True)
            return
        # clip target position
        if self._joints[name]["has_limits"]:
            target_position = min(max(target_position, self._joints[name]["lower"]), self._joints[name]["upper"])
        # scale target position for prismatic joints
        if self._joints[name]["type"] == _dynamic_control.JOINT_PRISMATIC:
            target_position /= get_stage_units()
        # set target position
        self.dci.set_dof_position_target(self._joints[name]["dof"], target_position)

    def _get_joint_position(self, name: str) -> float:
        """Get the current position of a joint in the articulation

        :param name: The joint name
        :type name: str

        :return: The current position of the joint
        :rtype: float
        """
        position = self.dci.get_dof_state(self._joints[name]["dof"], _dynamic_control.STATE_POS).pos
        if self._joints[name]["type"] == _dynamic_control.JOINT_PRISMATIC:
            return position * get_stage_units()
        return position

    def _on_handle_accepted(self, goal_handle: 'rclpy.action.server.ServerGoalHandle') -> None:
        """Callback function for handling newly accepted goals

        :param goal_handle: The goal handle
        :type goal_handle: rclpy.action.server.ServerGoalHandle
        """
        goal_handle.execute()

    def _on_goal(self, goal: 'FollowJointTrajectory.Goal') -> 'rclpy.action.server.GoalResponse':
        """Callback function for handling new goal requests

        :param goal: The goal
        :type goal: FollowJointTrajectory.Goal

        :return: Whether the goal was accepted
        :rtype: rclpy.action.server.GoalResponse
        """
        # reject if joints don't match
        for name in goal.trajectory.joint_names:
            if name not in self._joints:
                print("[Warning][semu.robotics.ros2_bridge] RosControlFollowJointTrajectory: joints don't match ({} not in {})" \
                    .format(name, list(self._joints.keys())))
                return GoalResponse.REJECT

        # reject malformed trajectories (empty, short point, NaN/inf positions);
        # NaN/inf velocities are replaced by 0 (they are only used for feedback)
        joint_count = len(goal.trajectory.joint_names)
        if not goal.trajectory.points:
            print("[Warning][semu.robotics.ros2_bridge] {}: rejected empty trajectory"
                  .format(self.action_topic_name), flush=True)
            return GoalResponse.REJECT
        for point_index, point in enumerate(goal.trajectory.points):
            if len(point.positions) < joint_count:
                print("[Warning][semu.robotics.ros2_bridge] {}: rejected short positions at point {} ({} < {})"
                      .format(self.action_topic_name, point_index, len(point.positions), joint_count), flush=True)
                return GoalResponse.REJECT
            invalid = [(goal.trajectory.joint_names[i], point.positions[i])
                       for i in range(joint_count) if not math.isfinite(float(point.positions[i]))]
            if invalid:
                print("[Warning][semu.robotics.ros2_bridge] {}: rejected non-finite positions at point {}: {}"
                      .format(self.action_topic_name, point_index, invalid), flush=True)
                return GoalResponse.REJECT
            if point.velocities:
                sanitized = [float(v) if math.isfinite(float(v)) else 0.0 for v in point.velocities]
                if sanitized != list(point.velocities):
                    point.velocities = sanitized
                    print("[Warning][semu.robotics.ros2_bridge] {}: replaced non-finite velocities with 0 at point {}"
                          .format(self.action_topic_name, point_index), flush=True)

        # Preempt any in-flight goal so streaming clients can replace the
        # active trajectory. The previous _on_execute loop polls _action_goal
        # and exits with INVALID_GOAL once we clear it.
        if self._action_goal is not None:
            old_handle = self._action_goal_handle
            self._action_goal = None
            if old_handle is not None and old_handle.is_active:
                try:
                    old_handle.abort()
                except Exception as exc:
                    print("[Warning][semu.robotics.ros2_bridge] RosControlFollowJointTrajectory: abort failed: {}" \
                        .format(exc))
            self._action_goal_handle = None

        # Pad zero time_from_start so single-point trajectories become a
        # 2-point interpolation with enough physical time to converge.
        if goal.trajectory.points:
            first_tfs = goal.trajectory.points[0].time_from_start
            if first_tfs.sec == 0 and first_tfs.nanosec == 0:
                pad_total_ns = int(self._zero_time_pad_sec * 1e9)
                goal.trajectory.points[0].time_from_start.sec = pad_total_ns // 1_000_000_000
                goal.trajectory.points[0].time_from_start.nanosec = pad_total_ns % 1_000_000_000

        # check initial position
        if goal.trajectory.points[0].time_from_start:
            initial_point = JointTrajectoryPoint(
                positions=[
                    self._get_joint_position(name)
                    for name in goal.trajectory.joint_names
                ],
                time_from_start=Duration().to_msg(),
            )
            goal.trajectory.points.insert(0, initial_point)

        # reset internal data
        self._action_goal_handle = None
        self._action_start_time = None
        self._action_result_message = None
        self._action_point_index = 1

        # store goal data
        self._action_goal = goal

        return GoalResponse.ACCEPT

    def _on_trajectory_topic(self, msg: JointTrajectory) -> None:
        """Topic-interface entry point.

        Runs on the ROS executor thread. To avoid racing with the physics
        simulation on DCI calls, this method only validates the message and
        stashes it. step() picks it up on the main thread.
        """
        for name in msg.joint_names:
            if name not in self._joints:
                print("[Warning][semu.robotics.ros2_bridge] RosControlFollowJointTrajectory: topic msg joints don't match ({} not in {})" \
                    .format(name, list(self._joints.keys())))
                return
        if not msg.points:
            return
        joint_count = len(msg.joint_names)
        for point_index, point in enumerate(msg.points):
            if len(point.positions) < joint_count or any(
                    not math.isfinite(float(v)) for v in point.positions[:joint_count]):
                print("[Warning][semu.robotics.ros2_bridge] {}: ignored invalid topic trajectory point {}"
                      .format(self.action_topic_name, point_index), flush=True)
                return
            if point.velocities:
                point.velocities = [float(v) if math.isfinite(float(v)) else 0.0 for v in point.velocities]
        with self._pending_lock:
            self._pending_topic_msg = msg

    def _install_pending_trajectory(self) -> None:
        """Promote a pending topic msg to the active trajectory (main thread only)."""
        with self._pending_lock:
            msg = self._pending_topic_msg
            self._pending_topic_msg = None
        if msg is None:
            return

        # Preempt any in-flight trajectory.
        if self._action_goal is not None:
            self._action_goal = None
            self._action_goal_handle = None

        # Single-point trajectories are setpoint commands ("go to this pose").
        # Bypass the interpolator entirely and write joint targets directly —
        # the DCI position controller handles physical convergence. Running a
        # state-machine interpolation per msg means each new streaming goal
        # preempts the previous one mid-flight (pumas head_node at 10 Hz with
        # 1 s pad → ~10% progress per cycle; pumas arm_node with 0.2 s →
        # ~50% per cycle), which makes the joint barely track the stream.
        # time_from_start on a single point is only a deadline hint, not a
        # forced interpolation duration.
        if len(msg.points) == 1:
            self.dci.wake_up_articulation(self._articulation)
            point = msg.points[0]
            for i, name in enumerate(msg.joint_names):
                if i < len(point.positions):
                    self._set_joint_position(name, point.positions[i])
            return

        # Multi-point / non-zero trajectory: run the interpolator.
        # Insert current state as t=0 anchor for interpolation.
        initial_point = JointTrajectoryPoint(
            positions=[
                self._get_joint_position(name) for name in msg.joint_names
            ],
            time_from_start=Duration().to_msg(),
        )
        msg.points.insert(0, initial_point)

        goal = FollowJointTrajectory.Goal()
        goal.trajectory = msg
        self._action_start_time = self._node.get_clock().now().nanoseconds / 1e9
        self._action_result_message = None
        self._action_point_index = 1
        self._action_goal = goal
        # _action_goal_handle stays None -> step() treats this as topic mode

    def _on_cancel(self, goal_handle: 'rclpy.action.server.ServerGoalHandle') -> 'rclpy.action.server.CancelResponse':
        """Callback function for handling cancel requests

        :param goal_handle: The goal handle
        :type goal_handle: rclpy.action.server.ServerGoalHandle

        :return: Whether the goal was canceled
        :rtype: rclpy.action.server.CancelResponse
        """
        if self._action_goal is None:
            return CancelResponse.REJECT
        # reset internal data
        self._action_goal = None
        self._action_goal_handle = None
        self._action_start_time = None
        self._action_result_message = None
        # Leave the ServerGoalHandle to rclpy's ActionServer. Destroying it here
        # makes the result_timeout expiry destroy it a second time (KeyError in
        # the executor), which kills every action server on this node.
        return CancelResponse.ACCEPT

    def _on_execute(self, goal_handle: 'rclpy.action.server.ServerGoalHandle') -> 'FollowJointTrajectory.Result':
        """Callback function for processing accepted goals

        :param goal_handle: The goal handle
        :type goal_handle: rclpy.action.server.ServerGoalHandle

        :return: The result of the goal execution
        :rtype: FollowJointTrajectory.Result
        """
        # reset internal data
        self._action_start_time = self._node.get_clock().now().nanoseconds / 1e9
        self._action_result_message = None
        # set goal
        self._action_goal_handle = goal_handle
        # wait for the goal to be executed
        while self._action_result_message is None: 
            if self._action_goal is None:
                result = FollowJointTrajectory.Result()
                result.error_code = result.INVALID_GOAL
                return result
            time.sleep(self._action_dt)
        self._action_goal = None
        self._action_goal_handle = None
        return self._action_result_message

    def step(self, dt: float) -> None:
        """Physics update step

        :param dt: The physics delta time
        :type dt: float
        """
        if not self.started:
            return
        # init articulation
        if not self._joints:
            self._init_articulation()
            return
        # Promote any pending topic-mode trajectory (received on the ROS
        # executor thread) so the DCI call for current joint state happens
        # here on the main thread.
        if self._pending_topic_msg is not None:
            self._install_pending_trajectory()
        # update articulation if a trajectory is active (action or topic mode)
        if self._action_goal is not None:
            self._action_dt = dt
            # end of trajectory
            if self._action_point_index >= len(self._action_goal.trajectory.points):
                handle = self._action_goal_handle
                self._action_goal = None
                # Set the terminal state BEFORE publishing the result message.
                # _on_execute (executor thread) returns as soon as
                # _action_result_message is set; if the handle has no terminal
                # state yet rclpy aborts it, racing with our succeed() -> "invalid
                # transition" exception -> the ROS thread dies and every action
                # server stops responding.
                result = FollowJointTrajectory.Result()
                result.error_code = result.SUCCESSFUL
                if handle is not None:
                    if handle.is_active:
                        try:
                            handle.succeed()
                        except Exception:
                            pass
                    self._action_goal_handle = None
                self._action_result_message = result
                return

            previous_point = self._action_goal.trajectory.points[self._action_point_index - 1]
            current_point = self._action_goal.trajectory.points[self._action_point_index]
            if self._action_start_time is None:
                # 開始時刻未設定のまま物理ステップが先行するレースのガード
                self._action_start_time = self._node.get_clock().now().nanoseconds / 1e9
            time_passed = self._node.get_clock().now().nanoseconds / 1e9 - self._action_start_time

            # set target using linear interpolation
            if time_passed <= self._duration_to_seconds(current_point.time_from_start):
                ratio = (time_passed - self._duration_to_seconds(previous_point.time_from_start)) \
                      / (self._duration_to_seconds(current_point.time_from_start) \
                          - self._duration_to_seconds(previous_point.time_from_start))
                self.dci.wake_up_articulation(self._articulation)
                for i, name in enumerate(self._action_goal.trajectory.joint_names):
                    side = -1 if current_point.positions[i] < previous_point.positions[i] else 1
                    target_position = previous_point.positions[i] \
                                    + side * ratio * abs(current_point.positions[i] - previous_point.positions[i])
                    self._set_joint_position(name, target_position)
            # send feedback
            else:
                self._action_point_index += 1
                # set joint targets for the new current point when advancing the index
                if self._action_point_index < len(self._action_goal.trajectory.points):
                    new_point = self._action_goal.trajectory.points[self._action_point_index]
                    self.dci.wake_up_articulation(self._articulation)
                    for i, name in enumerate(self._action_goal.trajectory.joint_names):
                        if i < len(new_point.positions):
                            target_position = new_point.positions[i]
                            self._set_joint_position(name, target_position)
                # feedback only when an action handle is attached
                if self._action_goal_handle is not None:
                    self._action_feedback_message.joint_names = list(self._action_goal.trajectory.joint_names)
                    self._action_feedback_message.actual.positions = [self._get_joint_position(name) \
                        for name in self._action_goal.trajectory.joint_names]
                    self._action_feedback_message.actual.time_from_start = Duration(seconds=time_passed).to_msg()
                    try:
                        self._action_goal_handle.publish_feedback(self._action_feedback_message)
                    except Exception:
                        pass
        # publish controller state for this timestep
        if self._state_pub is not None:
            msg = JointTrajectoryControllerState()
            msg.header.stamp = self._node.get_clock().now().to_msg()

            # Scope joint_names to this controller's joints when declared;
            # otherwise fall back to all articulation joints. Skip names that
            # aren't registered in self._joints (defensive).
            if self.controlled_joints:
                joint_names = [n for n in self.controlled_joints if n in self._joints]
            else:
                joint_names = sorted(self._joints.keys())
            msg.joint_names = joint_names

            # actual state from articulation
            actual_positions = []
            actual_velocities = []
            for name in joint_names:
                try:
                    pos = self._get_joint_position(name)
                except Exception:
                    pos = 0.0
                actual_positions.append(pos)
                try:
                    vel = self.dci.get_dof_state(
                        self._joints[name]["dof"],
                        _dynamic_control.STATE_VEL,
                    ).vel
                except Exception:
                    vel = 0.0
                actual_velocities.append(vel)

            msg.actual.positions = list(actual_positions)
            msg.actual.velocities = list(actual_velocities)

            # desired state from current trajectory point if a goal is active
            desired_positions = list(actual_positions)
            desired_velocities = [0.0] * len(joint_names)

            if self._action_goal is not None and self._action_goal.trajectory.points:
                # clamp index in case it's already at/after the end
                point_index = min(self._action_point_index, len(self._action_goal.trajectory.points) - 1)
                current_point = self._action_goal.trajectory.points[point_index]

                # map trajectory joint order to controller joint order
                for i, name in enumerate(self._action_goal.trajectory.joint_names):
                    if name in self._joints and name in joint_names:
                        j_idx = joint_names.index(name)
                        if i < len(current_point.positions):
                            desired_positions[j_idx] = current_point.positions[i]
                        if i < len(current_point.velocities):
                            desired_velocities[j_idx] = current_point.velocities[i]

            msg.desired.positions = desired_positions
            msg.desired.velocities = desired_velocities

            # error = desired - actual
            error_positions = [
                dp - ap for dp, ap in zip(desired_positions, actual_positions)
            ]
            error_velocities = [
                dv - av for dv, av in zip(desired_velocities, actual_velocities)
            ]
            msg.error.positions = error_positions
            msg.error.velocities = error_velocities

            # ROS 2 control_msgs/JointTrajectoryControllerState has BOTH the
            # legacy fields (actual/desired/error) and the new ones
            # (feedback/reference/error/output). pumas's arm_node reads
            # `feedback.positions[i]` without a size check and segfaults if
            # left empty — mirror the legacy data into the new fields.
            msg.feedback.positions = list(actual_positions)
            msg.feedback.velocities = list(actual_velocities)
            msg.reference.positions = list(desired_positions)
            msg.reference.velocities = list(desired_velocities)
            msg.output.positions = list(desired_positions)
            msg.output.velocities = list(desired_velocities)

            self._state_pub.publish(msg)
            if self._controller_state_pub is not None:
                self._controller_state_pub.publish(msg)

class RosControllerGripperCommand(RosController):
    def __init__(self,
                 node: Node,
                 _dci: 'omni.isaac.dynamic_control.DynamicControl') -> None:
        """GripperCommand interface

        :param node: The ROS node
        :type node: rclpy.node.Node
        :param dci: The dynamic control interface
        :type dci: omni.isaac.dynamic_control.DynamicControl
        """
        super().__init__(node)
        
        self.dci = _dci

        self._articulation = _dynamic_control.INVALID_HANDLE
        self._joints = {}

        self._action_server = None

        self._action_dt = 0.05
        self._action_goal = None
        self._action_goal_handle = None
        self._action_start_time = None
        # TODO: add to schema?
        self._action_timeout = 10.0
        self._action_position_threshold = 0.001
        self._action_previous_position_sum = float("inf")

        # feedback / result
        self._action_result_message = None
        self._action_feedback_message = GripperCommand.Feedback()
        self._action_type = GripperCommand  # サブクラスが実機の型に上書き可能

    def start(self, _articulation_path, _action_topic_name) -> None:
        """Start the action server
        """
        print("[Info][semu.robotics.ros2_bridge] RosControllerGripperCommand: starting {}" \
            .format(_action_topic_name))

        # get attributes and relationships
        self.articulation_path = _articulation_path
        self.action_topic_name = _action_topic_name

        # start action server
        self._action_server = ActionServer(self._node,
                                           self._action_type,
                                           self.action_topic_name,
                                           execute_callback=self._on_execute,
                                           goal_callback=self._on_goal,
                                           cancel_callback=self._on_cancel,
                                           handle_accepted_callback=self._on_handle_accepted)
        print("[Info][semu.robotics.ros2_bridge] RosControllerGripperCommand: register action {}" \
            .format(self.action_topic_name))

        self.started = True

    def stop(self) -> None:
        """Stop the action server
        """
        super().stop()
        self._articulation = _dynamic_control.INVALID_HANDLE
        # destroy action server
        if self._action_server is not None:
            print("[Info][semu.robotics.ros2_bridge] RosControllerGripperCommand: destroy action server")
            # self._action_server.destroy()
            self._action_server = None
        self._action_goal_handle = None
        self._action_goal = None

    def _duration_to_seconds(self, duration: Duration) -> float:
        """Convert a ROS2 Duration to seconds

        :param duration: The ROS2 Duration
        :type duration: Duration

        :return: The duration in seconds
        :rtype: float
        """
        return Duration.from_msg(duration).nanoseconds / 1e9

    def _init_articulation(self) -> None:
        """Initialize the articulation and register joints
        """
        # get articulation
        self._articulation = self.dci.get_articulation(self.articulation_path)
        if self._articulation == _dynamic_control.INVALID_HANDLE:
            print("[Warning][semu.robotics.ros2_bridge] RosControllerGripperCommand: {} is not an articulation".format(self.articulation_path))
            return

        dof_props = self.dci.get_articulation_dof_properties(self._articulation)
        if dof_props is None:
            return

        upper_limits = dof_props["upper"]
        lower_limits = dof_props["lower"]
        has_limits = dof_props["hasLimits"]

        # get joints
        for i in range(self.dci.get_articulation_dof_count(self._articulation)):
            dof_ptr = self.dci.get_articulation_dof(self._articulation, i)
            if dof_ptr != _dynamic_control.DofType.DOF_NONE:
                dof_name = self.dci.get_dof_name(dof_ptr)
                if dof_name not in self._joints:
                    _joint = self.dci.find_articulation_joint(self._articulation, dof_name)
                    self._joints[dof_name] = {"joint": _joint,
                                                "type": self.dci.get_joint_type(_joint),
                                                "dof": self.dci.find_articulation_dof(self._articulation, dof_name),
                                                "lower": lower_limits[i],
                                                "upper": upper_limits[i],
                                                "has_limits": has_limits[i]}

        if not self._joints:
            print("[Warning][semu.robotics.ros2_bridge] RosControllerGripperCommand: no joints found in {}".format(self.articulation_path))
            self.started = False

    def _set_joint_position(self, name: str, target_position: float) -> None:
        """Set the target position of a joint in the articulation

        :param name: The joint name
        :type name: str
        :param target_position: The target position
        :type target_position: float
        """
        # Planner/filter bugs must never inject NaN into PhysX: one non-finite DOF
        # target invalidates the whole articulation (and all TF derived from it).
        # Goals are validated on receipt; this is the last line of defense.
        if not math.isfinite(float(target_position)):
            key = (name, repr(target_position))
            warned = getattr(self, '_invalid_target_warned', set())
            if key not in warned:
                warned.add(key)
                self._invalid_target_warned = warned
                print("[Warning][semu.robotics.ros2_bridge] {}: ignored non-finite target {}={}"
                      .format(self.action_topic_name, name, target_position), flush=True)
            return
        # clip target position
        if self._joints[name]["has_limits"]:
            target_position = min(max(target_position, self._joints[name]["lower"]), self._joints[name]["upper"])
        # scale target position for prismatic joints
        if self._joints[name]["type"] == _dynamic_control.JOINT_PRISMATIC:
            target_position /= get_stage_units()
        # set target position
        self.dci.set_dof_position_target(self._joints[name]["dof"], target_position)

    def _get_joint_position(self, name: str) -> float:
        """Get the current position of a joint in the articulation

        :param name: The joint name
        :type name: str

        :return: The current position of the joint
        :rtype: float
        """
        position = self.dci.get_dof_state(self._joints[name]["dof"], _dynamic_control.STATE_POS).pos
        if self._joints[name]["type"] == _dynamic_control.JOINT_PRISMATIC:
            return position * get_stage_units()
        return position

    def _on_handle_accepted(self, goal_handle: 'rclpy.action.server.ServerGoalHandle') -> None:
        """Callback function for handling newly accepted goals

        :param goal_handle: The goal handle
        :type goal_handle: rclpy.action.server.ServerGoalHandle
        """
        goal_handle.execute()

    def _on_goal(self, goal: 'GripperCommand.Goal') -> 'rclpy.action.server.GoalResponse':
        """Callback function for handling new goal requests

        :param goal: The goal
        :type goal: GripperCommand.Goal

        :return: Whether the goal was accepted
        :rtype: rclpy.action.server.GoalResponse
        """
        # 以前のゴールが残っていても reject せず preempt (最新コマンド優先)
        if self._action_goal is not None:
            print("[Info][semu.robotics.ros2_bridge] RosControllerGripperCommand: preempting previous goal")

        # reset internal data
        self._action_goal = None
        self._action_goal_handle = None
        self._action_start_time = None
        self._action_result_message = None
        self._action_previous_position_sum = float("inf")

        return GoalResponse.ACCEPT

    def _on_cancel(self, goal_handle: 'rclpy.action.server.ServerGoalHandle') -> 'rclpy.action.server.CancelResponse':
        """Callback function for handling cancel requests

        :param goal_handle: The goal handle
        :type goal_handle: rclpy.action.server.ServerGoalHandle

        :return: Whether the goal was canceled
        :rtype: rclpy.action.server.CancelResponse
        """
        if self._action_goal is None:
            return CancelResponse.REJECT
        # reset internal data
        self._action_goal = None
        self._action_goal_handle = None
        self._action_start_time = None
        self._action_result_message = None
        self._action_previous_position_sum = float("inf")
        # see RosControlFollowJointTrajectory._on_cancel: rclpy owns the handle
        return CancelResponse.ACCEPT

    def _on_execute(self, goal_handle: 'rclpy.action.server.ServerGoalHandle') -> 'GripperCommand.Result':
        """Callback function for processing accepted goals

        :param goal_handle: The goal handle
        :type goal_handle: rclpy.action.server.ServerGoalHandle

        :return: The result of the goal execution
        :rtype: GripperCommand.Result
        """
        # reset internal data
        self._action_start_time = self._node.get_clock().now().nanoseconds / 1e9
        self._action_result_message = None
        self._action_previous_position_sum = float("inf")
        # set goal
        self._action_goal_handle = goal_handle
        self._action_goal = goal_handle.request
        # wait for the goal to be executed
        while self._action_result_message is None:
            if self._action_goal is None:
                # サーバのアクション型に合った Result を返す
                return self._action_type.Result()
            time.sleep(self._action_dt)
        self._action_goal = None
        self._action_goal_handle = None
        return self._action_result_message

    def _finish(self, result: 'GripperCommand.Result', succeeded: bool) -> None:
        """Terminate the active goal: terminal state first, result message last.

        _on_execute (executor thread) returns as soon as _action_result_message
        is set. If the handle has no terminal state at that moment rclpy aborts
        it, racing with our own succeed()/abort() ("invalid transition" -> the
        ROS thread dies). So the handle is transitioned before the result is
        published.
        """
        handle = self._action_goal_handle
        if handle is not None:
            if handle.is_active:
                try:
                    if succeeded:
                        handle.succeed()
                    else:
                        handle.abort()
                except Exception:
                    pass
            self._action_goal_handle = None
        self._action_result_message = result

    def step(self, dt: float) -> None:
        """Physics update step

        :param dt: The physics delta time
        :type dt: float
        """
        if not self.started:
            return
        # init articulation
        if not self._joints:
            self._init_articulation()
            return
        # update articulation
        if self._action_goal is not None and self._action_goal_handle is not None:
            self._action_dt = dt
            target_position = self._action_goal.command.position
            # set target
            self.dci.wake_up_articulation(self._articulation)
            for name in self._joints:
                self._set_joint_position(name, target_position)
            # end (position reached)
            position = 0
            current_position_sum = 0
            position_reached = True
            for name in self._joints:
                position = self._get_joint_position(name)
                current_position_sum += position
                if abs(position - target_position) > self._action_position_threshold:
                    position_reached = False
                    break
            if position_reached:
                # terminal state first, result message last (see
                # RosControlFollowJointTrajectory.step for the reason)
                result = GripperCommand.Result()
                result.position = position
                result.stalled = False
                result.reached_goal = True
                self._finish(result, succeeded=True)
                return
            # end (stalled)
            if abs(current_position_sum - self._action_previous_position_sum) < 1e-6:
                result = GripperCommand.Result()
                result.position = position
                result.stalled = True
                result.reached_goal = False
                self._finish(result, succeeded=True)
                return
            self._action_previous_position_sum = current_position_sum
            # end (timeout)
            time_passed = self._node.get_clock().now().nanoseconds / 1e9 - self._action_start_time
            if time_passed >= self._action_timeout:
                self._finish(GripperCommand.Result(), succeeded=False)
            # TODO: send feedback
            # self._action_goal_handle.publish_feedback(self._action_feedback_message)
