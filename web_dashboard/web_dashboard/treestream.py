import copy
import math
import os
import threading
import time
import typing

import py_trees_ros_interfaces.msg as py_trees_msgs
import py_trees_ros_interfaces.srv as py_trees_srvs
import rcl_interfaces.msg as rcl_msgs
import rcl_interfaces.srv as rcl_srvs

import rclpy
import rclpy.node

from . import console
from . import conversions
from . import exceptions
from . import utilities


class SnapshotStream(object):
    class Parameters(object):
        def __init__(
            self,
            blackboard_data: bool = False,
            blackboard_activity: bool = False,
            snapshot_period: float = math.inf
        ):
            self.blackboard_data = blackboard_data
            self.blackboard_activity = blackboard_activity
            self.snapshot_period = snapshot_period

        def __eq__(self, other):
            return ((self.blackboard_data == other.blackboard_data) and
                    (self.blackboard_activity == other.blackboard_activity) and
                    (self.snapshot_period == other.snapshot_period)
                    )

    def __init__(
        self,
        node: rclpy.node.Node,
        namespace: str,
        parameters: 'SnapshotStream.Parameters',
        callback: typing.Callable[[py_trees_msgs.BehaviourTree], None],
    ):
        self.namespace = namespace
        self.parameters = copy.copy(parameters) if parameters is not None else SnapshotStream.Parameters()
        self.node = node
        self.callback = callback

        self.topic_name = None
        self.subscriber = None

        self.services = {
            'open': None,
            'close': None,
            'reconfigure': None
        }

        self.service_names = {
            'open': self.namespace + "/open",
            'close': self.namespace + "/close",
            'reconfigure': self.namespace + "/reconfigure",
        }
        self.service_type_strings = {
            'open': 'py_trees_ros_interfaces/srv/OpenSnapshotStream',
            'close': 'py_trees_ros_interfaces/srv/CloseSnapshotStream',
            'reconfigure': 'py_trees_ros_interfaces/srv/ReconfigureSnapshotStream'
        }
        self.service_types = {
            'open': py_trees_srvs.OpenSnapshotStream,
            'close': py_trees_srvs.CloseSnapshotStream,
            'reconfigure': py_trees_srvs.ReconfigureSnapshotStream
        }
        # create service clients
        self.services["open"] = self.create_service_client(key="open")
        self.services["close"] = self.create_service_client(key="close")
        self.services["reconfigure"] = self.create_service_client(key="reconfigure")

        # create connection
        self._connect_on_init()

    def reconfigure(self, parameters: 'SnapshotStream.Parameters'):
        if self.parameters == parameters:
            return
        self.parameters = copy.copy(parameters)
        request = self.service_types["reconfigure"].Request()
        request.topic_name = self.topic_name
        request.parameters.blackboard_data = self.parameters.blackboard_data
        request.parameters.blackboard_activity = self.parameters.blackboard_activity
        request.parameters.snapshot_period = self.parameters.snapshot_period
        unused_future = self.services["reconfigure"].call_async(request)

    def _connect_on_init(self, timeout_sec=1.0):

        # request a stream
        request = self.service_types["open"].Request()
        request.parameters.blackboard_data = self.parameters.blackboard_data
        request.parameters.blackboard_activity = self.parameters.blackboard_activity
        request.parameters.snapshot_period = self.parameters.snapshot_period
        self.node.get_logger().debug("establishing a snapshot stream connection [{}][backend]".format(self.namespace))

        self.node.get_logger().debug(f'Reqeusting: {request} with {str(self.services["open"])}')
        future = self.services["open"].call_async(request)
        future.add_done_callback(self.handle_response)
        # rclpy.spin_until_future_complete(self.node, future)
        # response = future.result()

    def handle_response(self, future):
        # response = self.services["open"].call(request)
        response = future.result()
        self.topic_name = response.topic_name

        # connect to a snapshot stream
        self.node.get_logger().debug(f'got Response: {response}')
        start_time = time.monotonic()
        while True:
            elapsed_time = time.monotonic() - start_time
            if elapsed_time > 1.0:
                raise exceptions.TimedOutError("timed out waiting for a snapshot stream publisher [{}]".format(self.topic_name))
            if self.node.count_publishers(self.topic_name) > 0:
                break
            time.sleep(0.1)
        self.subscriber = self.node.create_subscription(
            msg_type=py_trees_msgs.BehaviourTree,
            topic=self.topic_name,
            callback=self.callback,
            qos_profile=utilities.qos_profile_latched()
        )
        self.node.get_logger().debug("  ...ok [backend]")

    def shutdown(self):
        if rclpy.ok() and self.services["close"] is not None:
            request = self.service_types["close"].Request()
            request.topic_name = self.topic_name
            future = self.services["close"].call_async(request)
            rclpy.spin_until_future_complete(
                node=self.node,
                future=future,
                timeout_sec=0.5)
            unused_response = future.result()

    def create_service_client(self, key: str):

        if self.service_names[key] is None:
            raise exceptions.NotReadyError(
                "no known '{}' service known [did you call setup()?]".format(self.service_types[key])
            )
        client = self.node.create_client(
            srv_type=self.service_types[key],
            srv_name=self.service_names[key],
            qos_profile=rclpy.qos.qos_profile_services_default
        )
        # hardcoding timeouts will get us into trouble
        if not client.wait_for_service(timeout_sec=3.0):
            raise exceptions.TimedOutError(
                "timed out waiting for {}".format(self.service_names['close'])
            )
        return client


class Backend():

    def __init__(self, parameters, tree_snapshot_callback):
        super().__init__()
        default_node_name = "tree_viewer_" + str(os.getpid())
        self.node = rclpy.create_node(default_node_name)
        self.shutdown_requested = False
        self.snapshot_stream_type = py_trees_msgs.BehaviourTree
        self.discovered_namespaces = []
        self.discovered_timestamp = time.monotonic()
        self.discovery_loop_time_sec = 3.0
        self.cached_blackboard = {"behaviours": {}, "data": {}}
        self.snapshot_stream = None
        self.parameters = parameters

        self.tree_snapshot_callback = tree_snapshot_callback

        self.tree = None

        self.lock = threading.Lock()
        self.enqueued_connection_request_namespace = None
        self.node.get_logger().debug('Init of Backend finished')

    def start_spin(self):
        spin_thread = threading.Thread(target=self.spin)
        spin_thread.start()

    def spin(self):
        self.node.get_logger().debug('Entered Spin')

        with self.lock:
            old_parameters = copy.copy(self.parameters)
        while rclpy.ok() and not self.shutdown_requested:
            self.discover_namespaces()
            with self.lock:
                self.node.get_logger().debug(f'Entered Spin, {self.parameters != old_parameters}, {self.parameters}, {old_parameters}', once=True)
                if self.parameters != old_parameters:
                    if self.snapshot_stream is not None:
                        self.snapshot_stream.reconfigure(self.parameters)
                old_parameters = copy.copy(self.parameters)

                self.node.get_logger().debug(f'Entered Spin, {self.enqueued_connection_request_namespace is not None}, {self.enqueued_connection_request_namespace}', once=True)
                if self.enqueued_connection_request_namespace is not None:
                    self.node.get_logger().debug('Enterd if')
                    self.connect(self.enqueued_connection_request_namespace)
                    self.enqueued_connection_request_namespace = None
            rclpy.spin_once(self.node, timeout_sec=0.1)
        if self.snapshot_stream is not None:
            self.snapshot_stream.shutdown()
        self.node.destroy_node()

    def terminate_ros_spinner(self):
        self.node.get_logger().info("shutdown requested [backend]")
        self.shutdown_requested = True

    def discover_namespaces(self):
        """
        Oneshot lookup for namespaces within which snapshot stream services exist.
        This is additionally conditioned on 'discovery_loop_time_sec' so that it
        doesn't spam the check at the same rate as the node is spinning.

        If a change in the result occurs, it emits a signal for the qt ui.
        """
        self.node.get_logger().debug('Entered discover namespace', once=True)
        timeout = self.discovered_timestamp + self.discovery_loop_time_sec
        if self.discovered_namespaces and (time.monotonic() < timeout):
            return
        open_service_type_string = "py_trees_ros_interfaces/srv/OpenSnapshotStream"
        service_names_and_types = self.node.get_service_names_and_types()
        new_service_names = [name for name, types in service_names_and_types if open_service_type_string in types]
        new_service_names.sort()
        new_namespaces = [utilities.parent_namespace(name) for name in new_service_names]
        self.node.get_logger().debug(f'discover namespace: new:{new_namespaces} old: {self.discovered_namespaces}', once=True)
        if self.discovered_namespaces != new_namespaces:
            self.discovered_namespaces = new_namespaces
            self.node.get_logger().debug(f'discover new namespace: {new_namespaces}', once=False)
            self.node.get_logger().debug("discovered namespaces changed {}[backend]".format(self.discovered_namespaces))
            # Hack to skip some stuff
            self.enqueued_connection_request_namespace = self.discovered_namespaces[0]

        self.discovered_timestamp = time.monotonic()

    def connect(self, namespace):
        """
        Cancel the current connection and create a new one to the specified namespace.

        Args:
            namespace: in which to find snapshot stream services
        """
        self.node.get_logger().debug('Entered Connect', once=True)
        if self.snapshot_stream is not None:
            self.node.get_logger().debug("cancelling existing snapshot stream connection [{}][backend]".format(self.snapshot_stream_watcher))
            self.snapshot_stream.shutdown()
            self.snapshot_stream = None
        self.node.get_logger().debug("creating a new snapshot stream connection [{}][backend]".format(namespace))
        self.snapshot_stream = SnapshotStream(
            node=self.node,
            namespace=namespace,
            callback=self.tree_snapshot_handler,
            parameters=self.parameters
        )

    def snapshot_blackboard_data(self, snapshot: bool):
        if self.parameter_client is not None:
            request = rcl_srvs.SetParameters.Request()  # noqa
            parameter = rcl_msgs.Parameter()
            parameter.name = "snapshot_blackboard_data"
            parameter.value.type = rcl_msgs.ParameterType.PARAMETER_BOOL  # noqa
            parameter.value.bool_value = snapshot
            request.parameters.append(parameter)
            unused_future = self.parameter_client.call_async(request)
        self.parameters.snapshot_blackboard_data = snapshot

    def tree_snapshot_handler(self, msg: py_trees_msgs.BehaviourTree):
        """
        Callback to receive incoming tree snapshots before relaying them to the web application.

        Args:
            msg: incoming serialised tree snapshot

        Note: this uses a clever(?) hack to accumulate visited path snapshots of the blackboard
        to gain a representation of the entire blackboard without having to transmit the
        entire blackboard on every update. Special care is needed to make sure what has been
        removed from the blackboard (does not get transmitted), actually gets removed.
        """
        self.node.get_logger().debug("handling incoming tree snapshot [backend]", once=True)
        colours = {
            'Sequence': '#FFA500',
            'Selector': '#00FFFF',
            'Parallel': '#FFFF00',
            'Behaviour': '#555555',
            'Decorator': '#DDDDDD',
        }
        tree = {
            'changed': "true" if msg.changed else "false",
            'timestamp': msg.statistics.stamp.sec + float(msg.statistics.stamp.nanosec) / 1.0e9,
            'behaviours': {},
            'blackboard': {'behaviours': {}, 'data': {}},
            'visited_path': []
        }
        # hack, update the blackboard from visited path contexts
        blackboard_variables = {}
        for blackboard_variable in msg.blackboard_on_visited_path:
            blackboard_variables[blackboard_variable.key] = blackboard_variable.value
        for behaviour in msg.behaviours:
            behaviour_id = str(conversions.msg_to_uuid4(behaviour.own_id))
            behaviour_type = conversions.msg_constant_to_behaviour_str(behaviour.type)
            if behaviour.is_active:
                tree['visited_path'].append(behaviour_id)
            tree['behaviours'][behaviour_id] = {
                'id': behaviour_id,
                'status': conversions.msg_constant_to_status_str(behaviour.status),
                'name': utilities.normalise_name_strings(behaviour.name),
                'colour': colours[behaviour_type],
                'details': behaviour.additional_detail,
                'children': [str(conversions.msg_to_uuid4(child_id)) for child_id in behaviour.child_ids],
                'data': {
                    'Class': behaviour.class_name,
                    'Feedback': behaviour.message,
                },
            }
            if behaviour.blackboard_access:
                variables = []
                for variable in behaviour.blackboard_access:
                    variables.append(variable.key + " ({})".format(variable.value))
                    tree['blackboard']['behaviours'].setdefault(behaviour_id, {})[variable.key] = variable.value
                tree['behaviours'][behaviour_id]['data']['Blackboard'] = variables
                # delete keys from the cache if they aren't in the visited variables list when
                # they should be (i.e. their parent behaviour is on the visited path and has
                # 'w' or 'x' permissions on the variable).
                if (
                    variable.key in self.cached_blackboard and
                    variable.value != 'r' and
                    behaviour.is_active and
                    variable.key not in blackboard_variables
                ):
                    del self.cached_blackboard[variable.key]
        # hack, update the blackboard from visited path contexts
        self.cached_blackboard.update(blackboard_variables)
        if self.snapshot_stream.parameters.blackboard_data:
            tree['blackboard']['data'] = copy.deepcopy(self.cached_blackboard)
        if self.snapshot_stream.parameters.blackboard_activity:
            xhtml = utilities.XhtmlSymbols()
            xhtml_snippet = "<table>"
            for item in msg.blackboard_activity:
                if item.activity_type == "READ":
                    info = xhtml.normal + xhtml.left_arrow + xhtml.space + item.current_value + xhtml.reset
                elif item.activity_type == "WRITE":
                    info = xhtml.green + xhtml.right_arrow + xhtml.space + item.current_value + xhtml.reset
                elif item.activity_type == "ACCESSED":
                    info = xhtml.yellow + xhtml.left_right_arrow + xhtml.space + item.current_value + xhtml.reset
                elif item.activity_type == "ACCESS_DENIED":
                    info = xhtml.red + xhtml.multiplication_x + xhtml.space + "client has no read/write access" + xhtml.reset
                elif item.activity_type == "NO_KEY":
                    info = xhtml.red + xhtml.multiplication_x + xhtml.space + "key does not yet exist" + xhtml.reset
                elif item.activity_type == "NO_OVERWRITE":
                    info = xhtml.yellow + xhtml.forbidden_circle + xhtml.space + item.current_value + xhtml.reset
                elif item.activity_type == "UNSET":
                    info = ""
                elif item.activity_type == "INITIALISED":
                    info = xhtml.green + xhtml.right_arrow + xhtml.space + item.current_value + xhtml.reset
                else:
                    info = ""
                xhtml_snippet += (
                    "<tr>"
                    "<td>" + xhtml.cyan + item.key + xhtml.reset + "</td>"
                    "<td style='text-align: center;'>" + xhtml.yellow + item.activity_type + xhtml.reset + "</td>"
                    "<td style='text-align: center;'>" + xhtml.normal + item.client_name + xhtml.reset + "</td>"
                    "<td>" + info + "</td>"
                    "</tr>"
                )
            xhtml_snippet += "</table>"
            tree['activity'] = [xhtml_snippet]

        self.tree = tree
        self.node.get_logger().debug(f'Got first Tree', once=True)
        self.tree_snapshot_callback(tree)
        return tree
