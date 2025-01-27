import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Bool, String, Empty
from geometry_msgs.msg import Point
from sensor_msgs.msg import BatteryState
from nicegui import app, Client, ui_run, ui
from rclpy.executors import ExternalShutdownException
from pathlib import Path
import threading
from geometry_msgs.msg import PoseStamped

from . import treestream as ros_backend


from . import subpage_tree


class Dashboard(Node):
    def __init__(self):
        super().__init__('mock_robi_state_subscriber')
        self.get_logger().set_level(rclpy.logging.LoggingSeverity.DEBUG)
        self.tree = None

        self.backend = ros_backend.Backend(
            parameters=ros_backend.SnapshotStream.Parameters(
                blackboard_data=True,
                blackboard_activity=True,
                snapshot_period=1.0),
            tree_snapshot_callback=self.tree_snapshot_callback
        )
        self.backend.start_spin()

        # Access via localhost:8080/tree
        subpage_tree.create()

    def tree_snapshot_callback(self, tree):
        for client in app.clients('/tree'):
            with client:
                ui.run_javascript(f'render_tree({{tree: {tree}}});')


def ros_main() -> None:
    # Standart ROS2 node initialization
    print('Starting ROS2...', flush=True)
    rclpy.init()
    my_Dashboard = Dashboard()

    try:
        rclpy.spin(my_Dashboard)
    except ExternalShutdownException:
        pass


########################################################################################
# Stuff needed for NiceGUI to work.
########################################################################################

# The main function is still mapped in the setup.py.
def main():
    pass  # This is originally used as the ROS entry point, but we give the control of the node to NiceGUI.

# I have not figured out how this code every runs when starting the launch file, but it does.


# Starting the ros node in a thread managed by nicegui. It will restarted with "on_startup" after a reload.
# It has to be in a thread, since NiceGUI wants the main thread for itself.
app.on_startup(lambda: threading.Thread(target=ros_main).start())

# This is for the automatic reloading by nicegui/fastapi
ui_run.APP_IMPORT_STRING = f'{__name__}:app'

# We add reload dirs to just watch changes in our package
ui.run(title='Robot Controll', uvicorn_reload_dirs=str(Path(__file__).parent.resolve()), favicon='🤖')
