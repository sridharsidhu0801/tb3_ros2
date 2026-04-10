from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

config_file = os.path.join(
	os.getcwd(), "config", "tb_velocity_smoother.yaml"
)

def generate_launch_description():
# Node defines what to run (the individual robot behaviors).
# LaunchDescription defines how they are grouped together to start at once.
	namespace = "tb1"
	# We use 'Node' to define the specific process
	watchdog_node = Node(
		package="tb_watchdog",
		executable="cmd_vel_watchdog",
		name="cmd_vel_watchdog",
		namespace=namespace,
		parameters=[
			{
				"input_topic": f"/{namespace}/cmd_vel_raw",
				"output_topic": f"/{namespace}/cmd_vel_watchdog",
				"timeout_sec": 0.5,
				"publish_rate": 20.0
			}
		],
	)

	velocity_smoother_node = Node(
		package="nav2_velocity_smoother",
		executable="velocity_smoother",
		name="velocity_smoother",
		namespace=namespace,
		parameters=[config_file],
		remappings=[
			("cmd_vel", f"/{namespace}/cmd_vel_watchdog"),
			("cmd_vel_smoothed", f"/{namespace}/cmd_vel"),
		],
	)

	    # Auto-configure + activate smoother
	configure_node = ExecuteProcess(
		cmd=['ros2', 'lifecycle', 'set', f'/{namespace}/velocity_smoother', 'configure'],
		output='screen'
	)

	activate_node = ExecuteProcess(
		cmd=['ros2', 'lifecycle', 'set', f'/{namespace}/velocity_smoother', 'activate'],
		output='screen'
	)

	# We use 'LaunchDescription' to return the list of actions to execute
	return LaunchDescription([
		watchdog_node,
		velocity_smoother_node,
		configure_node,
		activate_node,
])
