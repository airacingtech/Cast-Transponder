
# Launch file for the transponder

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    ld = LaunchDescription()

    # Node for interfacing with the transponder
    node = Node(
        package='transponder2ros',
        executable='transponder2ros_node',
        name='transponder_node',
        output='both',
        namespace='transponder',
        parameters=[
            # ROS network
            {'transponder_in': 'in'},         # Topic for data from the transponder
            {'transponder_out': 'out'},       # Topic for data to the transponder
            {'version_out': 'version'},       # Topic to print version information
            # Transponder network
            {'transponder_ip': '10.42.17.210'},  # IP address of the transponder
            {'udp_port': 15783},              # UDP port to communicate on (match transponder)
            {'max_age':  1.0},                # Max age of packets, otherwise reject [ s ]
            {'timeout': 10.0},                # Time before notifying of no packets [ s ]
            # Serial network (direct xbee connection for debugging)
            {'device_id': ''},                # Device ID of the XBee (leave blank to disable)
        ],
    )
    ld.add_action(node)

    # Odom 2 Transponder Node
    node = Node(
        package="transponder2ros",
        executable="odom2transponder_node",
        name="odom2trans",
        output="both",
        namespace="transponder",
        parameters=[
                {'odometry_in' : '/state/odom'},    # Odometry in topic
                {'transponder_out' : 'out'},        # Transponder out topic
                {'car' : 1},                        # Car ID
                {'lat0' : 45.61898},   # Monza      # Reference lla
                {'lon0' : 9.2811880},
                {'alt0' : 176.61984507},
        ],
    )
    ld.add_action(node)

    # Data faker node (for debugging, commented out)
    node = Node(
        package='transponder2ros',
        executable='debug_node',
        name='data_node',
        output='screen',
        namespace='transponder',
        parameters=[
            {'topic': 'out'},           # Topic to send out fake to transponder messages
        ]
    )
    # ld.add_action(node)

    return ld
