import os
import yaml

from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    dvl_a50_node = Node(
        package='dvl_a50', 
        executable='dvl_a50_sensor', 
        name='dvl_a50_node',
        output='screen',
        parameters=[{'dvl_ip_address': '192.168.2.95'}],
    )

    # HDK MicroStrain 3DM-GV7-INS
    gv7_serial_bridge = ExecuteProcess(
        cmd=["socat",
             "PTY,link=/tmp/ttyV0,raw,echo=0",
             "TCP:192.168.2.2:5555"],
        output="screen",
        respawn=True,
        respawn_delay=2.0,
    )
    gv7_default_params = os.path.join(
        get_package_share_directory("microstrain_inertial_driver"),
        "microstrain_inertial_driver_common", "config", "params.yml")
    gv7_ins_node = Node(
        package="microstrain_inertial_driver",
        executable="microstrain_inertial_driver_node",
        name="gv7_ins_node",
        output="screen",
        respawn=True,
        respawn_delay=2.0,
        parameters=[
            yaml.safe_load(open(gv7_default_params, "r")),
            {
                "port": "/tmp/ttyV0",
                "baudrate": 115200,
                "imu_data_rate": 100,                      # /imu/data
                "imu_mag_data_rate": 100,                  # /imu/mag
                "filter_imu_data_rate": 100,               # /ekf/imu/data (fused attitude)
                "filter_pps_source": 0,
                "filter_enable_gnss_pos_vel_aiding": False,
                "filter_enable_gnss_heading_aiding": False,
                "filter_enable_magnetometer_aiding": True,
                "filter_declination_source": 1,
                "filter_auto_init": True,
                "filter_auto_heading_alignment_selector": 4,
                #   R_body_imu (sensor frame -> vehicle FRD frame) =
                #     [ -0.030561  0.999533  0.000000 ]
                #     [  0.000357  0.000011  1.000000 ]
                #     [  0.999533  0.030561 -0.000357 ]
                "use_enu_frame": False,
            },
        ],
    )

    mavlink_bridge_node = Node(
        package="recording",
        executable="mavlink_bridge_node",
        name="mavlink_bridge_node",
        output="screen",
    )

    camera_node = Node(
        package="gscam",
        executable="gscam_node",
        name="camera_node",
        output="screen",
        parameters=[{
            "gscam_config": (
                "udpsrc port=5600 ! "
                "application/x-rtp, payload=96 ! "
                "rtpjitterbuffer latency=30 ! "
                "rtph264depay ! "
                "avdec_h264 ! "
                "videoconvert"
            ),
            "frame_id": "camera_link",
            "sync_sink": False,
            "preroll": False,
            "use_gst_timestamps": False,
        }],
    )

    monitor_node = Node(
        package="recording",
        executable="sensor_health_monitor",
        name="sensor_health_monitor",
        output="screen",
        parameters=[{
            "stale_threshold": 2.0,   # warn after this many seconds without data change
            "startup_grace": 3.0,     # no checks during the first seconds after launch
        }],
    )

    ld = LaunchDescription()
    ld.add_action(dvl_a50_node)
    ld.add_action(gv7_serial_bridge)
    ld.add_action(gv7_ins_node)
    ld.add_action(mavlink_bridge_node)
    # ld.add_action(camera_node)
    ld.add_action(monitor_node)
    
    return ld

