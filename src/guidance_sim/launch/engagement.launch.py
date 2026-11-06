"""Bring up Gazebo plus two ArduPilot SITL instances for an engagement.

Both instances are launched with the SAME home location on purpose. The
ArduPilot Gazebo plugin reports WorldPose (world-origin relative), so a shared
home puts both vehicles in one common NED frame and their LOCAL_POSITION_NED
streams can be differenced directly with no offset bookkeeping.

Ports, following sim_vehicle.py's 'from_ardupilot_port = 9002 + 10*i':

    instance 0  pursuer  FDM 9002  MAVLink out udp:127.0.0.1:14550
    instance 1  target   FDM 9012  MAVLink out udp:127.0.0.1:14560

Usage::

    ros2 launch guidance_sim engagement.launch.py
    ros2 launch guidance_sim engagement.launch.py headless:=true

Then, in another shell::

    ros2 run guidance_sim engage --law pn --takeoff
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess,
                            TimerAction)
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution


def _sitl(instance, out_port, home, vehicle, extra_params, delay, state_dir):
    """One sim_vehicle.py instance, started after Gazebo has settled.

    sim_vehicle.py only makes a per-instance subdirectory for its own
    *parameter storage* (eeprom.bin) when ONE process is driving several
    instances at once; launched as two separate processes like this, each
    one sees itself as a lone instance and defaults to storing eeprom.bin
    in the current directory. Without --use-dir both instances would write
    the SAME eeprom.bin at once, corrupting each other's stored params
    (FRAME_CLASS/FRAME_TYPE included) -- the pre-arm "Check frame class and
    type" failure traces back to exactly this.
    """
    ardupilot = os.path.expanduser('~/guidance_ws/src/ardupilot')
    cmd = [
        os.path.join(ardupilot, 'Tools', 'autotest', 'sim_vehicle.py'),
        '-v', vehicle,
        '-f', 'gazebo-iris',
        '--model', 'JSON',
        '-I', str(instance),
        '--custom-location', home,
        '--out', f'udp:127.0.0.1:{out_port}',
        '--no-rebuild',
        '--add-param-file', extra_params,
        '--use-dir', state_dir,
    ]
    return TimerAction(
        period=delay,
        actions=[ExecuteProcess(cmd=cmd, output='screen', shell=False)])


def generate_launch_description():
    pkg = get_package_share_directory('guidance_sim')

    world = LaunchConfiguration('world')
    home = LaunchConfiguration('home')
    headless = LaunchConfiguration('headless')

    world_arg = DeclareLaunchArgument(
        'world',
        default_value=PathJoinSubstitution([pkg, 'worlds', 'engagement.sdf']),
        description='SDF world to load')
    home_arg = DeclareLaunchArgument(
        'home',
        default_value='-35.363262,149.165237,584,353',
        description='home as lat,lon,alt,heading -- IDENTICAL for both '
                    'instances so they share one NED frame')
    headless_arg = DeclareLaunchArgument(
        'headless', default_value='false',
        description='run gz sim without the GUI')
    vehicle_arg = DeclareLaunchArgument(
        'vehicle', default_value='ArduCopter')

    params = os.path.join(pkg, 'config', 'sitl_guided.parm')

    # Gazebo needs to resolve model://iris_pursuer and model://iris_target.
    models = os.path.join(pkg, 'models')
    gz_env = {'GZ_SIM_RESOURCE_PATH':
              models + os.pathsep + os.environ.get('GZ_SIM_RESOURCE_PATH', '')}

    gz_gui = ExecuteProcess(
        cmd=['gz', 'sim', '-v4', '-r', world],
        output='screen',
        additional_env=gz_env,
        condition=UnlessCondition(headless))

    gz_headless = ExecuteProcess(
        cmd=['gz', 'sim', '-v4', '-r', '-s', world],
        output='screen',
        additional_env=gz_env,
        condition=IfCondition(headless))

    vehicle = LaunchConfiguration('vehicle')

    return LaunchDescription([
        world_arg, home_arg, headless_arg, vehicle_arg,
        gz_gui,
        gz_headless,
        # Gazebo must be up and the plugin listening before SITL tries to
        # connect, and the two instances must not race each other onto the
        # same build lock.
        _sitl(0, 14550, home, vehicle, params, delay=6.0,
              state_dir=os.path.expanduser(
                  '~/guidance_ws/sitl_state/pursuer')),
        _sitl(1, 14560, home, vehicle, params, delay=12.0,
              state_dir=os.path.expanduser(
                  '~/guidance_ws/sitl_state/target')),
    ])
