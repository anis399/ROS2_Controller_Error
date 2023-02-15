#!/usr/bin/python3
# -*- coding: utf-8 -*-
import os
import time
# 
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import (OnProcessStart, OnProcessExit)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

import xacro
from moveit_configs_utils import MoveItConfigsBuilder
from launch.actions import DeclareLaunchArgument
from ament_index_python.packages import get_package_prefix

#gazebo useful links: http://classic.gazebosim.org/tutorials?tut=ros_urdf&cat=connect_ros
#                    https://automaticaddison.com/how-to-load-a-urdf-file-into-gazebo-ros-2/

print(" *******************************************************************************")
print(" *")
print(" *     BBBBBBB      IIII   NNN     N   AA       TTTTTTTT   AA")
print(" *     B      B      II    N N     N   AAA        TTTT     AAA")
print(" *     B      BB     II    N  N    N   AAAA        TT      AAAA")
print(" *     BBBBBBBBB     II    N   N   N   AAAAA       TT      AAAAA")
print(" *     B        BB   II    N   N   N   AA   A      TT      AA   A")
print(" *     B        BB   II    N    N  N   AA    A     TT      AA    A")
print(" *     BBBBBBBBBB   IIII   N    NNNN   AA     A   TTTT     AA     A")
print(" *")
print(" *******************************************************************************")
print(" * All software (C, py, C++)")
print(" *******************************************************************************")
time.sleep(1)

print("")
print("                 _           _           _    _                    ")
print("     /\         (_)         | |         | |  | |                   ")
print("    /  \   _ __  _ ___   ___| |__   __ _| | _| | _____  _   _ _ __ ")
print("   / /\ \ | '_ \| / __| / __| '_ \ / _` | |/ / |/ / _ \| | | | '__|")
print("  / ____ \| | | | \__ \ \__ \ | | | (_| |   <|   < (_) | |_| | |   ")
print(" /_/    \_\_| |_|_|___/ |___/_| |_|\__,_|_|\_\_|\_\___/ \__,_|_|   ")
print("")
print("")
time.sleep(2)


def generate_launch_description():

    pkg_name = 'crutch_sim'
    world_name = 'myworld.world'
    urdf_name = 'crutch_urdf.urdf'
    xacro_name = 'crutch_urdf.xacro'

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    ### Getting paths relative to the install folder in the workspace
    ## package share location
    pkg_path_share = get_package_share_directory(pkg_name)
    print("pkg share: " + str(pkg_path_share))

    ## world
    world_path = os.path.join(pkg_path_share, 'worlds', world_name)
    print("world share: " + str(world_path))

    ##model
    model_path = os.path.join(pkg_path_share, 'model')

    ##urdf
    urdf_path = os.path.join(pkg_path_share, 'urdf', urdf_name)
    urdf_to_xml = open(urdf_path, 'r').read()
    urdf_to_xml = urdf_to_xml.replace('"', '\\"')
    swpan_args = '{name: \"my_robot\", xml: \"'  +  urdf_to_xml + '\" }'
    print("urdf share: " + str(urdf_path))
    
    ##xacro 
    xacro_path = os.path.join(pkg_path_share, 'xacro', xacro_name)
    print("xacro share: " + str(xacro_path))
    doc = xacro.parse(open(urdf_path))
    xacro.process_doc(doc)
    param = {'crutch_description': doc.toxml()}

    # print("process file")
    # robot_decription_xml = robot_description_config.toxml()
    # print("to xml")

    ### Gazebo
    gazebo_path = os.path.join(get_package_share_directory('gazebo_ros'), 'launch')
    print("Gazebo path: " + str(gazebo_path))
    run_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([gazebo_path, '/gazebo.launch.py']),
    )

    ###
    spawn_entity = Node(name='Spawn_process',
                        package='gazebo_ros',
                        type='spawn_model',
                        executable='spawn_entity.py',
                        arguments=['-entity', 'crutch_sim'
                                   '-topic', '/crutch_description'
                                   '-urdf', urdf_path, 
                                   '-model', model_path])

    return LaunchDescription([
        run_gazebo,
        spawn_entity
        # #Create updated xacro file from urdf
        # ExecuteProcess(
        #     cmd=['xacro', '-o', xacro_path, urdf_path, ],
        #     name='xacro_Process',
        #     output='screen'),
        
        # #Time synchronisation between ros2 & gazebo  
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation G(Gazebo) clock if true'
        ),
        
        # #Open/Run RVIZ
        # ExecuteProcess(
        #     cmd=['rviz2'],
        #     name='RVIZ2_Process',
        #     output='screen'),

        # #Open/Run Gazebo with custom world
        ExecuteProcess(
            cmd=['gazebo', '--verbose', world_path, '-s', 'libgazebo_ros_factory.so'],
            name='Gazebo_Process',
            output='screen'),

        # #Set ros2 time parameter
        ExecuteProcess(
            cmd=['ros2', 'param', 'set', '/gazebo', 'use_sim_time', use_sim_time],
            name='Set_Param_Process',
            output='screen'),

        # #Spawn robot to gazebo
        ExecuteProcess(
            cmd=['ros2', 'service', 'call', '/spawn_entity', 'gazebo_msgs/SpawnEntity', swpan_args],
            name="URDF_Spwaner_Process",
            output='screen'),

        # #Robot state publisher, publish all the static transformation of the crutch
        Node(
            package='crutch_controller',
            executable='imu_publisher',
            name='imu_publisher',
            output='screen',
            #parameters=[{'use_sim_time': use_sim_time}]
            ),

        # #Start broadcaster
        ExecuteProcess(
            cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', "forward_position_controller"],
            name="forward_position_controller",
            output='screen'),
        
        # #Start controller
        ExecuteProcess(
            cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', "forward_velocity_controller"],
            name="forward_velocity_controller",
            output='screen'),


    ])