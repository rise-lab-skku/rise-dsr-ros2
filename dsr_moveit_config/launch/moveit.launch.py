import os
os.system("clear")
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription,
                            Shutdown)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch.actions import OpaqueFunction
from launch_ros.substitutions import FindPackageShare
import yaml


def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, 'r') as file:
            return yaml.safe_load(file)
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        return None

def launch_setup(context, *args, **kwargs):
    use_fake_hardware_parameter_name = 'use_fake_hardware'
    hardware_plugin_parameter_name = 'hardware_plugin'

    # Command-line arguments
    hardware_plugin_arg = DeclareLaunchArgument(
        hardware_plugin_parameter_name,
        default_value='fake',
        description='Use fake hardware interface for ros2_control')

    hardware_plugin = LaunchConfiguration(hardware_plugin_parameter_name)
    hardware_plugin_arg.execute(context)
    hardware_plugin_value =  hardware_plugin.perform(context)

    use_fake_hardware_arg = DeclareLaunchArgument(
        use_fake_hardware_parameter_name,
        default_value='true' if hardware_plugin_value == 'fake' else 'false',
        description='Use fake hardware for moveit')

    db_arg = DeclareLaunchArgument(
        'db', default_value='False', description='Database flag'
    )


    # planning_context
    robot_xacro_file = os.path.join(get_package_share_directory('dsr_description'), 'urdf',
                                    'dsr.urdf.xacro')
    robot_description_config = Command(
        [FindExecutable(name='xacro'), ' ', robot_xacro_file, ' hardware_plugin:=', hardware_plugin_value])

    robot_description = {'robot_description': robot_description_config}

    robot_semantic_xacro_file = os.path.join(get_package_share_directory('dsr_moveit_config'),
                                                                        'srdf',
                                                                        'dsr.srdf.xacro')
    robot_description_semantic_config = Command(
        [FindExecutable(name='xacro'), ' ', robot_semantic_xacro_file]
    )
    robot_description_semantic = {
        'robot_description_semantic': robot_description_semantic_config
    }

    kinematics_yaml = load_yaml(
        'dsr_moveit_config', 'config/kinematics.yaml'
    )

    # Planning Functionality
    ompl_planning_pipeline_config = {
        'move_group': {
            'planning_plugin': 'ompl_interface/OMPLPlanner',
            'request_adapters': 'default_planner_request_adapters/AddTimeOptimalParameterization '
                                'default_planner_request_adapters/ResolveConstraintFrames '
                                'default_planner_request_adapters/FixWorkspaceBounds '
                                'default_planner_request_adapters/FixStartStateBounds '
                                'default_planner_request_adapters/FixStartStateCollision '
                                'default_planner_request_adapters/FixStartStatePathConstraints',
            'start_state_max_bounds_error': 0.1,
        }
    }
    ompl_planning_yaml = load_yaml(
        'dsr_moveit_config', 'config/ompl_planning.yaml'
    )

    ompl_planning_pipeline_config['move_group'].update(ompl_planning_yaml)

    # Trajectory Execution Functionality
    moveit_simple_controllers_yaml = load_yaml(
        'dsr_moveit_config', 'config/dsr_controllers.yaml'
    )
    moveit_controllers = {
        'moveit_simple_controller_manager': moveit_simple_controllers_yaml,
        'moveit_controller_manager': 'moveit_simple_controller_manager'
                                    '/MoveItSimpleControllerManager',
    }

    trajectory_execution = {
        'moveit_manage_controllers': True,
        'trajectory_execution.allowed_execution_duration_scaling': 1.2,
        'trajectory_execution.allowed_goal_duration_margin': 0.5,
        'trajectory_execution.allowed_start_tolerance': 0.01,
    }

    planning_scene_monitor_parameters = {
        'publish_planning_scene': True,
        'publish_geometry_updates': True,
        'publish_state_updates': True,
        'publish_transforms_updates': True,
    }

    # Start the actual move_group node/action server
    run_move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        parameters=[
            robot_description,
            robot_description_semantic,
            kinematics_yaml,
            ompl_planning_pipeline_config,
            trajectory_execution,
            moveit_controllers,
            planning_scene_monitor_parameters,
        ],
    )

    # RViz
    rviz_base = os.path.join(get_package_share_directory('dsr_moveit_config'), 'rviz')
    rviz_full_config = os.path.join(rviz_base, 'moveit.rviz')

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='log',
        arguments=['-d', rviz_full_config],
        parameters=[
            robot_description,
            robot_description_semantic,
            ompl_planning_pipeline_config,
            kinematics_yaml,
        ],
    )

    # Publish TF
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[robot_description],
    )

    ros2_controllers_path = os.path.join(
        get_package_share_directory('dsr_moveit_config'),
        'config',
        'ros_controllers.yaml',
    )

    ros2_control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[robot_description, ros2_controllers_path],
        remappings=[('joint_states', 'dsr/joint_states')],
        output={
            'stdout': 'screen',
            'stderr': 'screen',
        },
        on_exit=Shutdown(),
    )

    # Load controllers
    load_controllers = []
    inactive_controllers = []
    active_controllers= ['joint_state_broadcaster', 'arm_controller']
    for controller in inactive_controllers:
        load_controllers += [
            ExecuteProcess(
                cmd=['ros2 run controller_manager spawner.py --stopped {}'.format(controller)],
                shell=True,
                output='screen',
            )
        ]
    for controller in active_controllers:
        load_controllers += [
            ExecuteProcess(
                cmd=['ros2 run controller_manager spawner.py {}'.format(controller)],
                shell=True,
                output='screen',
            )
        ]

    # Warehouse mongodb server
    db_config = LaunchConfiguration('db')
    mongodb_server_node = Node(
        package='warehouse_ros_mongo',
        executable='mongo_wrapper_ros.py',
        parameters=[
            {'warehouse_port': 33829},
            {'warehouse_host': 'localhost'},
            {'warehouse_plugin': 'warehouse_ros_mongo::MongoDatabaseConnection'},
        ],
        output='screen',
        condition=IfCondition(db_config)
    )

    source_list = {'fake': {'source_list': ['dsr/joint_states'], 'rate': 30},
                    'real': {'source_list': ['hardware_interface/joint_states'], 'rate': 30},
                    'sim': {'source_list': ['sim_interface/joint_states'], 'rate': 30}}
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[source_list[hardware_plugin_value] if hardware_plugin_value in ['fake', 'real'] else source_list['sim']],
    )

    return [
            use_fake_hardware_arg,
            hardware_plugin_arg,
            db_arg,
            rviz_node,
            robot_state_publisher,
            run_move_group_node,
            ros2_control_node,
            mongodb_server_node,
            joint_state_publisher,
            ] + load_controllers

def generate_launch_description():

    return LaunchDescription(
        [OpaqueFunction(function = launch_setup)]
    )