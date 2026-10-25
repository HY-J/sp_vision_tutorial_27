import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    bringup_share = get_package_share_directory('sp_nav_bringup')
    sim_share = get_package_share_directory('sp_nav_sim')
    
    params_file = os.path.join(bringup_share, 'config', 'nav_params.yaml')
    sim_params_file = os.path.join(sim_share, 'config', 'sim_robot.yaml')
    rviz_config = os.path.join(bringup_share, 'rviz', 'rviz.rviz')

    return LaunchDescription([
        # 地图服务器
        Node(
            package='sp_map_server',
            executable='esdf_map_publisher',
            name='esdf_map_publisher',
            output='screen',
            parameters=[params_file],
        ),
        # 全局规划器
        Node(
            package='sp_global_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[params_file],
            remappings=[('goal_pose', '/goal_pose'),('make_plan', '/planner_server/make_plan'),],
        ),
        # 控制器
        Node(
            package='sp_controller_server',
            executable='controller_node',
            name='controller_server',
            output='screen',
            parameters=[params_file],
        ),
        # 决策层（上层行为树 click_nav.xml：接收 RViz 点击目标）
        Node(
            package='sp_decision',
            executable='sp_decision_node',
            name='sp_decision',
            output='screen',
            parameters=[params_file],
        ),
        # 导航接口（下层行为树 default_nav_with_fallback.xml：A* 规划 + 控制使能 + 状态跟踪）
        Node(
            package='sp_nav_bt',
            executable='nav_interface_node',
            name='nav_interface_node',
            output='screen',
            parameters=[params_file],
        ),
        # 仿真器
        Node(
            package='sp_nav_sim',
            executable='sim_robot',
            name='sim_robot_node',
            output='screen',
            emulate_tty=True,
            parameters=[sim_params_file],
        ),
        # RViz
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config],
        ),
    ])
