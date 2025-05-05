from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, LogInfo, TimerAction
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition

def generate_launch_description():
    """启动完整系统测试环境"""
    return LaunchDescription([
        # 声明参数
        DeclareLaunchArgument(
            'serial_port',
            default_value='/dev/ttyVIRTUAL1',
            description='串口设备路径'
        ),
        
        DeclareLaunchArgument(
            'use_decision',
            default_value='false',  # 默认禁用决策节点
            description='是否启动决策节点'
        ),
        
        DeclareLaunchArgument(
            'use_client',
            default_value='false',
            description='是否启动虚拟测试客户端'
        ),
        
        # 启动串口驱动节点
        Node(
            package='rm_serial',
            executable='serial_driver',
            name='serial_driver',
            parameters=[{
                'serial_port': LaunchConfiguration('serial_port'),
                'baud_rate': 115200,
                'cmd_timeout': 0.5,
                'timer_period': 0.02,
                'reconnect_delay': 2.0,
                'max_reconnect_attempts': -1
            }],
            output='screen'
        ),
        
        # 决策节点 (条件启动)
        Node(
            package='rm_decision',
            executable='rm_decision',
            name='rm_decision',
            output='screen',
            condition=IfCondition(LaunchConfiguration('use_decision'))
        ),
        
        # 延迟启动测试客户端，确保串口节点已经准备好
        TimerAction(
            period=2.0,
            actions=[
                Node(
                    package='rm_serial',
                    executable='test_client',
                    name='test_client',
                    output='screen',
                    parameters=[{
                        'port': '/dev/ttyVIRTUAL2',
                        'baudrate': 115200
                    }],
                    condition=IfCondition(LaunchConfiguration('use_client'))
                ),
                LogInfo(
                    msg=["测试环境已启动，串口设置为 ", LaunchConfiguration('serial_port')]
                )
            ]
        )
    ])