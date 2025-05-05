from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable

def generate_launch_description():
    """创建启动描述"""
    return LaunchDescription([
        # 设置环境变量来控制日志级别和格式
        SetEnvironmentVariable('RCUTILS_CONSOLE_OUTPUT_FORMAT', 
            '[{severity}] [{name}]: {message}'),
        SetEnvironmentVariable('RCUTILS_LOGGING_USE_STDOUT', '1'),
        SetEnvironmentVariable('RCUTILS_COLORIZED_OUTPUT', '1'),
        # 使用 ERROR/WARN/INFO 作为基础日志级别
        SetEnvironmentVariable('RCUTILS_LOGGING_MIN_SEVERITY', 'INFO'),

        DeclareLaunchArgument(
            'serial_port',
            default_value='/dev/ttyUART',
            description='串口设备路径'
        ),
        DeclareLaunchArgument(
            'baud_rate',
            default_value='115200',
            description='波特率'
        ),
        DeclareLaunchArgument(
            'debug',
            default_value='false',
            description='是否开启调试输出'
        ),

        # 创建节点
        Node(
            package='rm_serial',
            executable='serial_driver',
            name='serial_driver',
            parameters=[{
                'serial_port': LaunchConfiguration('serial_port'),
                'baud_rate': LaunchConfiguration('baud_rate'),
                'debug_output': LaunchConfiguration('debug')
            }],
            output='screen',
            # 设置基础日志级别
            arguments=['--ros-args', '--log-level', 
                      PythonExpression(["'debug' if '", LaunchConfiguration('debug'), 
                                      "' == 'true' else 'info'"])],
            # 过滤掉框架内部日志
            additional_env={
                'RCUTILS_LOGGING_FILTER_RULES': 'serial_driver:=info;rcl:=error;rcutils:=error;rclcpp:=error'
            }
        )
    ])