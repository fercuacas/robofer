from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.conditions import IfCondition, UnlessCondition

def generate_launch_description():
    sim = LaunchConfiguration('sim')

    eyes_width  = LaunchConfiguration('eyes_width')
    eyes_height = LaunchConfiguration('eyes_height')
    fps         = LaunchConfiguration('fps')
    menu_timeout_ms = LaunchConfiguration('menu_timeout_ms')

    spi_device  = LaunchConfiguration('spi_device')
    spi_hz      = LaunchConfiguration('spi_hz')
    use_manual_cs = LaunchConfiguration('use_manual_cs')
    gpiochip_c  = LaunchConfiguration('gpiochip_c')
    dc_offset   = LaunchConfiguration('dc_offset')
    rst_offset  = LaunchConfiguration('rst_offset')
    cs_offset   = LaunchConfiguration('cs_offset')
    spi_chunk   = LaunchConfiguration('spi_chunk')
    madctl      = LaunchConfiguration('madctl')
    invert      = LaunchConfiguration('invert')
    self_test   = LaunchConfiguration('self_test')

    btn1_offset = LaunchConfiguration('btn1_offset')
    btn2_offset = LaunchConfiguration('btn2_offset')
    btn3_offset = LaunchConfiguration('btn3_offset')
    btn4_offset = LaunchConfiguration('btn4_offset')

    eyes_node_sim = Node(
        package='robofer',
        executable='eyes_unified_node',
        name='eyes_unified_node',
        output='screen',
        parameters=[{
            'backend': 'sim',
            'eyes_width': eyes_width,
            'eyes_height': eyes_height,
            'fps': fps,
            'menu_timeout_ms': menu_timeout_ms,
        }]
    )

    eyes_node_hw = Node(
        package='robofer',
        executable='eyes_unified_node',
        name='eyes_unified_node',
        output='screen',
        parameters=[{
            'backend': 'st7735',
            'eyes_width': eyes_width,
            'eyes_height': eyes_height,
            'fps': fps,
            'menu_timeout_ms': menu_timeout_ms,
            'spi_device': spi_device,
            'spi_hz': spi_hz,
            'use_manual_cs': use_manual_cs,
            'gpiochip_c': gpiochip_c,
            'dc_offset': dc_offset,
            'rst_offset': rst_offset,
            'cs_offset': cs_offset,
            'spi_chunk': spi_chunk,
            'madctl': madctl,
            'invert': invert,
            'self_test': self_test,
        }]
    )

    keyboard_node = Node(
        package='robofer',
        executable='keyboard_buttons_node',
        name='keyboard_buttons_node',
        output='screen',
        parameters=[{
            'key_up': 'w',
            'key_down': 's',
            'key_back': 'a',
            'key_ok': 'd',
            'repeat_ms': 0,
        }]
    )

    buttons_node = Node(
        package='robofer',
        executable='buttons_node',
        name='buttons_node',
        output='screen',
        parameters=[{
            'gpiochip': gpiochip_c,
            'rising_on_press': True,
            'debounce_ms': 150,
            'btn1_offset': btn1_offset,
            'btn2_offset': btn2_offset,
            'btn3_offset': btn3_offset,
            'btn4_offset': btn4_offset,
            'btn1_code': 0,
            'btn2_code': 1,
            'btn3_code': 2,
            'btn4_code': 3,
        }]
    )

    ld = LaunchDescription()

    ld.add_action(DeclareLaunchArgument('sim', default_value='true'))

    ld.add_action(DeclareLaunchArgument('eyes_width', default_value='128'))
    ld.add_action(DeclareLaunchArgument('eyes_height', default_value='64'))
    ld.add_action(DeclareLaunchArgument('fps', default_value='30'))
    ld.add_action(DeclareLaunchArgument('menu_timeout_ms', default_value='5000'))

    ld.add_action(DeclareLaunchArgument('spi_device', default_value='/dev/spidev1.0'))
    ld.add_action(DeclareLaunchArgument('spi_hz', default_value='24000000'))
    ld.add_action(DeclareLaunchArgument('use_manual_cs', default_value='true'))
    ld.add_action(DeclareLaunchArgument('gpiochip_c', default_value='gpiochip0'))
    ld.add_action(DeclareLaunchArgument('dc_offset', default_value='75'))
    ld.add_action(DeclareLaunchArgument('rst_offset', default_value='78'))
    ld.add_action(DeclareLaunchArgument('cs_offset', default_value='233'))
    ld.add_action(DeclareLaunchArgument('spi_chunk', default_value='2048'))
    ld.add_action(DeclareLaunchArgument('madctl', default_value='0'))
    ld.add_action(DeclareLaunchArgument('invert', default_value='false'))
    ld.add_action(DeclareLaunchArgument('self_test', default_value='true'))

    ld.add_action(DeclareLaunchArgument('btn1_offset', default_value='-1'))
    ld.add_action(DeclareLaunchArgument('btn2_offset', default_value='-1'))
    ld.add_action(DeclareLaunchArgument('btn3_offset', default_value='-1'))
    ld.add_action(DeclareLaunchArgument('btn4_offset', default_value='-1'))

    eyes_node_sim.condition = IfCondition(sim)
    eyes_node_hw.condition = UnlessCondition(sim)
    keyboard_node.condition = IfCondition(sim)
    buttons_node.condition = UnlessCondition(sim)

    ld.add_action(eyes_node_sim)
    ld.add_action(eyes_node_hw)
    ld.add_action(keyboard_node)
    ld.add_action(buttons_node)

    return ld

