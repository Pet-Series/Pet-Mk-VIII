########################################################################################
# (c) https://github.com/Pet-Series
#     https://github.com/Pet-Series/Pet-Mk-VIII
#
# Maintainer: stefan.kull@gmail.com
# The MIT License (MIT)
#
# ROS2 lanchfile
# Launches 5 nodes. One for each LED[0..4]=5 on the panel.
# 
# 1) Remember to add '<exec_depend>....</exec_depend>' in package.xml
# 2) Remember to add '(os.path.join('share', package_name), glob('launch/*.launch.py'))' in setup.py

from launch import LaunchDescription
from launch_ros.actions import Node

LED4 =  5 # GPIO05 (Pin 29)
LED3 =  6 # GPIO06 (Pin 31)
LED2 = 13 # GPIO13 (Pin 33)
LED1 = 19 # GPIO19 (Pin 35)
LED0 = 26 # GPIO26 (Pin 37)

def generate_launch_description():
    ld = LaunchDescription()

    # Create one node for each LED[0..4]=5 on the panel.
    led0_node = Node( 
        package="pet_mk_viii",
        executable="pet_ledlights_node",
        parameters=[
            {"led_topic": 'panel_led_0'},
            {"led_gpio_pin": LED0 },
        ]
    )

    led1_node = Node( 
        package="pet_mk_viii",
        executable="pet_ledlights_node",
        parameters=[
            {"led_topic": 'panel_led_1'},
            {"led_gpio_pin": LED1 },
        ]
    )

    led2_node = Node( 
        package="pet_mk_viii",
        executable="pet_ledlights_node",
        parameters=[
            {"led_topic": 'panel_led_2'},
            {"led_gpio_pin": LED2 },
        ]
    )

    led3_node = Node( 
        package="pet_mk_viii",
        executable="pet_ledlights_node",
        parameters=[
            {"led_topic": 'panel_led_3'},
            {"led_gpio_pin": LED3 },
        ]
    )

    led4_node = Node( 
        package="pet_mk_viii",
        executable="pet_ledlights_node",
        parameters=[
            {"led_topic": 'panel_led_4'},
            {"led_gpio_pin": LED4 },
        ]
    )

    # Expose nodes
    ld.add_action(led0_node)
    ld.add_action(led1_node)
    ld.add_action(led2_node)
    ld.add_action(led3_node)
    ld.add_action(led4_node)
    
    return ld