#!/bin/bash
# Launch from docker image pet-mk-viii-runtime
source /home/pet/ws_ros2/install/setup.bash
#exec ros2 launch pet_mk_viii panel_led[0..4]_subscribers.launch.py
exec ros2 launch pet_mk_viii pet-mk-viii-bringup.py
#ros2 run pet_mk_viii pet_buttons_node.py &
#ros2 run pet_mk_viii_joystick pet_potentiometer_node.py &
#echo 'Done'