#!/bin/bash
colcon build
source install/local_setup.bash
gnome-terminal --tab --title="webots" --command="bash -c 'source install/local_setup.bash; ros2 launch webots_pkg swarm_launch.py; $SHELL'"
gnome-terminal --tab --title="cfl" --command="bash -c 'ros2 run webots_pkg crazyflie_follower_left; $SHELL'"
gnome-terminal --tab --title="cfr" --command="bash -c 'ros2 run webots_pkg crazyflie_follower_right; $SHELL'"
gnome-terminal --tab --title="cl" --command="bash -c 'ros2 run webots_pkg crazyflie_leader; $SHELL'"
gnome-terminal --tab --title="serial" --command="bash -c 'ros2 run webots_pkg serial_communication; $SHELL'"
