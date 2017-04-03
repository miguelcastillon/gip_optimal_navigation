import os
import time
os.system("gnome-terminal -e 'bash -c \"roscore; exec bash\"'")
time.sleep(2)
os.system("gnome-terminal -e 'bash -c \"roslaunch turtlebot_gazebo turtlebot_world.launch world_file:=/opt/ros/indigo/share/turtlebot_gazebo/worlds/world1.world; exec bash\"'")
time.sleep(2)
os.system("gnome-terminal -e 'bash -c \"roslaunch turtlebot_teleop keyboard_teleop.launch; exec bash\"'")
time.sleep(2)
os.system("gnome-terminal -e 'bash -c \"roslaunch turtlebot_gazebo amcl_demo.launch map_file:=/home/miguel/GIP/demo_map_original.yaml; exec bash\"'")
time.sleep(2)
os.system("gnome-terminal -e 'bash -c \"roslaunch turtlebot_rviz_launchers view_navigation.launch; exec bash\"'")

