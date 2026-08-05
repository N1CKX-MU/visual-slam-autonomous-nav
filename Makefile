.PHONY: build slam nav explore full map-save clean

build:
	source /opt/ros/jazzy/setup.bash && cd ~/visual-slam-autonomous-nav && colcon build --packages-select autonomous_nav --symlink-install

slam:
	ros2 launch autonomous_nav slam_mapping.launch.py

nav:
	ros2 launch autonomous_nav navigation.launch.py

explore:
	ros2 launch autonomous_nav autonomous_explore.launch.py

full:
	ros2 launch autonomous_nav full_stack.launch.py

map-save:
	ros2 run nav2_map_server map_saver_cli -f ./src/autonomous_nav/maps/warehouse

clean:
	rm -rf build/ install/ log/
