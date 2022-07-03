## three ros2 packages: 
- uf_nav
- uf_nav_interfaces
- uf_py_templates
## installation:
1. From your 'home' directory, create your ROS2 development workspace directory
   - mkdir dev_ws_uf
2. Move into this directory and create an 'src' directory
   - cd dev_ws_uf
   - mkdir src
3. Move into the src directory
   - cd src
4. Download the three package directories from github
   - git clone https://github.com/av-mae-uf/uf_nav_repo.git
   - Note that you will have a new directory named uf_nav_repo in your src directory.  The three packages (uf_nav, uf_nav_interfaces, and uf_py_templates) will be in this directory.
## build:
1. Change directories to your development workspace directory
   - cd
   - cd dev_ws_uf
2. Build the three packages
   - colcon build
3. While in your development workspace directory, source the workspace
   - source install/setup.bash
## use launch file to start the four nodes in uf_nav package
- ros2 launch uf_nav uf_nav_launch.py
- If all goes well, you will have a vehicle (arrow) moving along a course.  The look-ahead-pose and the point on the path closest to the vehicle position are displayed as coordinate systems.  Three views are available under the 'Views' menu on the right.

## notes:
- The misc directory contains a python program named convert_to_UTM.  The program is run as *python3 convert_to_UTM.py lat_long_input.csv UTM_output.csv*.  This program converts an input file that has lat/long data separated by a comma to UTM coordinates, separated by a comma.
