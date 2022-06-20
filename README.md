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
## build:
1. Change directories to your development workspace directory
   - cd
   - cd dev_ws_uf
2. Build the three packages
   - colcon build
## use launch file to start the four nodes in uf_nav package
- ros2 launch uf_nav uf_nav_launch.py

