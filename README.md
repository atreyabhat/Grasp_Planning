# Grasp_Planning
Project repo for Vision Based Manipulation course @WPI

# Package Details:
grasp_gen_interface - define the srv file  
grasp_gen_service - create a service to load the DL model and provide grasp rectangle when requested  
grasp_pose_generated - generate grasping pose based on the grasp rectangle  
vbm_project_env - project environment with table and coke   

# Requirements:
trasnformers3d  
python-opencv  
torch-cuda  
imageio  
pcl (sudo apt-get install libpcap-dev, if you face issues with installation)


# Steps to get grasp:
1) Build the src
   ```bash
   colcon build --symlink-install

2) Launch the Gazebo:
   ```bash
   ros2 launch vbm_project_env simulation.launch.py

3) Run RVIZ:
   ```bash
   ros2 run rviz2 rviz2

4) Launch Grasp Pose Generator:
   ```bash
   ros2 launch grasp_pose_generator grasp_pose_generator_launch.py 

5) Start the Grasp Generation Service:
   ```bash
   ros2 run grasp_gen_service grasp_gen_service

6) Service call:
   ```bash
   ros2 service call /generate_grasp grasp_gen_interface/srv/GraspGen input:\ 'generate_grasp_grconvnet'\
   or 
   ```bash
   ros2 service call /generate_grasp grasp_gen_interface/srv/GraspGen input:\ 'generate_grasp_ggcnn'\
   
   
