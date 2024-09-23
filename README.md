# ACDC Research Project - Advanced Filtering for Object Tracking

## Introduction
This repository aims to implement various Gausian filters - Kalman Filter(KF), Extended Kalman Filter(EKF), and Unscented Kalman Filter(UKF). It also deals with addition of noise measurements to camera and radar sensor measurements that aims to reduce Mean Squared Error (MSE) for the fused object after object tracking. The source code for the filtering techniques and noise generation scripts are built upon existing program by [RWTH Institute for Automotive Engineering](https://github.com/ika-rwth-aachen). This project is under the [ACDC Research Project Course](https://github.com/ika-rwth-aachen/acdc-research-projects) by the IKA, RWTH Aachen University. 

This project aims to:
1. Implement EKF and UKF source codes to the ROS nodes. 
2. apply different kinds of noise distributions to sensor measurement data.

A detailed documentation regarding this project can be found [here]().

## Getting started

### Installation
0. Follow all steps described under [Installation](https://github.com/ika-rwth-aachen/acdc/wiki#installations) in this repository's Wiki to setup your coding environment.

1. Clone this repository with the contained submodules:
    ```bash
    git clone --recurse-submodules https://github.com/mananvora/acdc.git
    ```

2. Pull the Docker image that is needed to run our tasks.:
    ```bash
    docker pull rwthika/acdc:latest
    ```

3. In a terminal, navigate to the Docker directory of this repository and launch the ACDC Docker container with the provided run script:
    ```bash
    # acdc/docker
    ./ros1_run.sh
    ```
    Once you run this script, the docker container will start running. When this is done, proceed to the **Quick start** section. 
    
### Quick start

1. Open a new terminal and run the ACDC Docker container again. After that, compile the C++ ROS code and source the `setup.bash` file with the following script:
    ```bash
    catkin build
    source devel/setup.bash
    ```
2. The simulations can be launched now. They can be executed with their respective launch commands:

   Launch RVIZ playback node:
   ```bash
   roslaunch acdc_launchpack bag_playback.launch
   ```
   
   Launch and object_fusion_wrapper node:
   ```bash
   roslaunch object_fusion_wrapper fusion.launch
   ```
   
   Play bag files to visualize fused object:
   ```bash
   rosbag play -l ../bag/Bagfiles_original/KF_bagfiles/acdc_fusion_guidance_noise_gaussian_noise.bag
   or 
   rosbag play -l /home/rosuser/ws/bag/Bagfiles_original/KF_bagfiles/acdc_fusion_guidance_noise_gaussian_noise.bag
   ```
   
  Record bag files: 
     Launch and object_fusion_wrapper node:
   ```bash
   roslaunch object_fusion_wrapper record_and_play_fused_object.launch
   ```
  To find which bag file to be played please look into [README_bagfiles_structure.md](https://github.com/mananvora/acdc/blob/main/bag/README_bagfiles_structure.md?plain=1).
   
- Recording of the bag files is further explained in [README.md](https://github.com/mananvora/acdc/tree/main/catkin_workspace/src/workshops/section_3/object_fusion_wrapper)


**Note:** The source code files *StateFuser.cpp* and *StatePredictor.cpp* must be changed for different filters. Further, the parameteres for fusion also have to be changed. This has explained [here](https://github.com/mananvora/acdc/blob/main/catkin_workspace/src/workshops/section_3/object_fusion/README.md)

## License

Copyright (c) 2022, Institute for Automotive Engineering (ika), RWTH University
