# ACDC Research Project - Processing of Traffic Light Status Information in MPC-Planner

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
    git clone --recurse-submodules https://github.com/Ihrer73/acdc.git
    ```

2. Pull the Docker image that is needed to run our tasks.:
    ```bash
    docker pull rwthika/acdc:latest
    ```

3. In a terminal, navigate to the Docker directory of this repository and launch the ACDC Docker container with the provided run script:
    ```bash
    # acdc/docker
    ./run.sh
    ```
    Once you run this script, the docker container will start running. When this is done, proceed to the **Quick start** section. 
    
### Quick start

1. Open a new terminal and run the ACDC Docker container again. After that, compile the C++ ROS code and source the `setup.bash` file with the following script:
    ```bash
    catkin build
    source devel/setup.bash
    ```
2. The simulations can be launched now. They can be executed with their respective launch commands:

   Launch playback node:
   ```bash
   roslaunch acdc_launchpack bag_playback.launch
   ```
   
   Launch RVIZ and object_fusion_wrapper node:
   ```bash
   roslaunch object_fusion_wrapper fusion.launch
   ```

    Play bag files:
   ```bash
   rosbag play -l ../bag/acdc_fusion_guidaance_noise.bag
   ```

**Note:** The source code files *StateFuser.cpp* and *StatePredictor.cpp* must be changed for different filters. 

## License

Copyright (c) 2022, Institute for Automotive Engineering (ika), RWTH University