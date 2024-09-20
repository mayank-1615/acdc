There exists two launch files in the launch directory of the object_fusion_wrapper package.

### fusion.launch
This launch file is responsible for starting the object fusion node with the necessary configurations, allowing it to perform sensor fusion, likely within a simulation or data replay context from the bag files. 


### record_and_play_fused_object.launch
The task of this launch file is to play a bag file and record specified topics into a new bag file
Modicfications can be made to this file to run specific filtering techniques - KF/EKF/UKF and particular bag files with specific noise distributions.


One can refer to the dtructure of the bagfiles firectory at [README_bagfiles_structure.md](https://github.com/mananvora/acdc/blob/main/bag/README_bagfiles_structure.md?plain=1)


 **Play bag file:**
    ```xml
    <node pkg="rosbag" type="play" name="rosbag_play" args="-l /home/rosuser/ws/bag/Bagfiles_originial/*filter*/acdc_fusion_guidance_*noise*.bag"/>
    ```
    
Replace the '*noise*' with the noise name for whichever noise distribution has been added to the bag file during 
    1. For EKF/UKF - "EUKF"
    2. For KF - "KF"
    
    
Replace the '*noise*' with the noise name for whichever noise distribution has been added to the bag file during 

    1. gaussian noise - "noise_noise"
    2. laplcae noise - "laplace_noise"
    3. cauchy noise - "cauchy_noise"
    4. normal logarithmic noise - "lognormal_noise"
    
    example: bag file for EKF and gaussian noise distribution - /home/rosuser/ws/bag/Bagfiles_originial/EUKF/acdc_fusion_guidance_gaussian_noise.bag

**Record bag file**
    ```xml
    <node pkg="rosbag" type="record" name="rosbag_record" args="-O /home/rosuser/ws/bag/Bagfiles_original/Recordings/*filter_dir*/recording_*noise*_*filter*.bag /acdc_fusion_guidance_*noise*.bag /sensors/fusion/ikaObjectList /sensors/reference/ikaObjectList"/>
    ```

Replace the '*filter_dir*' with the filter name 
    1. For KF - "KF"
    2. For EKF - "EKF"
    3. For UKF - "UKF"
Replace the '*filter*' with the filter name 
    1. For KF - "kf"
    2. For EKF - "ekf"
    3. For UKF - "ukf"
Replace the '*noise*' with the noise name 
    1. gaussian noise - "noise"
    2. laplcae noise - "laplace"
    3. cauchy noise - "cauchy"
    4. normal logarithmic noise - "lognormal"
    
*For instance if EKF is to be implemented on bagfile with gaussian noise distribution*
	home/rosuser/ws/bag/Bagfiles_original/Recordings/EUKF/recording_gaussian_ekf.bag /acdc_fusion_guidance_*noise*.bag
