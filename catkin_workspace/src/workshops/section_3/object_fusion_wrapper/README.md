There exists two launch files in the launch directory of the object_fusion_wrapper package.

## fusion.launch
This launch file is responsible for starting the object fusion node with the necessary configurations, allowing it to perform sensor fusion, likely within a simulation or data replay context from the bag files. 



## play_record_bagfiles.launch
The task of this launch file is to play a bag file and record specified topics into a new bag file
Modicfications can be made to this file to run specific filtering techniques - KF/EKF/UKF and particular bag files with specific noise distributions.

play bag file: 
    <node pkg="rosbag" type="play" name="rosbag_play" args="-l /home/rosuser/ws/bag/acdc_fusion_guidance_XX__NOISE__XX.bag"/>
replace the 'XX__NOISE__XX' with the noise name for whichever noise distribution has been added to the bag file during 
    1. gaussian noise - "noise"
    2. laplcae noise - "laplace"
    3. cauchy noise - "cauchy"
    4. normal logarithmic noise - "lognormal"
    example: bag file laplace noise distribution - /home/rosuser/ws/bag/acdc_fusion_guidance_laplace.bag"

record bag file
    <node pkg="rosbag" type="record" name="rosbag_record" args="-O /home/rosuser/ws/bag/acdc_fusion_guidance_XX__NOISE_FILTER__XX.bag /sensors/fusion/ikaObjectList /sensors/reference/ikaObjectList"/>

replace the XX__NOISE_FILTER__XX with the name of the noise ditribution that was added to the bag file and the name of the filter that was implemented
    1. KF filter - "KF"
    2. EKF filter - "EKF"
    3. UKF filter = "UKF"

    example: EKF filter implemented on laplace noise distribution bag file - /home/rosuser/ws/bag/acdc_fusion_guidance_laplace_ekf.bag