This file desribes the steps to add noise to the bagfiles 
The acdc_fusion_guidance.bag is the ground truth. noise will be added to this bagfile. 

To execute the follwoing command for the addition of the noise to the bag file:
Please note that the term <noise> has to be replaced by the noise distributino that is ti be added.

```bash
rosrun rosbag_noise <noise>.py ~/ws/bag/bag/acdc_fusion_guidance.bag
```

For example if the gaussian noise is to be implemented, execute the following command 
```bash
rosrun rosbag_noise gaussian.py ~/ws/bag/bag/acdc_fusion_guidance.bag
```
