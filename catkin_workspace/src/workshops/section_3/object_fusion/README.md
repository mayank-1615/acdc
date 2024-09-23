Contents of this directory include the source codes for the fitlering techniques that have been used for purpose of this project. Currently the source code for EKF is being implemented. 


**Fusion Parameters**: Change the parameter file for fusion for the respective filter. 
parameter file location: *object_fusion_wrapper* node.
~/acdc/catkin_workspace/workshops/section_3/object_fusion_wrapper/param/fusion.yaml

//      existence_probability_output_threshold: 0.90   # 0.95 for UKF, 0.9 for EKF/KF
//      existence_probability_delete_threshold: 0.90   # 0.95 for UKF, 0.9 for EKF/KF

The following directories include the source code for repsective filters:

1. Task: Fusion
    **/src/modules/fuser/source/filters/_filter_/StateFuser.cpp**
    
    For the filter that is to be implemented replace the source code of the respective filter from this directory at the workspace where code will be sourced from in the *object_fusion* package
    For instance if you want to implement the EKF, replace the following file 
     **/src/modules/fuser/source/filters/StateFuser.cpp** file with **/src/modules/fuser/source/filters/EKF/StateFuser.cpp**

2. Task: State Prediction
    **/src/modules/Predictor/source/filters/_filter_/StatePredictor.cpp**
    
    For the filter that is to be implemented replace the source code of the respective filter from this directory at the workspace where code will be sourced from in the *object_fusion* package 
    For instance if you want to implement the EKF, replace the following file 
     **/src/modules/Predictor/source/filters/StatePredictor.cpp** with **/src/modules/Predictor/source/filters/EKF/StatePredictor.cpp**
