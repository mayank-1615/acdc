import rosbag
import numpy as np
import matplotlib.pyplot as plt
# This file modifies for CTRA motion model. 
# only use this for EKF/UKF filters
# The input and outpat paths of the files have to be mdified accordingly for diffreent filters that have to be imlemented with the respective noise

# input path = always from /Bagfiles_original/KF_bagfiles/ directory
# output path = always to /Bagfiles_original/EUKF_bagfiles/ directory
# input_bag_path = 'Bagfiles_original/KF_bagfiles/acdc_fusion_guidance_gaussian_noise.bag'
# output_bag_path = 'Bagfiles_original/EUKF_bagfiles/acdc_fusion_guidance_gaussian_noise.bag'

# For the sake of the report we have implemented the gaussian noise for EKF
input_bag_path = 'Bagfiles_original/KF_bagfiles/acdc_fusion_guidance_gaussian_noise.bag'
output_bag_path = 'Bagfiles_original/EUKF_bagfiles/acdc_fusion_guidance_gaussian_noise.bag'
delta_t = 0.05  # 50ms time step (20 Hz)
yawrate_limit = 0.8  # Define the yaw rate limit

# absolute value from relative values in x and y
def calculate_abs_from_rel(X, Y):
    return np.sqrt(X**2 + Y**2)

# Function to calculate yaw rate and apply limit
def calculate_yawrate(prev_heading, curr_heading, delta_t, yawrate_limit):
    yawrate = (curr_heading - prev_heading) / delta_t
    # Apply the yawrate limit
    yawrate = np.clip(yawrate, -yawrate_limit, yawrate_limit)
    return yawrate

# Dictionary to store previous headings and yaw rates for plotting
prev_heading = {}
current_heading = {}
yawrate_data = {}  # To store yaw rate for each object wrt time
time_data = []  # To store time for plotting

# Topics of interest
topics_of_interest = ["/sensors/reference/ikaObjectList", "/sensors/camera_front/ikaObjectList", "/sensors/radar_front/ikaObjectList"]

with rosbag.Bag(output_bag_path, 'w') as outbag:
    with rosbag.Bag(input_bag_path, 'r') as bag:
        for topic, msg, t in bag.read_messages():
            # Time in seconds
            time_data.append(t.to_sec())

            if topic in topics_of_interest:
                print(f"Processing topic: {topic}, time: {t.to_sec()}")
                for obj in msg.objects:
                    obj.IdMotionModel = 4
                    fMean = obj.fMean
                    if len(fMean) == 11:
                        # Create a unique object ID by combining topic and IdInternal
                        object_id = f"{topic}_{obj.IdInternal}"
                        current_heading[object_id] = fMean[10]

                        # Determine yawrate
                        if obj.bObjectNew or object_id not in prev_heading:
                            yawrate = 0
                        else:
                            yawrate = calculate_yawrate(prev_heading[object_id], current_heading[object_id], delta_t, yawrate_limit)

                        # Store yawrate for plotting
                        if object_id not in yawrate_data:
                            yawrate_data[object_id] = []
                        yawrate_data[object_id].append(yawrate)

                        # Update previous heading
                        prev_heading[object_id] = current_heading[object_id]
                        
                        absAccX = fMean[5]
                        absAccY = fMean[6]
                        absVelX = fMean[3]
                        absVelY = fMean[4]
                        abs_acc = calculate_abs_from_rel(absAccX, absAccY)
                        abs_vel = calculate_abs_from_rel(absVelX, absVelY)

                        # Create new fMean based on CTRA model
                        new_fMean = [
                            fMean[0],  # posX
                            fMean[1],  # posY
                            fMean[2],  # posZ
                            abs_vel,   # absVel
                            abs_acc,   # absAcc
                            fMean[10], # heading
                            yawrate,   # yawrate 
                            fMean[7],  # length
                            fMean[8],  # width
                            fMean[9]   # height
                        ]
                            
                        # Update the fMean with the new yawrate
                        # new_fMean = (fMean[0], fMean[1], fMean[2], fMean[3], fMean[4], fMean[5], fMean[6], fMean[7], fMean[8], fMean[9], fMean[10], yawrate)
                        obj.fMean = new_fMean
                    fCovariance = obj.fCovariance
                    if len(fCovariance) == 121:
                        # Create new fCovariance based on CTRA model
                        new_fCovariance = [0] * 100
                        new_fCovariance[0] = fCovariance[0]       # posX
                        new_fCovariance[11] = fCovariance[12]     # posY
                        new_fCovariance[22] = fCovariance[24]     # posZ
                        
                        # Conditional assignment for fCovariance[33]
                        if fMean[3] <= 0 or fMean[4] <= 0:
                            new_fCovariance[33] = -1  # Assign only the 33rd element to -1
                        else:
                            new_fCovariance[33] = calculate_abs_from_rel(fCovariance[36], fCovariance[48])  # absVel
                        
                        # Conditional assignment for fCovariance[44]
                        if fMean[5] <= 0 or fMean[6] <= 0:
                            new_fCovariance[44] = -1  # Assign only the 44th element to -1
                        else:
                            new_fCovariance[44] = calculate_abs_from_rel(fCovariance[60], fCovariance[72])  # absAccl
                        
                        new_fCovariance[55] = fCovariance[120]    # heading
                        new_fCovariance[66] = 0.01                  # keep it fixed as -1
                        new_fCovariance[77] = fCovariance[84]     # length
                        new_fCovariance[88] = fCovariance[96]     # width
                        new_fCovariance[99] = fCovariance[108]    # height

                        # Update the fCovariance in the message
                        obj.fCovariance = new_fCovariance    

                    else:
                        print(f"Object in topic {topic} does not have expected fMean size of 11, skipping object")


            outbag.write(topic, msg, t)

# Plotting yaw rate for all objects with respect to time
# plt.figure(figsize=(10, 6))

# for object_id, yawrates in yawrate_data.items():
#    plt.plot(time_data[:len(yawrates)], yawrates, label=f'Object {object_id}')

# plt.title('Yaw Rate for all objects over time')
# plt.xlabel('Time (seconds)')
# plt.ylabel('Yaw Rate (rad/s)')
# plt.legend()
# plt.grid(True)
# plt.show()




