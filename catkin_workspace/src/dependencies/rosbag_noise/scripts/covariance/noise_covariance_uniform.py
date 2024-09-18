#!/usr/bin/env python

#
#  ==============================================================================
#  MIT License
#
#  Copyright 2022 Institute for Automotive Engineering of RWTH Aachen University.
#
#  Permission is hereby granted, free of charge, to any person obtaining a copy
#  of this software and associated documentation files (the "Software"), to deal
#  in the Software without restriction, including without limitation the rights
#  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
#  copies of the Software, and to permit persons to whom the Software is
#  furnished to do so, subject to the following conditions:
#
#  The above copyright notice and this permission notice shall be included in all
#  copies or substantial portions of the Software.
#
#  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
#  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
#  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
#  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
#  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
#  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
#  SOFTWARE.
#  ==============================================================================

import random
import sys
from copy import deepcopy

import rosbag
from definitions.msg import IkaObjectList, IkaSensorStamp


# to execute this script, build the catkin package with its dependencies.
# then execute:
#  rosrun rosbag_noise main.py bag_file.bag

def make_noise(object_list, mode, noise_type='gaussian'):
    noise_list = IkaObjectList()
    noise_list.header = object_list.header

    if mode == 'camera':
        noise_list.IdSource = 16  # CAMERA
    elif mode == 'radar':
        noise_list.IdSource = 14  # RADAR

    for reference_object in object_list.objects:
        obj = deepcopy(reference_object)
        obj.fMean = list(obj.fMean)
        obj.fCovariance = list(obj.fCovariance)

        if noise_type == 'gaussian':
            if mode == 'camera':
                obj.fMean[0] += random.gauss(0, 0.6)
                obj.fMean[1] += random.gauss(0, 0.2)
                obj.fCovariance[0] = 0.6**2
                obj.fCovariance[12] = 0.2**2
            elif mode == 'radar':
                obj.fMean[0] += random.gauss(0, 0.2)
                obj.fMean[1] += random.gauss(0, 0.6)
                obj.fCovariance[0] = 0.2**2
                obj.fCovariance[12] = 0.6**2

        elif noise_type == 'uniform':
            if mode == 'camera':
                obj.fMean[0] += random.uniform(-0.6, 0.6)
                obj.fMean[1] += random.uniform(-0.2, 0.2)
                obj.fCovariance[0] = (0.6**2) / 3  # Variance for uniform distribution
                obj.fCovariance[12] = (0.2**2) / 3
            elif mode == 'radar':
                obj.fMean[0] += random.uniform(-0.2, 0.2)
                obj.fMean[1] += random.uniform(-0.6, 0.6)
                obj.fCovariance[0] = (0.2**2) / 3
                obj.fCovariance[12] = (0.6**2) / 3

        # Other parameters and covariances, leaving as is or with small constant noise
        for i in range(24, 121, 12):
            obj.fCovariance[i] = 0.01

        obj.IdType = 4  # CAR
        obj.IdExternal = noise_list.IdSource
        obj.bObjectMeasured = 1

        sensor_stamp = IkaSensorStamp()
        sensor_stamp.IdSensor = obj.IdExternal
        sensor_stamp.IdObjectWithinSensor = obj.IdInternal
        sensor_stamp.measuredStamp = object_list.header.stamp
        obj.measHist.append(sensor_stamp)

        noise_list.objects.append(obj)

    return noise_list


def main():
    input_name = sys.argv[1]
    input_bag = rosbag.Bag(input_name, 'r')
    input_objectlist_topic = "/fusion/ikaObjectList"
    input_objectlist_topic_output = "/sensors/reference/ikaObjectList"

    output_name = sys.argv[1][:-4] + '_noise.bag'
    bag = rosbag.Bag(output_name, 'w')
    output_objectlist_topic_camera = '/sensors/camera_front/ikaObjectList'
    output_objectlist_topic_radar = '/sensors/radar_front/ikaObjectList'

    try:
        for topic, msg, t in input_bag.read_messages():
            if topic == input_objectlist_topic:
                bag.write(input_objectlist_topic_output, msg, t)

                # Add Gaussian noise
                estimated_objects_camera_gaussian = make_noise(msg, 'camera', 'gaussian')
                bag.write(output_objectlist_topic_camera + "_gaussian", estimated_objects_camera_gaussian, t)

                estimated_objects_radar_gaussian = make_noise(msg, 'radar', 'gaussian')
                bag.write(output_objectlist_topic_radar + "_gaussian", estimated_objects_radar_gaussian, t)

                # Add Uniform noise
                estimated_objects_camera_uniform = make_noise(msg, 'camera', 'uniform')
                bag.write(output_objectlist_topic_camera + "_uniform", estimated_objects_camera_uniform, t)

                estimated_objects_radar_uniform = make_noise(msg, 'radar', 'uniform')
                bag.write(output_objectlist_topic_radar + "_uniform", estimated_objects_radar_uniform, t)
            else:
                bag.write(topic, msg, t)
    finally:
        bag.close()

    print("saved to " + output_name)


if __name__ == '__main__':
    main()
