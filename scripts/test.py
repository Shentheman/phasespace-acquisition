#!/usr/bin/python
from __future__ import division
import rospy, yaml, copy, math, numpy as np, collections, os
from catkin.find_in_workspaces import find_in_workspaces
from IPython import embed
import std_msgs, tf
import itertools
import tf.transformations

from phasespace_acquisition.object_pose_publisher import ObjectPosePublisher 

def main():
    rospy.init_node('test', anonymous=True)
    # Make sure sim time is working
    while not rospy.Time.now():
        pass
    x = ObjectPosePublisher(1)

if __name__ == '__main__':
    main()
