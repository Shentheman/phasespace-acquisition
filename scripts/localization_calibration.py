#!/usr/bin/python
from __future__ import division
import rospy, yaml, copy, math, numpy as np, collections, os
from catkin.find_in_workspaces import find_in_workspaces
from IPython import embed
import std_msgs, tf
import itertools
import tf.transformations

from phasespace_acquisition.msg import PhaseSpaceMarker, PhaseSpaceMarkerArray

#  "phasespace/palm/left"
PHASESPACE_ID_FOR_CALIBRATION = 1

class LocalizationCalibration(object):
    def __init__(self):
        base_path = find_in_workspaces(
            search_dirs=['share'],
            project="phasespace_acquisition",
            path='config',
            first_match_only=True)
        if len(base_path) == 0:
            raise Exception("Unable to find base_path.")
        self.base_path_ = base_path[0]
        self.tracking_config_ = yaml.load(
            open(self.base_path_ + '/tracking.yaml'))

        # Based on http://wiki.ros.org/tf, 100ms (10hz) is a good value.
        self.rate_ = rospy.Rate(self.tracking_config_["publish_frequency"])

        self.listener_ = tf.TransformListener()

        self.tracked_collision_objects_\
                = self.tracking_config_["tracked_objects"]
        object_for_calibration = None
        for k,v in self.tracked_collision_objects_.iteritems():
            if v['phasespace_id'] == PHASESPACE_ID_FOR_CALIBRATION:
                object_for_calibration = v
        assert(object_for_calibration!=None)

        
        baseFrame_2_frameId_trans = []
        baseFrame_2_frameId_quat = []
        while not rospy.is_shutdown():
            try:
                (baseFrame_2_frameId_trans, baseFrame_2_frameId_quat)\
                        = self.listener_.lookupTransform(
                        object_for_calibration["base_frame"],
                        object_for_calibration["frame_id"],
                        rospy.Time(0))
                print baseFrame_2_frameId_trans
                #  Note that phasespace is not able to handle
                #  orientation. So we won't print orientation here
            except (tf.LookupException,
                    tf.ConnectivityException, tf.ExtrapolationException):
                continue
            self.rate_.sleep()

if __name__ == '__main__':
    rospy.init_node('LocalizationCalibration', anonymous=True)
    # Make sure sim time is working
    while not rospy.Time.now():
        pass
    x = LocalizationCalibration()
