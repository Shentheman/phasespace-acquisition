#!/usr/bin/python
from __future__ import division
import rospy, yaml, copy, math, numpy as np, collections, os
from catkin.find_in_workspaces import find_in_workspaces
from IPython import embed
import std_msgs, tf
import itertools
import tf.transformations

from phasespace_acquisition.msg import PhaseSpaceMarker, PhaseSpaceMarkerArray

AVG_IDS = [1,2]
AVG_FRAME_ID = "phasespace/palm"

class Msg2TFAvgPalm(object):
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

        self.publish_rate_ = rospy.Rate(
            self.tracking_config_["publish_frequency"])

        self.br_ = tf.TransformBroadcaster()

        self.tracked_collision_objects_\
                = self.tracking_config_["tracked_objects"]
        # Change key to int
        for k in self.tracked_collision_objects_.keys():
            self.tracked_collision_objects_[int(k)]\
                    = self.tracked_collision_objects_.pop(k)
        for k in AVG_IDS:
            assert(k in self.tracked_collision_objects_.keys())
 

        rospy.Subscriber(self.tracking_config_["phasespace_topic"],
                         PhaseSpaceMarkerArray, self.phasespace_callback)
        self.phasespace_marker_array = None

        # Original origin is defined originally on the floor by Pem
        # for the phasespace system.
        # Now here we define a new origin, as shown on RVIZ as the
        # TF of the map.
        """
        self.originalOrigin_2_newOrigin_trans = [0,0,0]
        self.originalOrigin_2_newOrigin_quat = [0,0,0,1]
        self.originalOrigin_2_newOrigin_mat\
                = tf.transformations.quaternion_matrix(
                        self.originalOrigin_2_newOrigin_quat)
        self.originalOrigin_2_newOrigin_mat[0,3]\
                = self.originalOrigin_2_newOrigin_trans[0]
        self.originalOrigin_2_newOrigin_mat[1,3]\
                = self.originalOrigin_2_newOrigin_trans[1]
        self.originalOrigin_2_newOrigin_mat[2,3]\
                = self.originalOrigin_2_newOrigin_trans[2]

        # Archived:
        #  self.originalOrigin_2_newOrigin_mat\
                #  = np.array([\
                #  [-1,0,0,0],\
                #  [0,0,1,0],\
                #  [0,1,0,0],\
                #  [0,0,0,1]])
        """

        self.originalOrigin_2_newOrigin_mat\
                = np.array([\
                [0,1,0,0],\
                [0,0,1,0],\
                [1,0,0,0],\
                [0,0,0,1]])

    def phasespace_callback(self, data):
        self.phasespace_marker_array = data

    def publish_tf(self):
        while not rospy.is_shutdown():
            if self.phasespace_marker_array != None:
                transs = []
                for ps_obj in self.phasespace_marker_array.data:
                    k = int(ps_obj.id)
                    if k in AVG_IDS:
                        obj = self.tracked_collision_objects_[k]
                        trans = [
                            ps_obj.point.x / 1000., ps_obj.point.y / 1000.,
                            ps_obj.point.z / 1000.
                        ]
                        transs.append(trans)
                avg_trans = [0,0,0]
                for i in range(3):
                    l = [x[i] for x in transs]
                    avg_trans[i] = sum(l) / len(l)
                quat = [0,0,0,1]
                originalOrigin_2_obj_mat\
                        = tf.transformations.translation_matrix(trans)
                newOrigin_2_obj_mat = np.dot(
                    np.linalg.inv(self.originalOrigin_2_newOrigin_mat),
                    originalOrigin_2_obj_mat)
                self.br_.sendTransform(
                    tf.transformations.translation_from_matrix(
                        newOrigin_2_obj_mat),
                    tf.transformations.quaternion_from_matrix(
                        newOrigin_2_obj_mat), rospy.Time.now(),
                    AVG_FRAME_ID, obj["base_frame"])

            self.publish_rate_.sleep()


if __name__ == '__main__':
    rospy.init_node('Msg2TFAvgPalm', anonymous=True)
    # Make sure sim time is working
    while not rospy.Time.now():
        pass
    x = Msg2TFAvgPalm()
    x.publish_tf()
