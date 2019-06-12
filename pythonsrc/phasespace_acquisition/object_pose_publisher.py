#!/usr/bin/python
from __future__ import division
import rospy, yaml, copy, math, numpy as np, os, tf
from catkin.find_in_workspaces import find_in_workspaces
from IPython import embed
from std_msgs.msg import String
import tf.transformations
from phasespace_acquisition.msg import PhaseSpaceMarker, PhaseSpaceMarkerArray

class ObjectPosePublisher(object):
    def __init__(self):
        # 1. Config
        base_path = find_in_workspaces(
            search_dirs=['share'],
            project="phasespace_acquisition",
            path='config',
            first_match_only=True)[0]
        path = os.path.join(base_path, '/tracking.yaml'))
        self.tracking_config = yaml.load(open(path))

        # 2. ROS listener
        self.phasespace_marker_array = None
        rospy.Subscriber(self.tracking_config["phasespace_topic"], PhaseSpaceMarkerArray, self.phasespace_callback)

        # 3. Transformation
        # Original origin is defined originally on the floor by Pem
        # for the phasespace system.
        # Now here we define a new origin, as shown on RVIZ as the
        # TF of the map.
        self.originalOrigin_2_newOrigin_mat = np.array([[0,1,0,0], [0,0,1,0], [1,0,0,0], [0,0,0,1]])

    def phasespace_callback(self, data):
        self.phasespace_marker_array = data

    def getNewOrigin2ObjTrans(self, frame_id):
        if self.phasespace_marker_array is None:
            return
        trans = []
        found = False
        for ps_obj in self.phasespace_marker_array.data:
            if frame_id == int(ps_obj.id):
                trans = [
                    ps_obj.point.x / 1000., ps_obj.point.y / 1000.,
                    ps_obj.point.z / 1000.
                ]
                assert (not found)
                found = True

        originalOrigin_2_obj_mat = tf.transformations.translation_matrix(trans)
        newOrigin_2_obj_mat = np.dot(np.linalg.inv(self.originalOrigin_2_newOrigin_mat), originalOrigin_2_obj_mat)
        return tf.transformations.translation_from_matrix(newOrigin_2_obj_mat)
