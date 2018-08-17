#!/usr/bin/python
import roslib, rospy, tf, yaml, numpy, argparse, sys, copy
from tf2_msgs.msg import TFMessage
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, Quaternion, Point, TransformStamped
from IPython import embed
from catkin.find_in_workspaces import find_in_workspaces


class FakeLocalizer(object):
    def __init__(self):
        base_path = find_in_workspaces(
            search_dirs=['share'],
            project="phasespace_acquisition",
            path='config',
            first_match_only=True)
        if len(base_path) == 0:
            raise Exception("Unable to find base_path.")
        self.base_path_ = base_path[0]

        self.localization_config_ = yaml.load(
            open(self.base_path_ + '/fake_localization.yaml'))

        # Based on http://wiki.ros.org/tf, 100ms (10hz) is a good value.
        self._rate = rospy.Rate(self.localization_config_["publish_frequency"])

        # We need to publish the tf bw /map and /odom_combined
        # to /base_pose_ground_truth as nav_msgs/Odometry,
        # so that fake_localization can use it to localize the robot base.
        self._vOdom_pub = rospy.Publisher(
            '/base_pose_ground_truth', Odometry, queue_size=10)
        self._vOdom = Odometry()
        # self._vOdom.header.frame_id = "map"
        # self._vOdom.child_frame_id = "odom_combined"

        self.map_2_odomCombined_trans = self.localization_config_[
            "map_2_odomCombined_trans"]
        self.map_2_odomCombined_quat = self.localization_config_[
            "map_2_odomCombined_quat"]

    def run(self):
        rospy.loginfo("[FakeLocalizer.run] Start running!")
        while not rospy.is_shutdown():
            self._vOdom.header.stamp = rospy.Time.now()
            self._vOdom.pose.pose.position.x\
                    = self.map_2_odomCombined_trans[0]
            self._vOdom.pose.pose.position.y\
                    = self.map_2_odomCombined_trans[1]
            self._vOdom.pose.pose.position.z\
                    = self.map_2_odomCombined_trans[2]
            self._vOdom.pose.pose.orientation.x\
                    = self.map_2_odomCombined_quat[0]
            self._vOdom.pose.pose.orientation.y\
                    = self.map_2_odomCombined_quat[1]
            self._vOdom.pose.pose.orientation.z\
                    = self.map_2_odomCombined_quat[2]
            self._vOdom.pose.pose.orientation.w\
                    = self.map_2_odomCombined_quat[3]
            self._vOdom_pub.publish(self._vOdom)

            self._rate.sleep()


if __name__ == "__main__":
    rospy.init_node("FakeLocalizer")
    # Make sure sim time is working
    while not rospy.Time.now():
        pass
    x = FakeLocalizer()
    x.run()
