#!/usr/bin/env python3
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import Pose, PoseStamped
from pal_detection_msgs.msg import Detections2d
import ros_numpy
import tf

class Subscribers():
    def __init__(self):
        self.aligned_depth_img = None
        self.transformer_listener = tf.TransformListener()
        self.intrinsics = {'fx': 0, 'fy': 0, 'px': 0, 'py': 0, 'w': 0, 'h': 0}
        rospy.Subscriber(
            "/camera1/depth_registered/camera_info", CameraInfo, self.info_callback)
        rospy.Subscriber("/person_detector/detections", Detections2d,
                         self.detections_callback)
        self.px_to_xyz_pub = rospy.Publisher("/px_to_xyz", PoseArray, queue_size=1)

    def calculate_dist_3D(self, depth_msg):
        depth_image = ros_numpy.numpify(depth_msg)
        index_mat = np.indices(depth_image.shape)
#       print(index_mat.shape)
        dist_mat = np.zeros((3, self.intrinsics['h'], self.intrinsics['w']))
        dist_mat[0] = (index_mat[0] - self.intrinsics['py']) * \
            depth_image / self.intrinsics['fy']
        dist_mat[1] = (index_mat[1] - self.intrinsics['px']) * \
            depth_image / self.intrinsics['fx']
        dist_mat[2] = depth_image
        return dist_mat
    

    def info_callback(self, msg):
        self.intrinsics['fx'] = msg.K[0]
        self.intrinsics['fy'] = msg.K[4]
        self.intrinsics['px'] = msg.K[2]
        self.intrinsics['py'] = msg.K[5]
        self.intrinsics['w'] = msg.width
        self.intrinsics['h'] = msg.height

    def transform_poses(self, target_frame, source_frame, pose_arr):
        """
        Transform poses from source_frame to target_frame
        """
        trans_pose_arr = PoseArray()
        for i in range(len(pose_arr.poses)):
            trans_pose = PoseStamped()
            pose = PoseStamped()
            pose.header.frame_id = source_frame
            pose.pose = pose_arr.poses[i]
            self.transformer_listener.waitForTransform(
                target_frame, source_frame, rospy.Time(), rospy.Duration(1))
            trans_pose = self.transformer_listener.transformPose(
                target_frame, pose)
            trans_pose_arr.poses.append(trans_pose.pose)

        trans_pose_arr.header.frame_id = target_frame
        trans_pose_arr.header.stamp = rospy.Time()
        return trans_pose_arr
    
    def detections_callback(self, msg):
        #print("callback")
        depth_msg = rospy.wait_for_message(
            "/camera1/depth_registered/image_raw", Image)
        #print("received Image")
        dist_mat = self.calculate_dist_3D(depth_msg)

        pose_msg = PoseArray()
        pose_msg.header.frame_id = "camera1_kinect_depth_optical_frame"
        pose_msg.header.stamp = rospy.Time.now()
        pose_msg.poses = []

        for single_detection in msg.detections:
            single_x = single_detection.x + (single_detection.width//2)
            single_y = single_detection.y + (single_detection.height//2)
            pose = Pose()
            # pose.position.x = dist_mat[1, single_x, single_y]
            # pose.position.y = dist_mat[0, single_x, single_y]
            # pose.position.z = dist_mat[2, single_x, single_y]
            pose.position.x = dist_mat[1, single_y, single_x]
            pose.position.y = dist_mat[0, single_y, single_x]
            pose.position.z = dist_mat[2, single_y, single_x]
            pose.position.x /= 1000.0
            pose.position.y /= 1000.0
            pose.orientation.w = 1
            pose.orientation.x = 0
            pose.orientation.y = 0
            pose.orientation.z = 0
            pose_msg.poses.append(pose) 
        pub_msg = PoseArray()
        pub_msg = self.transform_poses("base_link", "camera1_kinect_depth_optical_frame", pose_msg)     
        self.px_to_xyz_pub.publish(pub_msg)

if __name__ == "__main__":
    rospy.init_node("dist3d_calculator")
    subs = Subscribers()
    rospy.spin()
