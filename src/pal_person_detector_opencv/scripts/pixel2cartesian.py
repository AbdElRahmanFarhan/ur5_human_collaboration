#!/usr/bin/env python
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import Pose
from pal_detection_msgs.msg import Detections2d
import ros_numpy

class Subscribers():
    def __init__(self):
        self.aligned_depth_img = None
        self.intrinsics = {'fx': 0, 'fy': 0, 'px': 0, 'py': 0, 'w': 0, 'h': 0}
        rospy.Subscriber(
            "/camera/depth/camera_info", CameraInfo, self.info_callback)
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

    def detections_callback(self, msg):
        #print("callback")
        depth_msg = rospy.wait_for_message(
            "/camera/depth/image_raw", Image)
        #print("received Image")
        dist_mat = self.calculate_dist_3D(depth_msg)

        pose_msg = PoseArray()
        pose_msg.header.frame_id = "camera_link"
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

            pose.orientation.w = 1
            pose.orientation.x = 0
            pose.orientation.y = 0
            pose.orientation.z = 0
            pose_msg.poses.append(pose)       
        
        self.px_to_xyz_pub.publish(pose_msg)

if __name__ == "__main__":
    rospy.init_node("dist3d_calculator")
    subs = Subscribers()
    rospy.spin()
