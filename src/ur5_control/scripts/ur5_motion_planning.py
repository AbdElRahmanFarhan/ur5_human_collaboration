#!/usr/bin/env python3

import rospy
from moveit_commander import MoveGroupCommander, RobotCommander, PlanningSceneInterface
from sensor_msgs.msg import JointState
from math import pi, cos
from ur5_control.Ur5Robot import *
from moveit_msgs.msg import PlanningScene
from gazebo_msgs.srv import DeleteModelRequest, DeleteModel, DeleteModelResponse

# initialize ros node
rospy.init_node("motion_planning")
planner_type = rospy.get_param("planner_type")
model_client = rospy.ServiceProxy("/gazebo/delete_model", DeleteModel)
rospy.sleep(5)

# initialize the robot driver
ur5 = Ur5Robot(planner_id=planner_type)
ur5.go_to_joint_goal([0, 0, 0, 0, 0, 0])

"""
Object to pick
"""
object_pose = Pose()
object_pose.position.x = 0.1
object_pose.position.y = -0.6
object_pose.position.z = 1.010115
object_pose.orientation.x = 0
object_pose.orientation.y = 0
object_pose.orientation.z = 0
object_pose.orientation.w = 1
object_h = 0.05

"""
Picking
"""
# distances for approach and retreat when picking
pick_approach = 0.05
pick_retreat = 0.1
contact_margin = 0.04
#  pick location
pick_pose = [object_pose.position.x, object_pose.position.y, object_pose.position.z + object_h/2.0 + contact_margin + pick_approach, 0, 0, 0]
# pick_pose = [0.5, 0.1, 1.13 + pick_approach, 0, 0, 0 ]

# go to pick location
ur5.go_to_pose_goal(pick_pose)
rospy.sleep(2)

# approach the object, pick it and then retreat.
ur5.pick_object([0, 0, -pick_approach], [0, 0, pick_retreat])
rospy.sleep(2)

"""
Placing
"""
ur5.go_to_joint_goal([pi, 0, 0, 0, 0, 0])
# distances for approach and retreat when placing
place_approach = 0.05
place_retreat = 0.1

# the place location
# here we are defining the place location with respect to the pick location
place_pose = pick_pose
place_pose[1] = -pick_pose[1]
place_pose[2] = object_pose.position.z + object_h/2.0 + contact_margin + place_approach

# go to the place location
rospy.sleep(2)

ur5.go_to_pose_goal(place_pose)
rospy.sleep(2)

# place the object
ur5.place_object([0, 0, -place_approach], [0, 0, place_retreat])

rospy.sleep(2)

# return home
ur5.go_to_joint_goal([0, 0, 0, 0, 0, 0])

# delete model
req = DeleteModelRequest()
req.model_name = "gear_part"
res = DeleteModelResponse()
model_client.wait_for_service()
res = model_client(req)