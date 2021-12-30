#!/usr/bin/env python3

import rospy
from moveit_commander import MoveGroupCommander, RobotCommander, PlanningSceneInterface
from sensor_msgs.msg import JointState
from math import pi, cos
from ur5_control.Ur5Robot import *
from moveit_msgs.msg import PlanningScene

# initialize ros node
rospy.init_node("motion_planning")
planner_type = rospy.get_param("planner_type")
rospy.sleep(1)

# initialize the robot driver
ur5 = Ur5Robot(planner_id=planner_type)
ur5.go_to_joint_goal([-pi/2.0, 0, 0, 0, 0, 0])

"""
Search for the object of interest
"""
# decide the object to be picked
object_name = "gear_part"
object_found = False

object_msg = rospy.wait_for_message("/move_group/monitored_planning_scene", PlanningScene)
sz = len(object_msg.world.collision_objects)
while  sz == 0:
    object_msg = rospy.wait_for_message("/move_group/monitored_planning_scene", PlanningScene)
    sz = len(object_msg.world.collision_objects)
    rospy.sleep(1)

for i in range(sz):
    if object_msg.world.collision_objects[i].id == object_name:
        [object_w, object_l, object_h] = object_msg.world.collision_objects[i].primitives[0].dimensions
        object_pose = object_msg.world.collision_objects[i].primitive_poses[0]
        object_found = True

if not object_found:
    raise ValueError("cannot find the object of interest in the planning scene")

"""
Pick and PLace
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
rospy.sleep(1)

# approach the object, pick it and then retreat.
ur5.pick_object(object_name, [0, 0, -pick_approach], [0, 0, pick_retreat])

# distances for approach and retreat when placing
place_approach = 0.05
place_retreat = 0.1

# the place location
# here we are defining the place location with respect to the pick location
place_pose = pick_pose
place_pose[1] = pick_pose[1] + 0.4
place_pose[2] = object_pose.position.z + object_h/2.0 + contact_margin + place_approach

# go to the place location
ur5.go_to_pose_goal(place_pose)
rospy.sleep(1)

# place the object
ur5.place_object(object_name, [0, 0, -place_approach], [0, 0, place_retreat])
