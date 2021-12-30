#!/usr/bin/env python3

from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import DeleteModelRequest, DeleteModel, SetLinkProperties, SetLinkPropertiesRequest

import rospy
from math import cos

def shutdown_func():
    print ("1")
    del_req = DeleteModelRequest(model_name = "cylinder")
    del_client(del_req)
    print ("2")

def up_down(distance=0.2, start_position=[0.5, 0, 1.3]):
    move_msg = ModelState()
    move_msg.model_name = "cylinder"
    move_msg.reference_frame = "world"
    move_msg.pose.position.x = start_position[0]
    move_msg.pose.position.y = start_position[1]
    i = 0

    rospy.on_shutdown(shutdown_func)
    hz = 50.
    rate = rospy.Rate(hz)
    while not rospy.is_shutdown():
        move_msg.pose.position.z = cos(i) * distance/2.0 + start_position[2]
        move_pub.publish(move_msg)
        i += 1/hz
        rate.sleep()
        
def left_right(distance=0.2, start_position=[0.5, 0, 1.3]):
    move_msg = ModelState()
    move_msg.model_name = "cylinder"
    move_msg.reference_frame = "world"
    move_msg.pose.position.x = start_position[0]
    move_msg.pose.position.z = start_position[2]
    i = 0

    rospy.on_shutdown(shutdown_func)
    hz = 50.
    rate = rospy.Rate(hz)
    while not rospy.is_shutdown():
        move_msg.pose.position.y = cos(i) * distance/2.0 + start_position[1]
        move_pub.publish(move_msg)
        i += 1/hz
        rate.sleep()

rospy.init_node("object_mover")
rospy.sleep(3)
move_pub = rospy.Publisher("/gazebo/set_model_state", ModelState, queue_size=1)
del_client = rospy.ServiceProxy("/gazebo/delete_model", DeleteModel)
del_client.wait_for_service()
rospy.sleep(3)
# up_down()
left_right()


