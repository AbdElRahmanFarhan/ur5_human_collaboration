#!/usr/bin/env python

import rospy
import numpy as np

from moveit_commander import MoveGroupCommander
from moveit_msgs.msg import RobotTrajectory
from geometry_msgs.msg import WrenchStamped, Pose
from std_srvs.srv import Empty, EmptyRequest
from std_msgs.msg import Bool
from sensor_msgs.msg import JointState


class Ur5Robot:

    def __init__(self, planner_id="RRT"):
        # initialize moveit group
        self.__GROUP = "arm"
        self.___PLANNING_JOINT_NAMES = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
                                        "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]
        self.__group_commander = MoveGroupCommander(self.__GROUP)
        self.__group_commander.set_planner_id(planner_id)
        self.__group_commander.allow_replanning(True)
        # self.__group_commander.allow_looking(True)

        # initialize vacuum services
        self.__vacuum_on_service = rospy.ServiceProxy("/ur5_gripper/on", Empty)
        self.__vacuum_off_service = rospy.ServiceProxy("/ur5_gripper/off", Empty)

        self.__planning_attempts = 2
        
    """ 
    Turn on Vacuum gripper
    """
    def vacuum_gripper_on(self):
        self.__vacuum_on_service.wait_for_service(0.2)
        request = EmptyRequest()
        self.__vacuum_on_service(request)

    """ 
    Turn off Vacuum gripper
    """
    def vacuum_gripper_off(self):
        self.__vacuum_off_service.wait_for_service(0.2)
        request = EmptyRequest()
        self.__vacuum_off_service(request)

    """
    move in a straight line with respect to the tool reference frame
    :param pose_goal list of [x, y, z] displacement of the tools with respect to its frame
    """
    def move_tool_in_straight_line(self, pose_goal, avoid_collisions=True, n_way_points=2):

        way_points_list = []
        way_point = Pose()
        x_way_points = np.linspace(0, pose_goal[0], n_way_points)
        y_way_points = np.linspace(0, pose_goal[1], n_way_points)
        z_way_points = np.linspace(0, pose_goal[2], n_way_points)
        qx_way_points = np.linspace(0, 0, n_way_points)
        qy_way_points = np.linspace(0, 0, n_way_points)
        qz_way_points = np.linspace(0, 0, n_way_points)
        qw_way_points = np.linspace(0, 0, n_way_points)
        for i in range(n_way_points):
            way_point.position.x = x_way_points[i]
            way_point.position.y = y_way_points[i]
            way_point.position.z = z_way_points[i]
            way_point.orientation.x = qx_way_points[i]
            way_point.orientation.y = qy_way_points[i]
            way_point.orientation.z = qz_way_points[i]
            way_point.orientation.w = qw_way_points[i]
            way_points_list.append(way_point)

        self.__group_commander.set_pose_reference_frame(self.__group_commander.get_end_effector_link())

        success = False

        for i in range(self.__planning_attempts):
            plan, fraction = self.__group_commander.compute_cartesian_path(way_points_list, 0.005, 0.0, avoid_collisions)

            # post processing for the trajectory to ensure its validity 
            last_time_step = plan.joint_trajectory.points[0].time_from_start.to_sec
            new_plan = RobotTrajectory()
            new_plan.joint_trajectory.header = plan.joint_trajectory.header
            new_plan.joint_trajectory.joint_names = plan.joint_trajectory.joint_names
            new_plan.joint_trajectory.points.append(plan.joint_trajectory.points[0])

            for i in range(1, len(plan.joint_trajectory.points)):
                point = plan.joint_trajectory.points[i]
                if point.time_from_start.to_sec > last_time_step:
                    new_plan.joint_trajectory.points.append(point)
                last_time_step = point.time_from_start.to_sec

            # execute the trajectory
            success = self.__group_commander.execute(new_plan)
            if success:
                break

        if not success:   
            raise ValueError("robot cannot move in a straight line")

    """
    Go to pose goal. Plan and execute and wait for the controller response
    :param pose_goal list in this form [x, y, z, r, p, y]
     or [x, y, z, qx, qy, qz, qw]
     or [x, y, z]
    :type list of int
    :param ef_link
    :type: string
    """
    def go_to_pose_goal(self, pose_goal):
        # set the pose reference frame
        self.__group_commander.set_pose_reference_frame("world")

        if len(pose_goal) == 6 or len(pose_goal) == 7:
            self.__group_commander.set_pose_target(pose_goal)

        elif len(pose_goal) == 3:
            self.__group_commander.set_position_target(pose_goal)

        else:
            raise ValueError('Invalid inputs to pose goal')

        # plan to the goal then execute the plan
        sucess = False
        for i in range(self.__planning_attempts):
            success = self.__group_commander.go()
            if success:
                break

        if not success:
            raise ValueError('robot cannot go to the pose goal')

    """
    Go to joint goal
    """
    def go_to_joint_goal(self, joint_goal):

        if len(joint_goal) == 6:
            joint_goal_msg = JointState()
            joint_goal_msg.name = self.___PLANNING_JOINT_NAMES
            joint_goal_msg.position = joint_goal
            self.__group_commander.set_joint_value_target(joint_goal_msg)

        else:
            raise ValueError('Invalid inputs to joint goal')

        success = False
        for i in range(self.__planning_attempts):
            success = self.__group_commander.go()
            if success:
                break

        if not success:
            raise ValueError('robot cannot go to the pose goal')

    """
    pick an object 
    This function assumes that the gripper is already in a place near to the object.
     With an appropriate orientation.
    So what is left in order to pick. is to approach the object. turn  gripper on and then retreat.
    """
    def pick_object(self, object_name, approach_dist, retreat_dist):

        # approach the object
        self.move_tool_in_straight_line(approach_dist, avoid_collisions=True)
        rospy.sleep(1)

        # turn on the vacuum gripper to pick the object
        self.vacuum_gripper_on()
        rospy.sleep(1)

        # retreat 
        self.move_tool_in_straight_line(retreat_dist, avoid_collisions=True)

    """
    Place an object using the vacuum gripper
    """
    def place_object(self, object_name, approach_dist, retreat_dist):

        # approach the table
        self.move_tool_in_straight_line(approach_dist, avoid_collisions=True)
        rospy.sleep(1)
        
        # turn off the vacuum gripper to place the object
        self.vacuum_gripper_off() 
        rospy.sleep(1)

        # retreat 
        self.move_tool_in_straight_line(retreat_dist, avoid_collisions=True)
