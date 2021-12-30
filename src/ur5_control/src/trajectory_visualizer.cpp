#include "ros/ros.h"
#include "moveit/move_group_interface/move_group_interface.h"
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

class trajectory_visualizer
{
private:
    ros::Subscriber sub;
    moveit_visual_tools::MoveItVisualTools visual_tools;
    moveit::planning_interface::MoveGroupInterface group;
    planning_scene_monitor::PlanningSceneMonitorPtr scene;

public:
    trajectory_visualizer(ros::NodeHandle *nh):
    visual_tools(moveit_visual_tools::MoveItVisualTools("world")),
    group(moveit::planning_interface::MoveGroupInterface("arm"))
    {
        sub = nh->subscribe("move_group/display_planned_path", 1, &trajectory_visualizer::trajectory_callback, this);
    }

    void trajectory_callback(const moveit_msgs::DisplayTrajectory& traj_msg)
    {
        visual_tools.deleteAllMarkers();
        visual_tools.publishTrajectoryLine(traj_msg.trajectory[0], group.getRobotModel()->getLinkModel("vacuum_ef"), group.getRobotModel()->getJointModelGroup("arm"));
        visual_tools.trigger();
    }
};

int main(int argc, char **argv)
{
    // initialize node
    ros::init(argc, argv, "talker");

    // node handle object
    ros::NodeHandle nh;

    // trajectory visualizer object
    trajectory_visualizer traj_vis(&nh);

    ros::spin();
    return 0;
}