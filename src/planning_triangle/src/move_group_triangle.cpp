//
// Created by renno on 19.05.17.
//

#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>

int main(int argc, char **argv)
{
    //Setup

    ros::init(argc, argv, "move_group_triangle");
    ros::NodeHandle node_handle;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    sleep(1.0); //Waits 5 sec before starting a program

    moveit::planning_interface::MoveGroup group("sia5");

    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    moveit::planning_interface::MoveGroup::Plan my_plan;

    ros::Publisher display_publisher = node_handle.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
    moveit_msgs::DisplayTrajectory display_trajectory;

    //Getting basic information

    ROS_INFO("Reference frame: %s", group.getPlanningFrame().c_str());

    ROS_INFO("Reference frame: %s", group.getEndEffectorLink().c_str());

    //Cartesian paths
    std::vector<geometry_msgs::Pose> waypoints;

    geometry_msgs::Pose target_pose3 = group.getCurrentPose().pose;

    target_pose3.position.z -= 0.3;
    target_pose3.position.y -= 0.3;
    waypoints.push_back(target_pose3);  // down and left

    target_pose3.position.y += 0.6;
    waypoints.push_back(target_pose3);  // right

    target_pose3.position.y -= 0.3;
    target_pose3.position.z += 0.3;
    waypoints.push_back(target_pose3); // up and left

    moveit_msgs::RobotTrajectory trajectory;
    double fraction = group.computeCartesianPath(waypoints,
                                                 0.01,  // eef_step
                                                 0.0,   // jump_threshold
                                                 trajectory);
   
   my_plan.trajectory_ = trajectory;

    ROS_INFO("Visualizing plan 4 (cartesian path) (%.2f%% acheived)",
             fraction * 100.0);
    sleep(10);

    
   ROS_INFO("MOVING");
   group.execute(my_plan);
   sleep(5);

    ros::shutdown();
    return 0;
}