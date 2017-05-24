//
// Created by renno on 19.05.17.
//

#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

int main(int argc, char **argv)
{
    //setup

    ros::init(argc, argv, "move_group_interface_tutorial");
    ros::NodeHandle node_handle;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    sleep(5.0);

    moveit::planning_interface::MoveGroup group("right_arm");

    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    ros::Publisher display_publisher = node_handle.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
    moveit_msgs::DisplayTrajectory display_trajectory;

    //Getting basic information

    ROS_INFO("Reference frame: %s", group.getPlanningFrame().c_str());

    ROS_INFO("Reference frame: %s", group.getEndEffectorLink().c_str());

    //Planning to pose goal
    //Plan a motion for this group to a desired pose for the end-effector.
    geometry_msgs::Pose target_pose1;
    target_pose1.orientation.w = 1.0;
    target_pose1.position.x = 0.28;
    target_pose1.position.y = -0.7;
    target_pose1.position.z = 1.0;
    group.setPoseTarget(target_pose1);

    //We call the planner to compute the plan and visualize it. Just planning not moving.
    moveit::planning_interface::MoveGroup::Plan my_plan;
    bool success = group.plan(my_plan);

    ROS_INFO("Visualizing plan 1 (pose goal) %s",success?"":"FAILED");
    /* Sleep to give Rviz time to visualize the plan. */
    sleep(5.0);

    //Visualizing plans

    if (1)
    {
        ROS_INFO("Visualizing plan 1 (again)");
        display_trajectory.trajectory_start = my_plan.start_state_;
        display_trajectory.trajectory.push_back(my_plan.trajectory_);
        display_publisher.publish(display_trajectory);
        /* Sleep to give Rviz time to visualize the plan. */
        sleep(5.0);
    }

    //Moving to a pose goal

    /* Uncomment below line when working with a real robot*/
    /* group.move() */

    //Planning to a joint-space goal

   // Let’s set a joint space goal and move towards it. This will replace the pose target we set above.

    //modify one of the joints, plan to the new joint space goal and visualize the plan.
    std::vector<double> group_variable_values;
    group.getCurrentState()->copyJointGroupPositions(group.getCurrentState()->getRobotModel()->getJointModelGroup(group.getName()), group_variable_values);

    group_variable_values[0] = -1.0;
    group.setJointValueTarget(group_variable_values);
    success = group.plan(my_plan);

    ROS_INFO("Visualizing plan 2 (joint space goal) %s",success?"":"FAILED");
    /* Sleep to give Rviz time to visualize the plan. */
    sleep(5.0);

    //Planning with Path Constraints

    moveit_msgs::OrientationConstraint ocm;
    ocm.link_name = "r_wrist_roll_link";
    ocm.header.frame_id = "base_link";
    ocm.orientation.w = 1.0;
    ocm.absolute_x_axis_tolerance = 0.1;
    ocm.absolute_y_axis_tolerance = 0.1;
    ocm.absolute_z_axis_tolerance = 0.1;
    ocm.weight = 1.0;

    //Set it as the path constraint for the group.
    moveit_msgs::Constraints test_constraints;
    test_constraints.orientation_constraints.push_back(ocm);
    group.setPathConstraints(test_constraints);

    /*We will reuse the old goal that we had and plan to it. Note that this will only work if the current state already satisfies the path constraints.
     * So, we need to set the start state to a new pose.*/
    robot_state::RobotState start_state(*group.getCurrentState());
    geometry_msgs::Pose start_pose2;
    start_pose2.orientation.w = 1.0;
    start_pose2.position.x = 0.55;
    start_pose2.position.y = -0.05;
    start_pose2.position.z = 0.8;
    const robot_state::JointModelGroup *joint_model_group =
            start_state.getJointModelGroup(group.getName());
    start_state.setFromIK(joint_model_group, start_pose2);
    group.setStartState(start_state);

    //Now we will plan to the earlier pose target from the new start state that we have just created.
    group.setPoseTarget(target_pose1);
    success = group.plan(my_plan);

    ROS_INFO("Visualizing plan 3 (constraints) %s",success?"":"FAILED");
/* Sleep to give Rviz time to visualize the plan. */
    sleep(10.0);

    group.clearPathConstraints();

    //Cartesian paths
    std::vector<geometry_msgs::Pose> waypoints;

    geometry_msgs::Pose target_pose3 = start_pose2;
    target_pose3.position.x += 0.2;
    target_pose3.position.z += 0.2;
    waypoints.push_back(target_pose3);  // up and out

    target_pose3.position.y -= 0.2;
    waypoints.push_back(target_pose3);  // left

    target_pose3.position.z -= 0.2;
    target_pose3.position.y += 0.2;
    target_pose3.position.x -= 0.2;
    waypoints.push_back(target_pose3);  // down and right (back to start)

    moveit_msgs::RobotTrajectory trajectory;
    double fraction = group.computeCartesianPath(waypoints,
                                                 0.01,  // eef_step
                                                 0.0,   // jump_threshold
                                                 trajectory);

    ROS_INFO("Visualizing plan 4 (cartesian path) (%.2f%% acheived)",
             fraction * 100.0);
/* Sleep to give Rviz time to visualize the plan. */
    sleep(15.0);

    //Adding and Removing objects and Attaching/Detaching Objects

    moveit_msgs::CollisionObject collision_object;
    collision_object.header.frame_id = group.getPlanningFrame();

/* The id of the object is used to identify it. */
    collision_object.id = "box1";

/* Define a box to add to the world. */
    shape_msgs::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[0] = 0.4;
    primitive.dimensions[1] = 0.1;
    primitive.dimensions[2] = 0.4;

/* A pose for the box (specified relative to frame_id) */
    geometry_msgs::Pose box_pose;
    box_pose.orientation.w = 1.0;
    box_pose.position.x =  0.6;
    box_pose.position.y = -0.4;
    box_pose.position.z =  1.2;

    collision_object.primitives.push_back(primitive);
    collision_object.primitive_poses.push_back(box_pose);
    collision_object.operation = collision_object.ADD;

    std::vector<moveit_msgs::CollisionObject> collision_objects;
    collision_objects.push_back(collision_object);

    ROS_INFO("Add an object into the world");
    planning_scene_interface.addCollisionObjects(collision_objects);

/* Sleep so we have time to see the object in RViz */
    sleep(2.0);

    group.setPlanningTime(10.0);

    //Now when we plan a trajectory it will avoid the obstacle
    group.setStartState(*group.getCurrentState());
    group.setPoseTarget(target_pose1);
    success = group.plan(my_plan);

    ROS_INFO("Visualizing plan 5 (pose goal move around box) %s",
             success?"":"FAILED");
/* Sleep to give Rviz time to visualize the plan. */
    sleep(10.0);

    ROS_INFO("Attach the object to the robot");
    group.attachObject(collision_object.id);
/* Sleep to give Rviz time to show the object attached (different color). */
    sleep(4.0);

    ROS_INFO("Detach the object from the robot");
    group.detachObject(collision_object.id);
/* Sleep to give Rviz time to show the object detached. */
    sleep(4.0);

    ROS_INFO("Remove the object from the world");
    std::vector<std::string> object_ids;
    object_ids.push_back(collision_object.id);
    planning_scene_interface.removeCollisionObjects(object_ids);
/* Sleep to give Rviz time to show the object is no longer there. */
    sleep(4.0);

    //Dual-arm pose goals

    moveit::planning_interface::MoveGroup two_arms_group("arms");

    two_arms_group.setPoseTarget(target_pose1, "r_wrist_roll_link");

    geometry_msgs::Pose target_pose2;
    target_pose2.orientation.w = 1.0;
    target_pose2.position.x = 0.7;
    target_pose2.position.y = 0.15;
    target_pose2.position.z = 1.0;

    two_arms_group.setPoseTarget(target_pose2, "l_wrist_roll_link");

    moveit::planning_interface::MoveGroup::Plan two_arms_plan;
    two_arms_group.plan(two_arms_plan);
    sleep(4.0);

    ros::shutdown();
    return 0;
}