#include <ros/ros.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>

int main(int argc, char** argv){

  // Start the ros node
  ros::init(argc, argv, "Draw_X");
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  /* Get the parameter values from the ros_param server */
  std::string base_name;
  bool debug;
  nh.param<std::string>(ros::this_node::getName()+"/arm_base", base_name, "base");
  nh.param<bool>(ros::this_node::getName()+"/debug", debug, false);
  std::vector<Eigen::Vector3d> x_corners;
  x_corners.resize(4);
  for (size_t i=0; i < x_corners.size(); ++i)
  {
    double x, y, z;
    nh.param<double>(ros::this_node::getName()+"/corner"+std::to_string(i+1)+"_x", x, 0.65);
    nh.param<double>(ros::this_node::getName()+"/corner"+std::to_string(i+1)+"_y", y, 0.0);
    nh.param<double>(ros::this_node::getName()+"/corner"+std::to_string(i+1)+"_z", z, 0.05);
    x_corners[i] = Eigen::Vector3d(x, y, z);
    ROS_INFO_STREAM("The position of the " << i+1 << "th corner of X is: \n" << Eigen::Vector3d(x, y, z));
  }

  /* Initial move group c++ interface */
  static const std::string PLANNING_GROUP = "ur5_arm";
  moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  move_group.setPoseReferenceFrame("base");

  /* Initial MoveIt! visualization tool */
  namespace rvt = rviz_visual_tools;
  moveit_visual_tools::MoveItVisualTools visual_tools("base");
  visual_tools.deleteAllMarkers();
  visual_tools.loadRemoteControl(); // load remote control
  Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
  text_pose.translation().z() = 0.75;
  visual_tools.publishText(text_pose, "Draw X Demo Started (Press \"next\" to continue)", rvt::WHITE, rvt::XLARGE);
  visual_tools.trigger();
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to start the demo");

  /* Initial MoveIt! visualization tool */
  geometry_msgs::PoseStamped pose_base = move_group.getCurrentPose("base");
  geometry_msgs::PoseStamped pose_eef = move_group.getCurrentPose("tool0");

  /* Plan to the pose above the first corner of X */
  ROS_INFO("Plan to the pose above the first corner of X");
  geometry_msgs::Pose pose_target;
  pose_target.position.x = x_corners[0][0];
  pose_target.position.y = x_corners[0][1];
  pose_target.position.z = x_corners[0][2] + 0.05;
  pose_target.orientation.x = pose_eef.pose.orientation.x;
  pose_target.orientation.y = pose_eef.pose.orientation.y;
  pose_target.orientation.z = pose_eef.pose.orientation.z;
  pose_target.orientation.w = pose_eef.pose.orientation.w;
  move_group.setPoseTarget(pose_target, "tool0");

  if (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS)
  {
    if (move_group.execute(my_plan) != moveit::planning_interface::MoveItErrorCode::SUCCESS)
    {
      ROS_ERROR_STREAM("Execution fail.");
      ros::waitForShutdown();
      return 0;
    }
  }
  else
  {
    ROS_ERROR_STREAM("Plan fail.");
    ros::waitForShutdown();
    return 0;
  }

  /* Plan to the first corner of X */
  ROS_INFO("Plan to the first corner of X.");
  pose_target.position.z -= 0.05;
  move_group.setPoseTarget(pose_target, "tool0");
  if (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS)
  {
    if (move_group.execute(my_plan) != moveit::planning_interface::MoveItErrorCode::SUCCESS)
    {
      ROS_ERROR_STREAM("Execution fail.");
      ros::waitForShutdown();
      return 0;
    }
  }
  else
  {
    ROS_ERROR_STREAM("Plan fail.");
    ros::waitForShutdown();
    return 0;
  }

  /* Plan to the second corner of X */
  std::vector<geometry_msgs::Pose> waypoints;
  waypoints.push_back(pose_target); // add the current pose as the start of cartesian path
  geometry_msgs::Pose cartesian_pose_target; 
  cartesian_pose_target.position.x = x_corners[1][0];
  cartesian_pose_target.position.y = x_corners[1][1];
  cartesian_pose_target.position.z = x_corners[1][2];
  cartesian_pose_target.orientation.x = pose_eef.pose.orientation.x;
  cartesian_pose_target.orientation.y = pose_eef.pose.orientation.y;
  cartesian_pose_target.orientation.z = pose_eef.pose.orientation.z;
  cartesian_pose_target.orientation.w = pose_eef.pose.orientation.w;
  waypoints.push_back(cartesian_pose_target); // add the target pose as the end of cartesian path

  moveit_msgs::RobotTrajectory trajectory;
  const double jump_threshold = 0.0;
  const double eef_step = 0.002;
  double fraction = move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
  ROS_INFO_NAMED("Cartesian path", "(%.2f%% acheived)", fraction * 100.0);
  if (fraction == 1.0)
  {
    visual_tools.publishPath(waypoints, rvt::LIME_GREEN, rvt::SMALL);
    visual_tools.trigger();
    my_plan.trajectory_ = trajectory;
    if (move_group.execute(my_plan) != moveit::planning_interface::MoveItErrorCode::SUCCESS)
    {
      ROS_ERROR_STREAM("Execution fail.");
      ros::waitForShutdown();
      return 0;
    }    
  }
  else
  {
    ROS_ERROR_STREAM("Plan fail.");
    ros::waitForShutdown();
    return 0;
  }
    
  /* Plan to the pose above the second corner of X */
  ROS_INFO("Plan to the pose above the second corner of X.");
  pose_target.position.x = x_corners[1][0];
  pose_target.position.y = x_corners[1][1];
  pose_target.position.z = x_corners[1][2] + 0.05;
  move_group.setPoseTarget(pose_target, "tool0");
  if (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS)
  {
    if (move_group.execute(my_plan) != moveit::planning_interface::MoveItErrorCode::SUCCESS)
    {
      ROS_ERROR_STREAM("Execution fail.");
      ros::waitForShutdown();
      return 0;
    }
  }
  else
  {
    ROS_ERROR_STREAM("Plan fail.");
    ros::waitForShutdown();
    return 0;
  }

  /* Plan to the pose above the third corner of X */
  ROS_INFO("Plan to the pose above the third corner of X");
  pose_target.position.x = x_corners[2][0];
  pose_target.position.y = x_corners[2][1];
  pose_target.position.z = x_corners[2][2] + 0.05;
  move_group.setPoseTarget(pose_target, "tool0");
  if (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS)
  {
    if (move_group.execute(my_plan) != moveit::planning_interface::MoveItErrorCode::SUCCESS)
    {
      ROS_ERROR_STREAM("Execution fail.");
      ros::waitForShutdown();
      return 0;
    }
  }
  else
  {
    ROS_ERROR_STREAM("Plan fail.");
    ros::waitForShutdown();
    return 0;
  }

  /* Plan to the third corner of X */
  ROS_INFO("Plan to the third corner of X.");
  pose_target.position.z -=  0.05;
  move_group.setPoseTarget(pose_target, "tool0");
  if (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS)
  {
    if (move_group.execute(my_plan) != moveit::planning_interface::MoveItErrorCode::SUCCESS)
    {
      ROS_ERROR_STREAM("Execution fail.");
      ros::waitForShutdown();
      return 0;
    }
  }
  else
  {
    ROS_ERROR_STREAM("Plan fail.");
    ros::waitForShutdown();
    return 0;
  }

  /* Plan to the fourth corner of X */
  ROS_INFO("Plan to the fourth corner of X.");
  waypoints.clear();
  waypoints.push_back(pose_target); // add the current pose as the start of cartesian path
  cartesian_pose_target.position.x = x_corners[3][0];
  cartesian_pose_target.position.y = x_corners[3][1];
  cartesian_pose_target.position.z = x_corners[3][2];
  waypoints.push_back(cartesian_pose_target); // add the target pose as the end of cartesian path

  fraction = move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
  ROS_INFO_NAMED("Cartesian path", "(%.2f%% acheived)", fraction * 100.0);
  if (fraction == 1.0)
  {
    visual_tools.publishPath(waypoints, rvt::LIME_GREEN, rvt::SMALL);
    visual_tools.trigger();
    my_plan.trajectory_ = trajectory;
    if (move_group.execute(my_plan) != moveit::planning_interface::MoveItErrorCode::SUCCESS)
    {
      ROS_ERROR_STREAM("Execution fail.");
      ros::waitForShutdown();
      return 0;
    }    
  }
  else
  {
    ROS_ERROR_STREAM("Plan fail.");
    ros::waitForShutdown();
    return 0;
  }

  /* Plan to the pose above the fourth corner of X */
  ROS_INFO("Plan to the pose above the fourth corner of X.");
  pose_target.position.x = x_corners[3][0];
  pose_target.position.y = x_corners[3][1];
  pose_target.position.z = x_corners[3][2] + 0.05;
  move_group.setPoseTarget(pose_target, "tool0");
  if (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS)
  {
    if (move_group.execute(my_plan) != moveit::planning_interface::MoveItErrorCode::SUCCESS)
    {
      ROS_ERROR_STREAM("Execution fail.");
      ros::waitForShutdown();
      return 0;
    }
  }
  else
  {
    ROS_ERROR_STREAM("Plan fail.");
    ros::waitForShutdown();
    return 0;
  }

  /* Plan to the home pose */
  ROS_INFO("Plan to the home pose.");
  move_group.setNamedTarget("home");
  if (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS)
  {
    if (move_group.execute(my_plan) != moveit::planning_interface::MoveItErrorCode::SUCCESS)
    {
      ROS_ERROR_STREAM("Execution fail.");
      ros::waitForShutdown();
      return 0;
    }
  }
  else
  {
    ROS_ERROR_STREAM("Plan fail.");
    ros::waitForShutdown();
    return 0;
  }

  ros::waitForShutdown();
  ROS_INFO("Node pick up demo exited.");
  return 0;
}
