// Copyright (c) 2018 Intel Corporation
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// ROS
#include <ros/ros.h>

// MoveIt!
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>

// TF2
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

//For add object
#include <geometric_shapes/shapes.h>
#include <geometric_shapes/mesh_operations.h>
#include <geometric_shapes/shape_operations.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <geometric_shapes/shape_operations.h>

void addCollisionObjects(moveit::planning_interface::PlanningSceneInterface& planning_scene_interface)
{
    //Vector to scale
  Eigen::Vector3d vectorScale(0.001, 0.001, 0.001);
  // Define a collision object ROS message.
  moveit_msgs::CollisionObject object;
  // The id of the object is used to identify it.
  object.id = "container";
  object.header.frame_id = "operation_surface";

  //Path where is located le model 
  shapes::Mesh* m = shapes::createMeshFromResource("package://handson_description/meshes/container.stl", vectorScale); 
  // Define and load the mesh
  shape_msgs::Mesh mesh;
  shapes::ShapeMsg mesh_msg;  
  shapes::constructMsgFromShape(m, mesh_msg);
  mesh = boost::get<shape_msgs::Mesh>(mesh_msg);
  ROS_INFO("Object mesh to manipulate...loaded");

  object.meshes.resize(1);
  object.mesh_poses.resize(1);

  //Define a pose for the object (specified relative to frame_id)
  geometry_msgs::Pose mesh_pose;
  mesh_pose.position.x = 0.0;
  mesh_pose.position.y = 0.0;
  mesh_pose.position.z = 0.0;
  mesh_pose.orientation.w= 1.0; 
  mesh_pose.orientation.x= 0.0; 
  mesh_pose.orientation.y= 0.0;
  mesh_pose.orientation.z= 0.0;
    
  //The object is added like un colission object
  object.meshes.push_back(mesh);
  object.mesh_poses.push_back(mesh_pose);
  object.operation = object.ADD;      // operations are be object.REMOVE, object.APPEND, object.MOVE 

  // Create vector of collision object messages for the planning_scene_interface
  std::vector<moveit_msgs::CollisionObject> objects;
  objects.push_back(object);
    
  // Add the collision objects into the world
  planning_scene_interface.addCollisionObjects(objects);
  ros::Duration(1.5).sleep();
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "ur5_arm_pick_place");
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  ros::WallDuration(1.0).sleep();
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  moveit::planning_interface::MoveGroupInterface group("ur5_arm");
  group.setPlanningTime(45.0);

  addCollisionObjects(planning_scene_interface);

  // Move to the home pose
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  group.setStartStateToCurrentState();
  group.setNamedTarget("home");
  if (group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS)
  {
    if (group.execute(my_plan) != moveit::planning_interface::MoveItErrorCode::SUCCESS)
      return 1;
  }

  ros::waitForShutdown();
  return 0;
}