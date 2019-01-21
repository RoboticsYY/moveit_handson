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
#include <math.h>

// MoveIt!
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <moveit/collision_detection/collision_world.h>
#include <moveit/collision_detection/collision_tools.h>

// TF2
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_eigen/tf2_eigen.h>

//For add object
#include <geometric_shapes/shapes.h>
#include <geometric_shapes/mesh_operations.h>
#include <geometric_shapes/shape_operations.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <geometric_shapes/shape_operations.h>

#define IGNORE

void addContainer(moveit::planning_interface::PlanningSceneInterface& planning_scene_interface)
{
  // Vector to scale
  Eigen::Vector3d vectorScale(0.001, 0.001, 0.001);
  // Define a collision object ROS message.
  moveit_msgs::CollisionObject object;
  // The id of the object is used to identify it.
  object.id = "container";
  object.header.frame_id = "operation_surface";

  // Define a color applied to the collision object.
  moveit_msgs::ObjectColor color;
  color.id = "container";
  color.color.r = 1.0;
  color.color.g = 0.5;
  color.color.b = 0.5;
  color.color.a = 0.4;

  // Path where is located le model 
  shapes::Mesh* m = shapes::createMeshFromResource("package://handson_description/meshes/container.stl", vectorScale); 
  // Define and load the mesh
  shape_msgs::Mesh mesh;
  shapes::ShapeMsg mesh_msg;  
  shapes::constructMsgFromShape(m, mesh_msg);
  mesh = boost::get<shape_msgs::Mesh>(mesh_msg);
  ROS_INFO("Object mesh to manipulate...loaded");

  object.meshes.resize(1);
  object.mesh_poses.resize(1);

  tf2::Quaternion q;
  q.setRPY(0, 0, M_PI/2);

  // Define a pose for the object (specified relative to frame_id)
  geometry_msgs::Pose mesh_pose;
  mesh_pose.position.x = 0.175;
  mesh_pose.position.y = 0.0;
  mesh_pose.position.z = 0.005;
  mesh_pose.orientation.w= q.getW(); 
  mesh_pose.orientation.x= q.getX(); 
  mesh_pose.orientation.y= q.getY();
  mesh_pose.orientation.z= q.getZ();
    
  // The object is added like un colission object
  object.meshes.push_back(mesh);
  object.mesh_poses.push_back(mesh_pose);
  object.operation = object.ADD;      // operations are be object.REMOVE, object.APPEND, object.MOVE 

  // Create vector of collision object messages for the planning_scene_interface
  std::vector<moveit_msgs::CollisionObject> objects;
  objects.push_back(object);
  std::vector<moveit_msgs::ObjectColor> object_colors;
  object_colors.push_back(color);
    
  // Add the collision objects into the world
  planning_scene_interface.addCollisionObjects(objects, object_colors);
  ros::Duration(1.5).sleep();
}

void createBox(moveit::planning_interface::PlanningSceneInterface& planning_scene_interface, 
               std::string id = "object",
               std::string frame_id = "operation_surface",
               Eigen::Vector4d color = Eigen::Vector4d(0, 0, 0.6, 1.0),
               Eigen::Vector3d rotation = Eigen::Vector3d(0, 0, 0), 
               Eigen::Vector3d translation = Eigen::Vector3d(0, 0, 0))
{
  // Create vector to hold the collision objects.
  std::vector<moveit_msgs::CollisionObject> collision_objects;
  collision_objects.resize(1);

  // Define the object that we will be manipulating
  collision_objects[0].id = id;
  collision_objects[0].header.frame_id = frame_id;

  // Define a color applied to the collision object.
  std::vector<moveit_msgs::ObjectColor> object_colors;
  object_colors.resize(1);
  object_colors[0].id = id;
  object_colors[0].color.r = color[0];
  object_colors[0].color.g = color[1];
  object_colors[0].color.b = color[2];
  object_colors[0].color.a = color[3];

  /* Define the primitive and its dimensions. */
  collision_objects[0].primitives.resize(1);
  collision_objects[0].primitives[0].type = collision_objects[1].primitives[0].BOX;
  collision_objects[0].primitives[0].dimensions.resize(3);
  collision_objects[0].primitives[0].dimensions[0] = 0.07;
  collision_objects[0].primitives[0].dimensions[1] = 0.075;
  collision_objects[0].primitives[0].dimensions[2] = 0.005;

  tf2::Quaternion q;
  q.setRPY(rotation[0], rotation[1], rotation[2]);

  /* Define the pose of the object. */
  collision_objects[0].primitive_poses.resize(1);
  collision_objects[0].primitive_poses[0].position.x = translation[0];
  collision_objects[0].primitive_poses[0].position.y = translation[1];
  collision_objects[0].primitive_poses[0].position.z = translation[2];
  collision_objects[0].primitive_poses[0].orientation.w = q.getW();
  collision_objects[0].primitive_poses[0].orientation.x = q.getX();
  collision_objects[0].primitive_poses[0].orientation.y = q.getY();
  collision_objects[0].primitive_poses[0].orientation.z = q.getZ();

  collision_objects[0].operation = collision_objects[0].ADD;

  planning_scene_interface.addCollisionObjects(collision_objects, object_colors);
}

void removeBox(moveit::planning_interface::PlanningSceneInterface& planning_scene_interface, std::string id)
{
  std::vector<std::string> object_ids;
  object_ids.push_back(id);
  planning_scene_interface.removeCollisionObjects(object_ids);
}

void publishContactMarkerArray(ros::Publisher& pub, visualization_msgs::MarkerArray& pre_collision_points, visualization_msgs::MarkerArray& cur_collision_points)
{
  // delete old markers
  if (pre_collision_points.markers.size())
  {
    for (int i = 0; i < pre_collision_points.markers.size(); i++)
      pre_collision_points.markers[i].action = visualization_msgs::Marker::DELETE;

    pub.publish(pre_collision_points);
  }

  // move new markers into g_collision_points
  std::swap(pre_collision_points.markers, cur_collision_points.markers);

  // draw new markers (if there are any)
  if (pre_collision_points.markers.size())
    pub.publish(pre_collision_points);
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

  // Add rviz_visual_tools
  namespace rvt = rviz_visual_tools;
  moveit_visual_tools::MoveItVisualTools visual_tools("base");
  visual_tools.deleteAllMarkers();
  visual_tools.loadRemoteControl(); // load remote control

  // Create a marker array publisher for publishing contact points
  ros::Publisher g_marker_array_publisher = nh.advertise<visualization_msgs::MarkerArray>("/move_group/display_contacts", 100);

  visualization_msgs::MarkerArray g_collision_points;

  // Add box container
  addContainer(planning_scene_interface);

  // Add two destinations
  createBox(planning_scene_interface, "destination0", "operation_surface", 
                     Eigen::Vector4d(0, 0, 0.6, 1.0), 
                     Eigen::Vector3d(0, 0, 0),
                     Eigen::Vector3d(-0.15, 0.065, 0));
  createBox(planning_scene_interface, "destination1", "operation_surface",
                     Eigen::Vector4d(0, 0, 0.6, 1.0), 
                     Eigen::Vector3d(0, 0, 0),
                     Eigen::Vector3d(-0.15, -0.065, 0));

  // Get the pose of "operation_surface"
  robot_model::RobotModelConstPtr model = group.getRobotModel();
  robot_state::RobotStatePtr state(new robot_state::RobotState(model));
  geometry_msgs::Pose pose_surface = tf2::toMsg(state->getGlobalLinkTransform("operation_surface"));
  geometry_msgs::Pose pose_world = tf2::toMsg(state->getGlobalLinkTransform("world"));
  ROS_INFO_STREAM("Position of operation_surface(xyz): " << pose_surface.position.x << " "
                                                         << pose_surface.position.y << " "
                                                         << pose_surface.position.z << ", "
                                                         << "Position of world(xyz): " 
                                                         << pose_world.position.x << " "
                                                         << pose_world.position.y << " "
                                                         << pose_world.position.z << ", "
                                                         << "reference frame: " << model->getModelFrame());

  // Create planning_scene for collision-detection
  planning_scene::PlanningScene planning_scene(model);
  collision_detection::CollisionRequest collision_request;
  collision_request.contacts = true;
  collision_request.max_contacts = 100;
  collision_request.max_contacts_per_pair = 10;
  collision_request.verbose = false;
  collision_detection::CollisionResult collision_result;

  // Add the container to the planning_scene for collision checking
  Eigen::Vector3d vectorScale(0.001, 0.001, 0.001);
  shapes::Mesh* m = shapes::createMeshFromResource("package://handson_description/meshes/container.stl", vectorScale);
  shapes::ShapePtr container(m);
  Eigen::Affine3d container_pose; 
  tf2::fromMsg(pose_surface, container_pose);
  container_pose.translate(Eigen::Vector3d(0.175, 0, 0.005));
  container_pose.rotate(Eigen::AngleAxisd(M_PI/2, Eigen::Vector3d(0,0,1)));
  planning_scene.getWorldNonConst()->addToObject("container", container, container_pose);

  // Add random box
  std::vector<std::string> boxes;
  int nbox_per_layer = 2;
  int n_layer = 20;
  random_numbers::RandomNumberGenerator rng;
  double x = pose_surface.position.x + 0.175;
  double y = pose_surface.position.y;
  double h = pose_surface.position.z + 0.021;
  int index = 0;
  for(size_t i = 0; i < n_layer; i++)
  {
    int nbox_current_layer = 0;
    while( nbox_current_layer < nbox_per_layer)
    {
      // Generate random pose for the box
      double theta = rng.uniformReal(0, M_PI);
      double box_x = x + rng.uniformReal(-0.5, 0.5)*0.2;
      double box_y = y + rng.uniformReal(-0.5, 0.5)*0.4;
      std::string box_id = "box" + std::to_string(index);

      // Add the box through "planning_scene_interface"
      createBox(planning_scene_interface, box_id, model->getModelFrame(), 
                         Eigen::Vector4d(0, 0.6, 0, 1.0),
                         Eigen::Vector3d(0, 0, theta),
                         Eigen::Vector3d(box_x, box_y, h));

      const collision_detection::CollisionWorldConstPtr copy_world = planning_scene.getCollisionWorld();

      // Add the box in "planning_scene"
      shapes::ShapePtr box(new shapes::Box(0.07, 0.075, 0.005));
      Eigen::Affine3d box_pose; 
      tf2::fromMsg(pose_world, box_pose);
      box_pose.translate(Eigen::Vector3d(box_x, box_y, h));
      box_pose.rotate(Eigen::AngleAxisd(theta, Eigen::Vector3d(0,0,1)));
      planning_scene.getWorldNonConst()->addToObject(box_id, box, box_pose);

      // Test if the added box is in collision with the world
      collision_result.clear();
      // planning_scene.checkCollision(collision_request, collision_result);
      copy_world->checkWorldCollision(collision_request, collision_result, *planning_scene.getCollisionWorld());
      ROS_INFO_STREAM("Test: Current state is " << (collision_result.collision ? "in" : "not in") << " collision");

      // If the box is in collision, remove the box both in "planning_scene_interface" and "planning_scene"
      if (collision_result.collision)
      {
        if (collision_result.contact_count > 0)
        {
          ROS_INFO_STREAM("COLLIDING contact_point_count=" << (int)collision_result.contact_count);
          std_msgs::ColorRGBA color;
          color.r = 1.0;
          color.g = 0.0;
          color.b = 1.0;
          color.a = 0.5;
          visualization_msgs::MarkerArray markers;

          /* Get the contact ponts and display them as markers */
          collision_detection::getCollisionMarkersFromContacts(markers, model->getModelFrame(), collision_result.contacts, color,
                                                              ros::Duration(),  // remain until deleted
                                                              0.003);            // radius
          publishContactMarkerArray(g_marker_array_publisher, g_collision_points, markers);
        removeBox(planning_scene_interface, box_id);
        planning_scene.getWorldNonConst()->removeShapeFromObject(box_id, box);
        }
      }
      else
      {
        nbox_current_layer++;
        index++;
        // delete the old collision point markers
        visualization_msgs::MarkerArray empty_marker_array;
        publishContactMarkerArray(g_marker_array_publisher, g_collision_points, empty_marker_array);
      }
    }
    h += 0.011;
  }

  ROS_INFO_STREAM("Boxes generation finished.");

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