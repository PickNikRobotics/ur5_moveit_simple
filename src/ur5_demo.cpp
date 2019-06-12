/*
 * Software License Agreement (Apache License)
 *
 * Copyright (c) 2019 Plus One Robotics
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <ros/ros.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/trajectory_execution_manager/trajectory_execution_manager.h>
#include <moveit/trajectory_processing/iterative_spline_parameterization.h>

// Robotiq
#include <robotiq_2f_gripper_control/Robotiq2FGripper_robot_output.h>

// Visual tools
#include <moveit_visual_tools/moveit_visual_tools.h>

#include <moveit_simple/moveit_simple.h>
#include <moveit_simple/prettyprint.hpp>

#include <Eigen/Geometry>
#include <eigen_conversions/eigen_msg.h>
#include <tf/transform_datatypes.h>

namespace moveit_simple
{

class UR5Demo : public OnlineRobot
{
public:
  UR5Demo(ros::NodeHandle nh)
  : OnlineRobot(nh, "robot_description", "arm", "base_link", "tool0")
  , nh_(nh)
  {
    refreshRobot();

    robotiq_publisher_ = nh_.advertise<robotiq_2f_gripper_control::Robotiq2FGripper_robot_output>("Robotiq2FGripperRobotOutput", 10);
    if (false)
      loadPlanningSceneMonitor();

    online_visual_tools_->setPlanningSceneTopic("/planning_scene");
    online_visual_tools_->loadMarkerPub();
    online_visual_tools_->loadTrajectoryPub("/display_planned_path");
    online_visual_tools_->deleteAllMarkers();  // clear all old markers
    online_visual_tools_->enableBatchPublishing();
    online_visual_tools_->trigger();
  }

  bool loadPlanningSceneMonitor()
  {
    planning_scene_monitor_.reset(
        new planning_scene_monitor::PlanningSceneMonitor(planning_scene_, robot_model_loader_));

    if (planning_scene_monitor_->getPlanningScene())
    {
      planning_scene_monitor_->startSceneMonitor();
      planning_scene_monitor_->startWorldGeometryMonitor();
      planning_scene_monitor_->startStateMonitor();
      planning_scene_monitor_->setPlanningScenePublishingFrequency(100);
      planning_scene_monitor_->startPublishingPlanningScene(planning_scene_monitor::PlanningSceneMonitor::UPDATE_SCENE, "/planning_scene");
      planning_scene_monitor_->startSceneMonitor("/planning_scene");
    }

    // Spin while we wait for the full robot state to become available
    while (!planning_scene_monitor_->getStateMonitor()->haveCompleteState() && ros::ok())
    {
     ROS_INFO_STREAM_THROTTLE_NAMED(1, "UR5Demo", "Waiting for complete state from topic ");
    }

    online_visual_tools_->setPlanningSceneMonitor(planning_scene_monitor_);
  }

  bool spawnObject(const geometry_msgs::Pose& pose)
  {
    online_visual_tools_->publishCollisionCuboid(pose, 0.04, 0.04, 0.15, "object", rviz_visual_tools::colors::GREEN);
    online_visual_tools_->triggerPlanningSceneUpdate();
    return true;
  }
  void setIKSeedMidPoint(const std::string &pose_id)
  {
    robot_state::RobotStatePtr virtual_state = online_visual_tools_->getSharedRobotState();
    virtual_state->setToDefaultValues(joint_group_, pose_id);
    virtual_state->update();
    std::vector<double> mid_point;
    virtual_state->copyJointGroupPositions(joint_group_, mid_point);
    setIKSeedStateMidPoint(mid_point);
  }
  bool openGripper()
  {
    robotiq_2f_gripper_control::Robotiq2FGripper_robot_output open_msg;
    open_msg.rACT = 1;
    open_msg.rGTO = 1;
    open_msg.rATR = 0;
    open_msg.rPR = 0;
    open_msg.rSP = 255;
    open_msg.rFR = 175;
    robotiq_publisher_.publish(open_msg);
    ros::Duration(0.5).sleep();
    return true;
  }
  bool closeGripper()
  {
    robotiq_2f_gripper_control::Robotiq2FGripper_robot_output close_msg;
    close_msg.rACT = 1;
    close_msg.rGTO = 1;
    close_msg.rATR = 0;
    close_msg.rPR = 255;
    close_msg.rSP = 255;
    close_msg.rFR = 175;
    robotiq_publisher_.publish(close_msg);
    ros::Duration(0.5).sleep();
    return true;
  }

  bool runPickAndPlace()
  {
    geometry_msgs::Pose object_pose;
    object_pose.orientation.w = 1.0;
    object_pose.position.y = 0.35;
    object_pose.position.x = 0.2;
    object_pose.position.z = 0.075;
    spawnObject(object_pose);
    online_visual_tools_->prompt("continue");

    // create pick pose
    double approach_distance = 0.075;
    Eigen::Isometry3d pick_pose;
    object_pose.position.z = 0.18 + approach_distance;
    object_pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, M_PI, 0);
    tf::poseMsgToEigen(object_pose, pick_pose);

    // create place pose
    Eigen::Isometry3d place_pose = pick_pose * Eigen::Translation3d(0.0, -0.70, 0);
    place_pose = place_pose * Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitZ());
    std::vector<double> seed, pick_state, place_state;

    // Use ready as seed
    robot_state::RobotStatePtr virtual_state = online_visual_tools_->getSharedRobotState();
    virtual_state->setToDefaultValues(joint_group_, "ready");
    virtual_state->update();
    virtual_state->copyJointGroupPositions(joint_group_, seed);

    try{
      if (getPickPlaceJointSolutions(pick_pose, place_pose, "tool0", 1.0, seed, pick_state, place_state))
      {
        // compute actual pick/place points
        Eigen::Isometry3d pick_down_pose, place_down_pose;
        getPose(pick_state, pick_down_pose);
        pick_down_pose *= Eigen::Translation3d(0, 0, approach_distance);
        getPose(place_state, place_down_pose);
        place_down_pose *= Eigen::Translation3d(0, 0, approach_distance);

        const moveit_simple::InterpolationType cart = moveit_simple::interpolation_type::CARTESIAN;
        const moveit_simple::InterpolationType joint = moveit_simple::interpolation_type::JOINT;
        std::unique_ptr<moveit_simple::TrajectoryPoint> pick_point = std::unique_ptr<moveit_simple::TrajectoryPoint>(
            new moveit_simple::JointTrajectoryPoint(pick_state, 0.0, "pick_point"));
        std::unique_ptr<moveit_simple::TrajectoryPoint> pick_point2 = std::unique_ptr<moveit_simple::TrajectoryPoint>(
            new moveit_simple::JointTrajectoryPoint(pick_state, 0.0, "pick_point"));
        std::unique_ptr<moveit_simple::TrajectoryPoint> pick_down_point = std::unique_ptr<moveit_simple::TrajectoryPoint>(
            new moveit_simple::CartTrajectoryPoint(pick_down_pose, 0.0, "pick_down_point"));

        std::unique_ptr<moveit_simple::TrajectoryPoint> place_point = std::unique_ptr<moveit_simple::TrajectoryPoint>(
            new moveit_simple::JointTrajectoryPoint(place_state, 0.0, "place_point"));
        std::unique_ptr<moveit_simple::TrajectoryPoint> place_point2 = std::unique_ptr<moveit_simple::TrajectoryPoint>(
            new moveit_simple::JointTrajectoryPoint(place_state, 0.0, "place_point"));
        std::unique_ptr<moveit_simple::TrajectoryPoint> place_down_point = std::unique_ptr<moveit_simple::TrajectoryPoint>(
            new moveit_simple::CartTrajectoryPoint(place_down_pose, 0.0, "place_down_point"));


        // TODO(henningkayser): move to pre-pose that doesn't collide when interpolating to pick_point
        openGripper();
        bool check_collisions = false;
        std::string traj_name = "pick_approach";
        clearTrajectory(traj_name);
        addTrajPoint(traj_name, "ready", 3, joint, 100);
        addTrajPoint(traj_name, pick_point, joint, 100);
        addTrajPoint(traj_name, pick_down_point, cart, 10);
        ROS_ERROR_STREAM("pick_approach");
        online_visual_tools_->prompt("pick approach");
        execute(traj_name, check_collisions, true);

        // pick object
        online_visual_tools_->attachCO("object", "tool0");
        online_visual_tools_->trigger();
        closeGripper();

        // lift, move, lower
        traj_name = "pick_move";
        clearTrajectory(traj_name);
        addTrajPoint(traj_name, pick_point2, cart, 10);
        addTrajPoint(traj_name, place_point, cart, 50);
        addTrajPoint(traj_name, place_down_point, cart, 10);
        ROS_ERROR_STREAM("place");
        online_visual_tools_->prompt("pick");
        execute(traj_name, check_collisions, true);
        openGripper();

        // drop object
        online_visual_tools_->cleanupACO("object");
        online_visual_tools_->trigger();

        // retreat
        traj_name = "place_retreat";
        clearTrajectory(traj_name);
        addTrajPoint(traj_name, place_point2, cart, 50);
        addTrajPoint(traj_name, "ready", 4, joint, 100);
        ROS_ERROR_STREAM("place retreat");
        online_visual_tools_->prompt("place retreat");
        execute(traj_name, check_collisions, true);

      }
    }
    catch (const std::exception& e)
    {
      ROS_ERROR_STREAM("Execution failed: " << e.what());
      return false;
    }
    return true;
  }

  std::string trajectory_name_;

  // For visualizing things in rviz
  moveit_visual_tools::MoveItVisualToolsPtr visual_tools_;
  planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor_;

  ros::NodeHandle nh_;
  ros::Publisher robotiq_publisher_;
};


}

int main(int argc, char **argv)
{
  std::string name = "moveit_simple";
  ros::init(argc, argv, name);

  ros::AsyncSpinner spinner(4);
  spinner.start();

  std::unique_ptr<moveit_simple::UR5Demo> demo(new moveit_simple::UR5Demo(ros::NodeHandle()));
  ROS_INFO("Initialized");

  // limit joint windup
  // TODO(henningkayser): add suitable waypoints to showcase this feature
  demo->setLimitJointWindup(true);
  std::map<size_t, double> seed_fractions;
  seed_fractions[0] = 0.5; // limit base windup
  seed_fractions[1] = 0.5; // limit shoulder windup
  seed_fractions[2] = 0.5; // limit elbow windup
  seed_fractions[3] = 0.5; // limit wrist 1 joint windup
  seed_fractions[4] = 0.5; // limit wrist 2 joint windup
  seed_fractions[5] = 0.5; // limit wrist 3 joint windup
  demo->setIKSeedStateFractions(seed_fractions);

  demo->setIKSeedMidPoint("ready");
  // define end effector symmetry type and axis
  demo->setEndEffectorSymmetry(moveit_simple::EndEffectorSymmetry::Rectangular, Eigen::Vector3d::UnitZ());

  // run simple pick and place
  while (ros::ok())
    demo->runPickAndPlace();

  // plan and execute waypoint trajectory
  // demo->planTrajectory();
  // demo->executeTrajectory();

  // ros::waitForShutdown();
  ros::shutdown();
  return 0;

}
