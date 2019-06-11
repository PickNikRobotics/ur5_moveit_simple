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

// Visual tools
#include <moveit_visual_tools/moveit_visual_tools.h>

#include <moveit_simple/moveit_simple.h>
#include <moveit_simple/prettyprint.hpp>

namespace moveit_simple
{

class UR5Demo : public OnlineRobot
{
public:
  UR5Demo()
  : OnlineRobot(ros::NodeHandle(), "robot_description", "arm", "base_link", "tool_custom")
  {
    refreshRobot();
    planning_scene_monitor_.reset(
       new planning_scene_monitor::PlanningSceneMonitor(planning_scene_, robot_model_loader_));

    planning_scene_monitor_->setPlanningScenePublishingFrequency(100);
    planning_scene_monitor_->startPublishingPlanningScene(planning_scene_monitor::PlanningSceneMonitor::UPDATE_SCENE);
    planning_scene_monitor_->startStateMonitor();
    planning_scene_monitor_->startSceneMonitor("/planning_scene");

    // Spin while we wait for the full robot state to become available
    while (!planning_scene_monitor_->getStateMonitor()->haveCompleteState() && ros::ok())
    {
     ROS_INFO_STREAM_THROTTLE_NAMED(1, "UR5Demo", "Waiting for complete state from topic ");
    }

    visual_tools_.reset(new moveit_visual_tools::MoveItVisualTools("/world", "/moveit_visual_marker", planning_scene_monitor_));
    visual_tools_->setPlanningSceneTopic("/planning_scene");
    visual_tools_->loadMarkerPub();
    visual_tools_->loadTrajectoryPub("/robot_traj_display");
    visual_tools_->deleteAllMarkers();  // clear all old markers
    visual_tools_->enableBatchPublishing();
    visual_tools_->hideRobot();         // show that things have been reset
    visual_tools_->trigger();

  }

  bool planTrajectory()
  {
    trajectory_name_ = "traj1";
    const Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
    const moveit_simple::InterpolationType cart = moveit_simple::interpolation_type::CARTESIAN;
    const moveit_simple::InterpolationType joint = moveit_simple::interpolation_type::JOINT;

    if (false)
    {
      moveit::core::RobotStatePtr virtual_state = visual_tools_->getSharedRobotState();
      visual_tools_->prompt("home");
      virtual_state->setToDefaultValues(joint_group_, "home");
      virtual_state->update();
      visual_tools_->publishRobotState(virtual_state);
      visual_tools_->trigger();
      visual_tools_->prompt("ready");
      virtual_state->setToDefaultValues(joint_group_, "ready");
      virtual_state->update();
      visual_tools_->publishRobotState(virtual_state);
      visual_tools_->trigger();
      visual_tools_->prompt("done");
      visual_tools_->hideRobot();
      visual_tools_->trigger();
    }

    addTrajPoint(trajectory_name_, "home",      0.0, joint);
    addTrajPoint(trajectory_name_, "ready",      0.0, joint);
    // addTrajPoint(trajectory_name_, "tf_pub1",   0.0, cart, 8);
    // addTrajPoint(trajectory_name_, "tf_pub2",   12.0, cart, 8);
    ROS_INFO_NAMED("UR5Demo", "plan finished");
    return true;
  }

  bool executeTrajectory()
  {
    try
    {
      execute(trajectory_name_, true, true);
      return true;
    }
    catch (const std::exception& e)
    {
      ROS_ERROR_STREAM("Execution failed: " << e.what());
      return false;
    }
  }

  std::string trajectory_name_;

  // PSM
  planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor_;
  // For visualizing things in rviz
  moveit_visual_tools::MoveItVisualToolsPtr visual_tools_;
};
}

int main(int argc, char **argv)
{
  std::string name = "moveit_simple";
  ros::init(argc, argv, name);

  ros::AsyncSpinner spinner(4);
  spinner.start();

  std::unique_ptr<moveit_simple::UR5Demo> demo(new moveit_simple::UR5Demo());
  ROS_INFO("Initialized");

  // limit joint windup
  // TODO(henningkayser): add suitable waypoints to showcase this feature
  demo->setLimitJointWindup(true);
  std::map<size_t, double> seed_fractions; // pull all joints towards 0
  seed_fractions[0] = 0.1;
  seed_fractions[1] = 0.1;
  seed_fractions[2] = 0.1;
  seed_fractions[3] = 0.1;
  seed_fractions[4] = 0.1;
  seed_fractions[5] = 0.1;
  demo->setIKSeedStateFractions(seed_fractions);
  //TODO(henningkayser): showcase symmetric IK (pick/place)

  // plan & execute trajectory
  demo->planTrajectory();
  demo->executeTrajectory();

  ros::waitForShutdown();
  return 0;

}
