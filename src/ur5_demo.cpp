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

#include <Eigen/Geometry>
#include <eigen_conversions/eigen_msg.h>
#include <tf/transform_datatypes.h>

namespace moveit_simple
{

class UR5Demo : public OnlineRobot
{
public:
  UR5Demo()
  : OnlineRobot(ros::NodeHandle(), "robot_description", "arm", "base_link", "tool0")
  {
    refreshRobot();

    planning_scene_monitor_.reset(
       new planning_scene_monitor::PlanningSceneMonitor(planning_scene_, robot_model_loader_));

    planning_scene_monitor_->setPlanningScenePublishingFrequency(100);
    planning_scene_monitor_->startPublishingPlanningScene(planning_scene_monitor::PlanningSceneMonitor::UPDATE_SCENE);
    planning_scene_monitor_->startStateMonitor();
    planning_scene_monitor_->startSceneMonitor("/planning_scene");

    // Spin while we wait for the full robot state to become available
    std::vector<std::string> missing_joints;
    while (!planning_scene_monitor_->getStateMonitor()->haveCompleteState(missing_joints) && ros::ok())
    {
      std::stringstream ss;
      ss << "Waiting for complete state from topic. Missing[";
      for (auto &joint : missing_joints)
        ss << joint << ", ";
      ss << "]" << std::endl;
      ROS_ERROR_STREAM_THROTTLE_NAMED(1, "UR5Demo", ss.str());
    }

    trajectory_execution_manager_.reset(new trajectory_execution_manager::TrajectoryExecutionManager(
        planning_scene_monitor_->getRobotModel(), planning_scene_monitor_->getStateMonitor()));

    // online_visual_tools_->setPlanningSceneMonitor(planning_scene_monitor_);
    online_visual_tools_->setPlanningSceneTopic("/planning_scene");
    online_visual_tools_->loadMarkerPub();
    online_visual_tools_->loadTrajectoryPub("/display_planned_path");
    online_visual_tools_->deleteAllMarkers();  // clear all old markers
    online_visual_tools_->enableBatchPublishing();
    online_visual_tools_->trigger();
  }

  // bool planTrajectory()
  // {
  //   trajectory_name_ = "traj1";
  //   const Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
  //   const moveit_simple::InterpolationType cart = moveit_simple::interpolation_type::CARTESIAN;
  //   const moveit_simple::InterpolationType joint = moveit_simple::interpolation_type::JOINT;

  //   addTrajPoint(trajectory_name_, "home",      0.0, joint);
  //   addTrajPoint(trajectory_name_, "tf_pub1",   0.0, joint, 8);
  //   addTrajPoint(trajectory_name_, "tf_pub2",   12.0, joint, 8);
  //   ROS_INFO_NAMED("UR5Demo", "plan finished");
  //   return true;
  // }

  bool executeTrajectory(const std::string &trajectory_name, bool check_for_collisions)
  {
    ROS_ERROR_STREAM_NAMED("UR5Demo", "planning trajectory: " << trajectory_name);

    std::vector<moveit_simple::JointTrajectoryPoint> traj = plan(trajectory_name, check_for_collisions);
    ROS_ERROR_STREAM_NAMED("UR5Demo", "plan finished, len " << traj.size());

    moveit::core::RobotStatePtr virtual_robot_state(new moveit::core::RobotState(
          planning_scene_monitor::LockedPlanningSceneRO(planning_scene_monitor_)->getCurrentState()));
    virtual_robot_state->update();

    ROS_ERROR_STREAM_NAMED("UR5Demo", "got the current robot state");
    // virtual_robot_state->printStatePositionsWithJointLimits(joint_group_);
    robot_trajectory::RobotTrajectoryPtr trajectory(
        new robot_trajectory::RobotTrajectory(robot_model_ptr_, joint_group_->getName()));
    ROS_ERROR_STREAM_NAMED("UR5Demo", "trajectory initialized ");

    double dummy_dt = 0.01;  // this is overwritten and unimportant
    trajectory->addPrefixWayPoint(virtual_robot_state, dummy_dt);

    std::size_t count=0;
    for (auto &point: traj)
    {
      count++;
      std::vector<double> target_point = point.jointPoint();
      ROS_ERROR_STREAM_NAMED("UR5Demo",
                             "point: " << count << "\t"
                             << target_point[0]
                             << target_point[1]
                             << target_point[2]
                             << target_point[3]
                             << target_point[4]
                             << target_point[5]);

      virtual_robot_state->setJointGroupPositions(joint_group_, target_point);
      virtual_robot_state->update();
      trajectory->addSuffixWayPoint(virtual_robot_state, dummy_dt);
    }

    iterative_smoother_.computeTimeStamps(*trajectory, 1.0);

    moveit_msgs::RobotTrajectory trajectory_msg;
    trajectory->getRobotTrajectoryMsg(trajectory_msg);
    ROS_DEBUG_STREAM_NAMED("UR5Demo", "msg \n" << trajectory_msg);
    ROS_ERROR_STREAM_NAMED("UR5Demo", "publishing trajectory to mvt");

    visual_tools_->publishTrajectoryPath(trajectory_msg, virtual_robot_state, true);
    visual_tools_->trigger();
    visual_tools_->prompt("next");
    ROS_ERROR_STREAM_NAMED("UR5Demo", "calling execute");

    trajectory_execution_manager_->clear();
    ROS_ERROR_STREAM_NAMED("UR5Demo", "cleared");
    trajectory_execution_manager_->push(trajectory_msg);
    ROS_ERROR_STREAM_NAMED("UR5Demo", "pushed traj \n" << trajectory_msg);
    trajectory_execution_manager_->execute();
    ROS_ERROR_NAMED("UR5Demo", "execute");
    return trajectory_execution_manager_->waitForExecution();

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

  bool runPickAndPlace()
  {
    geometry_msgs::Pose object_pose;
    object_pose.orientation.w = 1.0;
    object_pose.position.y = 0.5;
    object_pose.position.x = 0.2;
    object_pose.position.z = 0.075;
    spawnObject(object_pose);
    online_visual_tools_->prompt("continue");

    // create pick pose
    Eigen::Isometry3d pick_pose;
    object_pose.position.z = 0.3;
    object_pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, M_PI, 0);
    tf::poseMsgToEigen(object_pose, pick_pose);

    // create place pose
    Eigen::Isometry3d place_pose = pick_pose * Eigen::Translation3d(0.0, -1.0, 0);

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
        pick_down_pose *= Eigen::Translation3d(0, 0, 0.05);
        getPose(place_state, place_down_pose);
        place_down_pose *= Eigen::Translation3d(0, 0, 0.05);

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

        // lift, move, lower
        traj_name = "pick_move";
        clearTrajectory(traj_name);
        addTrajPoint(traj_name, pick_point2, cart, 10);
        addTrajPoint(traj_name, place_point, cart, 50);
        addTrajPoint(traj_name, place_down_point, cart, 10);
        ROS_ERROR_STREAM("place");
        online_visual_tools_->prompt("pick");
        execute(traj_name, check_collisions, true);

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

  // Trajectory execution
  trajectory_execution_manager::TrajectoryExecutionManagerPtr trajectory_execution_manager_;
  trajectory_processing::IterativeSplineParameterization iterative_smoother_;

  // PSM

  // For visualizing things in rviz
  moveit_visual_tools::MoveItVisualToolsPtr visual_tools_;

  planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor_;


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
  std::map<size_t, double> seed_fractions;
  seed_fractions[0] = 0.1; // limit base windup
  seed_fractions[1] = 1; // limit shoulder windup
  seed_fractions[2] = 1; // limit elbow windup
  seed_fractions[3] = 0.7; // limit wrist 1 joint windup
  seed_fractions[4] = 0.7; // limit wrist 2 joint windup
  seed_fractions[5] = 0.7; // limit wrist 3 joint windup
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
