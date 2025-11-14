#include <memory>
#include <thread>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "moveit/move_group_interface/move_group_interface.h"
#include "moveit/planning_scene_interface/planning_scene_interface.h"
#include "geometry_msgs/msg/pose.hpp"
#include "xarm_remote/action/execute_task.hpp"
#include "xarm_remote/action/pick_place.hpp"

using namespace std::chrono_literals;

class XArmTaskServer : public rclcpp::Node
{
public:
  using ExecuteTask = xarm_remote::action::ExecuteTask;
  using PickPlace = xarm_remote::action::PickPlace;
  using GoalHandleExecuteTask = rclcpp_action::ServerGoalHandle<ExecuteTask>;
  using GoalHandlePickPlace = rclcpp_action::ServerGoalHandle<PickPlace>;

  explicit XArmTaskServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("xarm_task_server", options)
  {
    RCLCPP_INFO(this->get_logger(), "Initializing XArm Task Server...");

    // Create action servers
    execute_task_server_ = rclcpp_action::create_server<ExecuteTask>(
      this,
      "execute_task",
      std::bind(&XArmTaskServer::handle_execute_goal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&XArmTaskServer::handle_execute_cancel, this, std::placeholders::_1),
      std::bind(&XArmTaskServer::handle_execute_accepted, this, std::placeholders::_1));

    pick_place_server_ = rclcpp_action::create_server<PickPlace>(
      this,
      "pick_place",
      std::bind(&XArmTaskServer::handle_pick_place_goal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&XArmTaskServer::handle_pick_place_cancel, this, std::placeholders::_1),
      std::bind(&XArmTaskServer::handle_pick_place_accepted, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "XArm Task Server ready!");
  }

private:
  rclcpp_action::Server<ExecuteTask>::SharedPtr execute_task_server_;
  rclcpp_action::Server<PickPlace>::SharedPtr pick_place_server_;

  // ExecuteTask action handlers
  rclcpp_action::GoalResponse handle_execute_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const ExecuteTask::Goal> goal)
  {
    (void)uuid;
    RCLCPP_INFO(this->get_logger(), "Received execute task request: %s", goal->task_name.c_str());
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_execute_cancel(
    const std::shared_ptr<GoalHandleExecuteTask> goal_handle)
  {
    (void)goal_handle;
    RCLCPP_INFO(this->get_logger(), "Received request to cancel execute task");
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_execute_accepted(const std::shared_ptr<GoalHandleExecuteTask> goal_handle)
  {
    std::thread{std::bind(&XArmTaskServer::execute_task, this, std::placeholders::_1), goal_handle}.detach();
  }

  void execute_task(const std::shared_ptr<GoalHandleExecuteTask> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Executing task...");
    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<ExecuteTask::Feedback>();
    auto result = std::make_shared<ExecuteTask::Result>();

    auto start_time = std::chrono::steady_clock::now();

    try {
      // Create MoveGroupInterface
      auto move_group_arm = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
        shared_from_this(), "arm");
      
      auto move_group_gripper = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
        shared_from_this(), "gripper");

      // Handle different task types
      if (goal->task_name == "move_to_pose") {
        feedback->current_state = "Planning motion";
        feedback->progress = 0.3;
        goal_handle->publish_feedback(feedback);

        move_group_arm->setPoseTarget(goal->target_pose);
        
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        bool success = (move_group_arm->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

        if (success) {
          feedback->current_state = "Executing motion";
          feedback->progress = 0.6;
          goal_handle->publish_feedback(feedback);

          move_group_arm->execute(plan);
          
          result->success = true;
          result->message = "Successfully moved to target pose";
        } else {
          result->success = false;
          result->message = "Failed to plan motion";
        }
      }
      else if (goal->task_name == "move_to_named") {
        if (!goal->parameters.empty()) {
          feedback->current_state = "Moving to named pose";
          feedback->progress = 0.5;
          goal_handle->publish_feedback(feedback);

          move_group_arm->setNamedTarget(goal->parameters[0]);
          bool success = (move_group_arm->move() == moveit::core::MoveItErrorCode::SUCCESS);
          
          result->success = success;
          result->message = success ? "Reached named pose" : "Failed to reach named pose";
        }
      }
      else if (goal->task_name == "control_gripper") {
        feedback->current_state = "Controlling gripper";
        feedback->progress = 0.5;
        goal_handle->publish_feedback(feedback);

        std::vector<double> gripper_joint_values = {goal->gripper_position, -goal->gripper_position};
        move_group_gripper->setJointValueTarget(gripper_joint_values);
        bool success = (move_group_gripper->move() == moveit::core::MoveItErrorCode::SUCCESS);
        
        result->success = success;
        result->message = success ? "Gripper controlled" : "Failed to control gripper";
      }
      else {
        result->success = false;
        result->message = "Unknown task: " + goal->task_name;
      }

    } catch (const std::exception& e) {
      result->success = false;
      result->message = std::string("Exception: ") + e.what();
      RCLCPP_ERROR(this->get_logger(), "%s", result->message.c_str());
    }

    auto end_time = std::chrono::steady_clock::now();
    result->execution_time = std::chrono::duration<float>(end_time - start_time).count();

    if (rclcpp::ok()) {
      goal_handle->succeed(result);
      RCLCPP_INFO(this->get_logger(), "Task completed: %s", result->message.c_str());
    }
  }

  // PickPlace action handlers
  rclcpp_action::GoalResponse handle_pick_place_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const PickPlace::Goal> goal)
  {
    (void)uuid;
    (void)goal;
    RCLCPP_INFO(this->get_logger(), "Received pick and place request");
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_pick_place_cancel(
    const std::shared_ptr<GoalHandlePickPlace> goal_handle)
  {
    (void)goal_handle;
    RCLCPP_INFO(this->get_logger(), "Received request to cancel pick and place");
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_pick_place_accepted(const std::shared_ptr<GoalHandlePickPlace> goal_handle)
  {
    std::thread{std::bind(&XArmTaskServer::execute_pick_place, this, std::placeholders::_1), goal_handle}.detach();
  }

  void execute_pick_place(const std::shared_ptr<GoalHandlePickPlace> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Executing pick and place...");
    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<PickPlace::Feedback>();
    auto result = std::make_shared<PickPlace::Result>();

    auto start_time = std::chrono::steady_clock::now();

    try {
      auto move_group_arm = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
        shared_from_this(), "arm");
      
      auto move_group_gripper = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
        shared_from_this(), "gripper");

      // Phase 1: Approach pick position
      feedback->current_phase = "approaching";
      feedback->progress = 0.1;
      goal_handle->publish_feedback(feedback);

      geometry_msgs::msg::Pose approach_pose = goal->pick_pose;
      approach_pose.position.z += goal->approach_distance;
      
      move_group_arm->setPoseTarget(approach_pose);
      if (move_group_arm->move() != moveit::core::MoveItErrorCode::SUCCESS) {
        throw std::runtime_error("Failed to approach pick position");
      }

      // Phase 2: Open gripper
      feedback->current_phase = "opening_gripper";
      feedback->progress = 0.25;
      goal_handle->publish_feedback(feedback);

      move_group_gripper->setNamedTarget("open");
      move_group_gripper->move();

      // Phase 3: Move to pick position
      feedback->current_phase = "grasping";
      feedback->progress = 0.4;
      goal_handle->publish_feedback(feedback);

      move_group_arm->setPoseTarget(goal->pick_pose);
      if (move_group_arm->move() != moveit::core::MoveItErrorCode::SUCCESS) {
        throw std::runtime_error("Failed to reach pick position");
      }

      // Phase 4: Close gripper
      move_group_gripper->setNamedTarget("close");
      move_group_gripper->move();
      std::this_thread::sleep_for(500ms);

      // Phase 5: Lift object
      feedback->current_phase = "lifting";
      feedback->progress = 0.55;
      goal_handle->publish_feedback(feedback);

      geometry_msgs::msg::Pose retreat_pose = goal->pick_pose;
      retreat_pose.position.z += goal->retreat_distance;
      move_group_arm->setPoseTarget(retreat_pose);
      move_group_arm->move();

      // Phase 6: Move to place approach
      feedback->current_phase = "moving";
      feedback->progress = 0.7;
      goal_handle->publish_feedback(feedback);

      geometry_msgs::msg::Pose place_approach = goal->place_pose;
      place_approach.position.z += goal->approach_distance;
      move_group_arm->setPoseTarget(place_approach);
      move_group_arm->move();

      // Phase 7: Move to place position
      feedback->current_phase = "placing";
      feedback->progress = 0.85;
      goal_handle->publish_feedback(feedback);

      move_group_arm->setPoseTarget(goal->place_pose);
      if (move_group_arm->move() != moveit::core::MoveItErrorCode::SUCCESS) {
        throw std::runtime_error("Failed to reach place position");
      }

      // Phase 8: Open gripper
      move_group_gripper->setNamedTarget("open");
      move_group_gripper->move();
      std::this_thread::sleep_for(500ms);

      // Phase 9: Retreat
      feedback->current_phase = "retreating";
      feedback->progress = 0.95;
      goal_handle->publish_feedback(feedback);

      geometry_msgs::msg::Pose final_retreat = goal->place_pose;
      final_retreat.position.z += goal->retreat_distance;
      move_group_arm->setPoseTarget(final_retreat);
      move_group_arm->move();

      result->success = true;
      result->message = "Pick and place completed successfully";

    } catch (const std::exception& e) {
      result->success = false;
      result->message = std::string("Pick and place failed: ") + e.what();
      RCLCPP_ERROR(this->get_logger(), "%s", result->message.c_str());
    }

    auto end_time = std::chrono::steady_clock::now();
    result->total_time = std::chrono::duration<float>(end_time - start_time).count();

    if (rclcpp::ok()) {
      goal_handle->succeed(result);
      RCLCPP_INFO(this->get_logger(), "Pick and place result: %s", result->message.c_str());
    }
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<XArmTaskServer>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
