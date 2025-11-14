#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include "xarm_remote/action/xarm_task.hpp"
#include <moveit/move_group_interface/move_group_interface.h>

#include <memory>

using namespace std::placeholders;

namespace xarm_remote
{
class TaskServerNumber : public rclcpp::Node
{
public:
  explicit TaskServerNumber(const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
    : Node("task_server_number", options)
  {
    RCLCPP_INFO(get_logger(), "Starting XArm Task Server (Number-based)");
    action_server_ = rclcpp_action::create_server<xarm_remote::action::XarmTask>(
        this, "task_server_number", 
        std::bind(&TaskServerNumber::goalCallback, this, _1, _2),
        std::bind(&TaskServerNumber::cancelCallback, this, _1),
        std::bind(&TaskServerNumber::acceptedCallback, this, _1));
    
    RCLCPP_INFO(get_logger(), "Task Server ready! Send task numbers:");
    RCLCPP_INFO(get_logger(), "  0 = Home position");
    RCLCPP_INFO(get_logger(), "  1 = Pose 1");
    RCLCPP_INFO(get_logger(), "  2 = Pose 2");
    RCLCPP_INFO(get_logger(), "  3 = Pose 3");
    RCLCPP_INFO(get_logger(), "  10 = Open gripper");
    RCLCPP_INFO(get_logger(), "  11 = Close gripper");
  }

private:
  rclcpp_action::Server<xarm_remote::action::XarmTask>::SharedPtr action_server_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> arm_move_group_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> gripper_move_group_;

  rclcpp_action::GoalResponse goalCallback(
      const rclcpp_action::GoalUUID& uuid,
      std::shared_ptr<const xarm_remote::action::XarmTask::Goal> goal)
  {
    RCLCPP_INFO(get_logger(), "Received goal request with task number %d", goal->task_number);
    (void)uuid;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse cancelCallback(
      const std::shared_ptr<rclcpp_action::ServerGoalHandle<xarm_remote::action::XarmTask>> goal_handle)
  {
    (void)goal_handle;
    RCLCPP_INFO(get_logger(), "Received request to cancel goal");
    if(arm_move_group_){
      arm_move_group_->stop();
    }
    if(gripper_move_group_){
      gripper_move_group_->stop();
    }
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void acceptedCallback(
      const std::shared_ptr<rclcpp_action::ServerGoalHandle<xarm_remote::action::XarmTask>> goal_handle)
  {
    // Spin up a new thread to avoid blocking the executor
    std::thread{ std::bind(&TaskServerNumber::execute, this, _1), goal_handle }.detach();
  }

  void execute(const std::shared_ptr<rclcpp_action::ServerGoalHandle<xarm_remote::action::XarmTask>> goal_handle)
  {
    RCLCPP_INFO(get_logger(), "Executing task number %d", goal_handle->get_goal()->task_number);
    auto result = std::make_shared<xarm_remote::action::XarmTask::Result>();
    auto feedback = std::make_shared<xarm_remote::action::XarmTask::Feedback>();

    // Initialize MoveIt interfaces
    if(!arm_move_group_){
      arm_move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
        shared_from_this(), "arm");
    }
    if(!gripper_move_group_){
      gripper_move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
        shared_from_this(), "gripper");
    }

    int task_number = goal_handle->get_goal()->task_number;
    bool success = false;

    try {
      // Arm tasks (0-9)
      if (task_number >= 0 && task_number <= 3)
      {
        std::string target_name;
        switch(task_number) {
          case 0:
            target_name = "home";
            break;
          case 1:
            target_name = "pose_1";
            break;
          case 2:
            target_name = "pose_2";
            break;
          case 3:
            target_name = "pose_3";
            break;
        }

        feedback->status = "Planning motion to " + target_name;
        goal_handle->publish_feedback(feedback);
        RCLCPP_INFO(get_logger(), "Moving arm to: %s", target_name.c_str());

        arm_move_group_->setStartState(*arm_move_group_->getCurrentState());
        arm_move_group_->setNamedTarget(target_name);

        moveit::planning_interface::MoveGroupInterface::Plan arm_plan;
        bool plan_success = (arm_move_group_->plan(arm_plan) == moveit::core::MoveItErrorCode::SUCCESS);
        
        if(plan_success)
        {
          feedback->status = "Executing motion to " + target_name;
          goal_handle->publish_feedback(feedback);
          RCLCPP_INFO(get_logger(), "Plan succeeded, moving arm");
          
          bool move_success = (arm_move_group_->execute(arm_plan) == moveit::core::MoveItErrorCode::SUCCESS);
          if(move_success) {
            result->success = true;
            result->message = "Successfully moved to " + target_name;
            success = true;
          } else {
            result->success = false;
            result->message = "Failed to execute motion to " + target_name;
          }
        }
        else
        {
          RCLCPP_ERROR(get_logger(), "Planning failed for %s", target_name.c_str());
          result->success = false;
          result->message = "Planning failed for " + target_name;
        }
      }
      // Gripper tasks (10-11)
      else if (task_number == 10 || task_number == 11)
      {
        std::string gripper_target = (task_number == 10) ? "open" : "close";
        
        feedback->status = "Controlling gripper: " + gripper_target;
        goal_handle->publish_feedback(feedback);
        RCLCPP_INFO(get_logger(), "Setting gripper to: %s", gripper_target.c_str());

        gripper_move_group_->setStartState(*gripper_move_group_->getCurrentState());
        gripper_move_group_->setNamedTarget(gripper_target);

        moveit::planning_interface::MoveGroupInterface::Plan gripper_plan;
        bool plan_success = (gripper_move_group_->plan(gripper_plan) == moveit::core::MoveItErrorCode::SUCCESS);
        
        if(plan_success)
        {
          RCLCPP_INFO(get_logger(), "Plan succeeded, moving gripper");
          bool move_success = (gripper_move_group_->execute(gripper_plan) == moveit::core::MoveItErrorCode::SUCCESS);
          
          if(move_success) {
            result->success = true;
            result->message = "Successfully set gripper to " + gripper_target;
            success = true;
          } else {
            result->success = false;
            result->message = "Failed to move gripper to " + gripper_target;
          }
        }
        else
        {
          RCLCPP_ERROR(get_logger(), "Planning failed for gripper");
          result->success = false;
          result->message = "Planning failed for gripper " + gripper_target;
        }
      }
      else
      {
        RCLCPP_ERROR(get_logger(), "Invalid task number: %d", task_number);
        result->success = false;
        result->message = "Invalid task number. Valid: 0-3 (arm poses), 10-11 (gripper)";
      }

    } catch (const std::exception& e) {
      RCLCPP_ERROR(get_logger(), "Exception during execution: %s", e.what());
      result->success = false;
      result->message = std::string("Exception: ") + e.what();
    }

    if (rclcpp::ok()) {
      if(success) {
        goal_handle->succeed(result);
        RCLCPP_INFO(get_logger(), "Task %d completed successfully", task_number);
      } else {
        goal_handle->abort(result);
        RCLCPP_ERROR(get_logger(), "Task %d failed: %s", task_number, result->message.c_str());
      }
    }
  }
};
}  // namespace xarm_remote

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<xarm_remote::TaskServerNumber>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
