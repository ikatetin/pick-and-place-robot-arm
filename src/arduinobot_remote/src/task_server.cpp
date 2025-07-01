#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include "arduinobot_msgs/action/arduinobot_task.hpp"
#include <moveit/move_group_interface/move_group_interface.h>
#include <memory>


using namespace std::placeholders;
using std::placeholders::_1;
namespace arduinobot_remote
{
  class TaskServer : public rclcpp::Node
  { 
  public:
    explicit TaskServer(const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
      : Node("task_server", options)
    {
      RCLCPP_INFO(get_logger(), "Starting the Server");
      action_server_ = rclcpp_action::create_server<arduinobot_msgs::action::ArduinobotTask>(
          this, "task_server", std::bind(&TaskServer::goalCallback, this, _1, _2),
          std::bind(&TaskServer::cancelCallback, this, _1),
          std::bind(&TaskServer::acceptedCallback, this, _1));
    }

  private:
    rclcpp_action::Server<arduinobot_msgs::action::ArduinobotTask>::SharedPtr action_server_;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> arm_move_group_, gripper_move_group_;
    std::vector<double> arm_joint_goal_, gripper_joint_goal_;

    rclcpp_action::GoalResponse goalCallback(
        const rclcpp_action::GoalUUID& uuid,
        std::shared_ptr<const arduinobot_msgs::action::ArduinobotTask::Goal> goal)
    {
      RCLCPP_INFO(get_logger(), "Received goal request with id %d", goal->task_number);
      (void)uuid;
      return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse cancelCallback(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<arduinobot_msgs::action::ArduinobotTask>> goal_handle)
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
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<arduinobot_msgs::action::ArduinobotTask>> goal_handle)
    {
      // this needs to return quickly to avoid blocking the executor, so spin up a new thread
      std::thread{ std::bind(&TaskServer::execute, this, _1), goal_handle }.detach();
    }

    void execute(const std::shared_ptr<rclcpp_action::ServerGoalHandle<arduinobot_msgs::action::ArduinobotTask>> goal_handle)
    {
      RCLCPP_INFO(get_logger(), "Executing goal");
      auto result = std::make_shared<arduinobot_msgs::action::ArduinobotTask::Result>();

      // MoveIt 2 Interface
      if(!arm_move_group_){
        arm_move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(shared_from_this(), "arm");
      }
      if(!gripper_move_group_){
        gripper_move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(shared_from_this(), "gripper");
      }

      if (goal_handle->get_goal()->task_number == 0)
      {
        arm_joint_goal_ = {0.0, 0.0, 0.0, 0.0};
        gripper_joint_goal_ = {-0.7, 0.7};
      }
      else if (goal_handle->get_goal()->task_number == 1)
      {
        arm_joint_goal_ = {-1.14, -0.6, -0.07, 0.0};
        gripper_joint_goal_ = {0.0, 0.0};
      }
      else if (goal_handle->get_goal()->task_number == 2)
      {
        arm_joint_goal_ = {-1.57,0.0,-0.9, 0.0};
        gripper_joint_goal_ = {0.0, 0.0};
      }
      // else if (goal_handle->get_goal()->task_number == 4)
      // {
      //   arm_joint_goal_ = {0.0,M_PI/3,M_PI/3, 0.0};
      //   gripper_joint_goal_ = {0.0, 0.0};
      // }
      else if (goal_handle->get_goal()->task_number == 3)
      {
        Eigen::Vector3d target_pose(0.0, 1.2, 0.2);
        // Eigen::Isometry3d base_plate_transform1 = Eigen::Isometry3d::Identity();
        // base_plate_transform1.pretranslate(Eigen::Vector3d(0.0, 0.0, 0.307));
        // base_plate_transform1.rotate(Eigen::AngleAxisd(M_PI/2, Eigen::Vector3d::UnitZ())); // 90 độ quanh Z
        // base_plate_transform1.rotate(Eigen::AngleAxisd(M_PI/2, Eigen::Vector3d::UnitZ())); // 90 độ quanh Z

        // Eigen::Vector3d target_local = base_plate_transform1.inverse() * target_pose;
        arm_joint_goal_ = computeIK(target_pose.x(), target_pose.y(), target_pose.z());
        gripper_joint_goal_ = {-1.2, 1.2};
        // return;
      }
      else if (goal_handle->get_goal()->task_number == 4)
      {
        Eigen::Vector3d target_pose(1.0, 1.0, 0.2);
        // Eigen::Isometry3d base_plate_transform1 = Eigen::Isometry3d::Identity();
        // base_plate_transform1.pretranslate(Eigen::Vector3d(0.0, 0.0, 0.307));
        // base_plate_transform1.rotate(Eigen::AngleAxisd(M_PI/2, Eigen::Vector3d::UnitZ())); // 90 độ quanh Z
        // base_plate_transform1.rotate(Eigen::AngleAxisd(M_PI/2, Eigen::Vector3d::UnitZ())); // 90 độ quanh Z

        // Eigen::Vector3d target_local = base_plate_transform1.inverse() * target_pose;
        arm_joint_goal_ = computeIK(target_pose.x(), target_pose.y(), target_pose.z());
        gripper_joint_goal_ = {-1.2, 1.2};
        // return;
      }
      else
      {
        RCLCPP_ERROR(get_logger(), "Invalid Task Number");
        return;
      }

      arm_move_group_->setStartState(*arm_move_group_->getCurrentState());
      gripper_move_group_->setStartState(*gripper_move_group_->getCurrentState());

      bool arm_within_bounds = arm_move_group_->setJointValueTarget(arm_joint_goal_);
      bool gripper_within_bounds = gripper_move_group_->setJointValueTarget(gripper_joint_goal_);
      if (!arm_within_bounds | !gripper_within_bounds)
      {
        RCLCPP_WARN(get_logger(),
                    "Target joint position(s) were outside of limits, but we will plan and clamp to the limits ");
        return;
      }

      moveit::planning_interface::MoveGroupInterface::Plan arm_plan;
      moveit::planning_interface::MoveGroupInterface::Plan gripper_plan;
      bool arm_plan_success = (arm_move_group_->plan(arm_plan) == moveit::core::MoveItErrorCode::SUCCESS);
      bool gripper_plan_success = (gripper_move_group_->plan(gripper_plan) == moveit::core::MoveItErrorCode::SUCCESS);
      
      if(arm_plan_success && gripper_plan_success)
      {
        RCLCPP_INFO(get_logger(), "Planner SUCCEED, moving the arme and the gripper");
        arm_move_group_->move();
        gripper_move_group_->move();
      }
      else
      {
        RCLCPP_ERROR(get_logger(), "One or more planners failed!");
        return;
      }
    
      result->success = true;
      goal_handle->succeed(result);
      RCLCPP_INFO(get_logger(), "Goal succeeded");
      rclcpp::sleep_for(std::chrono::seconds(1));

      // Lấy robot state hiện tại
      moveit::core::RobotStatePtr current_state = arm_move_group_->getCurrentState();
      const std::string ee_link = "claw_support";  // thay bằng tên link end-effector của bạn

      // Lấy transform toàn cục của link end-effector
      const Eigen::Isometry3d& ee_transform = current_state->getGlobalLinkTransform(ee_link);
      const Eigen::Isometry3d& base_plate_transform = current_state->getGlobalLinkTransform("base_plate");
      const Eigen::Isometry3d& forward_drive_arm_transform = current_state->getGlobalLinkTransform("forward_drive_arm");
      const Eigen::Isometry3d& horizontal_arm_transform = current_state->getGlobalLinkTransform("horizontal_arm");
      const Eigen::Isometry3d& gripper_left_transform = current_state->getGlobalLinkTransform("gripper_left");
      const Eigen::Isometry3d& gripper_right_transform = current_state->getGlobalLinkTransform("gripper_right");

      // In ra pose
      RCLCPP_INFO(get_logger(), "End Effector Position: [%.3f, %.3f, %.3f]",ee_transform.translation().x(), ee_transform.translation().y(), ee_transform.translation().z());
      // RCLCPP_INFO(get_logger(), "End Effector Position: [%.3f, %.3f, %.3f]",ee_transform.translation().x(), ee_transform.translation().y(), ee_transform.translation().z());
      // RCLCPP_INFO(get_logger(), "End Effector Position: [%.3f, %.3f, %.3f]",ee_transform.translation().x(), ee_transform.translation().y(), ee_transform.translation().z());

      auto quat = Eigen::Quaterniond(ee_transform.rotation());
      RCLCPP_INFO(get_logger(), "Orientation (quaternion): [x=%.3f, y=%.3f, z=%.3f, w=%.3f]",
                  quat.x(), quat.y(), quat.z(), quat.w());

      //In ra khoang cach
      RCLCPP_INFO(get_logger(), "L1: %.3f", tinhKhoangCach(forward_drive_arm_transform, horizontal_arm_transform));
      RCLCPP_INFO(get_logger(), "L2: %.3f", tinhKhoangCach(horizontal_arm_transform, ee_transform));

      //0.351,  0.8,  0.82
  //     End Effector Position: [0.961, 0.421, 0.818]
  // [task_server_node-1] [INFO] [1749062986.031972586] [task_server]: Orientation (quaternion): [x=-0.281, y=0.188, z=-0.523, w=0.783]

    }

    double tinhKhoangCach(const Eigen::Isometry3d& link1, const Eigen::Isometry3d& link2) {
    // Lấy vị trí translation của 2 link
    Eigen::Vector3d pos1 = link1.translation();
    Eigen::Vector3d pos2 = link2.translation();

    // Tính khoảng cách Euclidean
    double distance = (pos1 - pos2).norm();

    return distance;
    }
  
    std::vector<double>  computeIK(float x, float y, float z) {
    // std::vector<double>  computeIK() {
      // float x = 0.961 ;
      // float y = 0.421 ;
      // float z = 0.818;
      

      // float x = 0.0 ;
      // float y = 0.8 ;
      // float z = 0.307;
      float L1 = 0.8;
      float L2 = 0.82;
      
      float base_z = 0.657;
      float theta1 = atan2(y, x) - M_PI/2;  // góc xoay đế
      RCLCPP_INFO(get_logger(), "x: %.3f", x);
      RCLCPP_INFO(get_logger(), "y: %.3f", y);
      RCLCPP_INFO(get_logger(), "z: %.3f", z);
      RCLCPP_INFO(get_logger(), "theta1: %.3f", radToDeg(theta1));

      float r = std::sqrt(x*x + y*y);
      float z_rel = z - base_z;
      RCLCPP_INFO(get_logger(), "z_rel: %.3f", z_rel);

      float D = std::sqrt(r*r + z_rel*z_rel);
      RCLCPP_INFO(get_logger(), "D: %.3f", D);

      if (D > (L1 + L2)) {
          throw std::runtime_error("Target is out of reach");
      }

      float angle_a = std::acos((L1*L1 + D*D - L2*L2) / (2 * L1 * D));
      RCLCPP_INFO(get_logger(), "angle_a: %.3f", radToDeg(angle_a));
      float angle_b = std::atan2(z_rel, r);
      RCLCPP_INFO(get_logger(), "angle_b: %.3f", radToDeg(angle_b));
      float theta2 = angle_b + angle_a;
      theta2 = -M_PI/2 + theta2;
      RCLCPP_INFO(get_logger(), "theta2: %.3f", radToDeg(theta2));

      float angle_c = std::acos((L1*L1 + L2*L2 - D*D) / (2 * L1 * L2));
      RCLCPP_INFO(get_logger(), "angle_c: %.3f", radToDeg(angle_c));
      float theta3 = angle_c;
      RCLCPP_INFO(get_logger(), "theta3 b4: %.3f", radToDeg(theta3));
      
      float theta4 = M_PI/2 - (theta2 + theta3);
      RCLCPP_INFO(get_logger(), "theta4: %.3f", radToDeg(theta4));
      theta3 = -M_PI/2 + theta3;
      RCLCPP_INFO(get_logger(), "theta3: %.3f", radToDeg(theta3));
      return std::vector<double> {theta1, theta2, theta3, theta4};
  }

  float radToDeg(float rad) {
    return rad * 180.0f / M_PI;
  }
  
  };
}  // namespace arduinobot_remote

RCLCPP_COMPONENTS_REGISTER_NODE(arduinobot_remote::TaskServer)