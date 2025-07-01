  #include <rclcpp/rclcpp.hpp>
  #include <rclcpp_action/rclcpp_action.hpp>
  #include <rclcpp_components/register_node_macro.hpp>
  #include "arduinobot_msgs/action/arduinobot_task.hpp"
  #include <moveit/move_group_interface/move_group_interface.hpp>

  #include <rclcpp/rclcpp.hpp>

  #include <moveit/planning_scene/planning_scene.hpp>

  #include <moveit/task_constructor/task.h>

  #include <moveit/task_constructor/stages/fixed_state.h>
  #include <moveit/task_constructor/stages/compute_ik.h>
  #include <moveit/task_constructor/stages/move_to.h>
  #include <moveit/task_constructor/cost_terms.h>

  #include <memory>

  using namespace moveit::task_constructor;
  using namespace moveit::task_constructor::stages;
  using namespace std::placeholders;
  static const rclcpp::Logger LOGGER = rclcpp::get_logger("tast_server_ik");

  namespace arduinobot_remote_ik
  {
  class TaskServerIK : public rclcpp::Node
  {
  public:
    explicit TaskServerIK(const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
      : Node("task_server_ik", options)
    {
      RCLCPP_INFO(get_logger(), "Starting the Server IK");
      action_server_ = rclcpp_action::create_server<arduinobot_msgs::action::ArduinobotTask>(
          this, "task_server", std::bind(&TaskServerIK::goalCallback, this, _1, _2),
          std::bind(&TaskServerIK::cancelCallback, this, _1),
          std::bind(&TaskServerIK::acceptedCallback, this, _1));
    }

  private:
    
    rclcpp_action::Server<arduinobot_msgs::action::ArduinobotTask>::SharedPtr action_server_;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> arm_move_group_, gripper_move_group_;
  
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
      std::thread{ std::bind(&TaskServerIK::execute, this, _1), goal_handle }.detach();
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
      
      auto target_pose = std::make_shared<geometry_msgs::msg::PoseStamped>();

      if (goal_handle->get_goal()->task_number == 0)
      {
          target_pose->header.frame_id = "world";
          target_pose->pose.position.x = 0.961;
          target_pose->pose.position.y = 0.421;
          target_pose->pose.position.z = 0.818;
          target_pose->pose.orientation.w = 1.0;
          RCLCPP_INFO(get_logger(), "pose init");
      }
      
      else
      {
        RCLCPP_ERROR(get_logger(), "Invalid Task Number");
        return;
      }
      
      RCLCPP_INFO(get_logger(), "ik init");
      Task task("arm_to_pose_task");
      task.stages()->setName("ik task");
      task.loadRobotModel(shared_from_this());

      // === Khởi tạo Start State ===
      RCLCPP_INFO(get_logger(), "Khởi tạo Start State");
      auto current_state = std::make_shared<moveit::core::RobotState>(*arm_move_group_->getCurrentState());
      current_state->update();

      auto scene = std::make_shared<planning_scene::PlanningScene>(arm_move_group_->getRobotModel());
      scene->setCurrentState(*current_state);

      
      auto start = std::make_unique<FixedState>("start");
      start->setState(scene);

      // Thêm vào task
      RCLCPP_INFO(get_logger(), "Thêm vào task");

      task.add(std::move(start));

      // === Stage tính toán IK ===
      RCLCPP_INFO(get_logger(), "Stage tính toán IK");
      auto ik_stage = std::make_unique<ComputeIK>("move to target");
      ik_stage->setGroup("arm");
      ik_stage->setEndEffector("gripper");  // hoặc end-effector group name của bạn
      ik_stage->setIKFrame("claw_support"); // link cuối của arm (end-effector)
      ik_stage->setTargetPose(*target_pose);
      ik_stage->properties().configureInitFrom(Stage::PARENT);  // Kế thừa thông tin scene

      task.add(std::move(ik_stage));

      RCLCPP_INFO(get_logger(), "execute plan");
      auto move_to = std::make_unique<MoveTo>("execute plan");
      move_to->setGroup("arm");
      move_to->properties().configureInitFrom(Stage::PARENT);
      task.add(std::move(move_to));
      // // === Khởi tạo task ===
      // if (!task.init()) {
      //     RCLCPP_ERROR(get_logger(), "Failed to initialize task!");
      //     result->success = false;
      //     goal_handle->abort(result);
      //     return;
      // }

      RCLCPP_INFO(get_logger(), "task.init(); 1");
      task.init();
      RCLCPP_INFO(get_logger(), "task.init(); 2");

      if (!task.plan(1)) {
        RCLCPP_ERROR(get_logger(), "Planning failed!");
        return;
      }
      RCLCPP_INFO(get_logger(), "task.init(); 4");

      
    }
  };

    // void MTCTaskNode::doTask()
    // {
    //   task_ = this::task;

    //   try
    //   {
    //     task_.init();
    //   }
    //   catch (mtc::InitStageException& e)
    //   {
    //     RCLCPP_ERROR_STREAM(LOGGER, e);
    //     return;
    //   }

    //   if (!task_.plan(5))
    //   {
    //     RCLCPP_ERROR_STREAM(LOGGER, "Task planning failed");
    //     return;
    //   }
    //   task_.introspection().publishSolution(*task_.solutions().front());

    //   auto result = task_.execute(*task_.solutions().front());
    //   if (result.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS)
    //   {
    //     RCLCPP_ERROR_STREAM(LOGGER, "Task execution failed");
    //     return;
    //   }

    //   return;
    // }
  }  // namespace arduinobot_remote

    
  RCLCPP_COMPONENTS_REGISTER_NODE(arduinobot_remote_ik::TaskServerIK)
