// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from arduinobot_msgs:action/ArduinobotTask.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "arduinobot_msgs/action/arduinobot_task.hpp"


#ifndef ARDUINOBOT_MSGS__ACTION__DETAIL__ARDUINOBOT_TASK__BUILDER_HPP_
#define ARDUINOBOT_MSGS__ACTION__DETAIL__ARDUINOBOT_TASK__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "arduinobot_msgs/action/detail/arduinobot_task__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace arduinobot_msgs
{

namespace action
{

namespace builder
{

class Init_ArduinobotTask_Goal_gripper_goal
{
public:
  explicit Init_ArduinobotTask_Goal_gripper_goal(::arduinobot_msgs::action::ArduinobotTask_Goal & msg)
  : msg_(msg)
  {}
  ::arduinobot_msgs::action::ArduinobotTask_Goal gripper_goal(::arduinobot_msgs::action::ArduinobotTask_Goal::_gripper_goal_type arg)
  {
    msg_.gripper_goal = std::move(arg);
    return std::move(msg_);
  }

private:
  ::arduinobot_msgs::action::ArduinobotTask_Goal msg_;
};

class Init_ArduinobotTask_Goal_arm_goal
{
public:
  explicit Init_ArduinobotTask_Goal_arm_goal(::arduinobot_msgs::action::ArduinobotTask_Goal & msg)
  : msg_(msg)
  {}
  Init_ArduinobotTask_Goal_gripper_goal arm_goal(::arduinobot_msgs::action::ArduinobotTask_Goal::_arm_goal_type arg)
  {
    msg_.arm_goal = std::move(arg);
    return Init_ArduinobotTask_Goal_gripper_goal(msg_);
  }

private:
  ::arduinobot_msgs::action::ArduinobotTask_Goal msg_;
};

class Init_ArduinobotTask_Goal_task_number
{
public:
  Init_ArduinobotTask_Goal_task_number()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ArduinobotTask_Goal_arm_goal task_number(::arduinobot_msgs::action::ArduinobotTask_Goal::_task_number_type arg)
  {
    msg_.task_number = std::move(arg);
    return Init_ArduinobotTask_Goal_arm_goal(msg_);
  }

private:
  ::arduinobot_msgs::action::ArduinobotTask_Goal msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::arduinobot_msgs::action::ArduinobotTask_Goal>()
{
  return arduinobot_msgs::action::builder::Init_ArduinobotTask_Goal_task_number();
}

}  // namespace arduinobot_msgs


namespace arduinobot_msgs
{

namespace action
{

namespace builder
{

class Init_ArduinobotTask_Result_success
{
public:
  Init_ArduinobotTask_Result_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::arduinobot_msgs::action::ArduinobotTask_Result success(::arduinobot_msgs::action::ArduinobotTask_Result::_success_type arg)
  {
    msg_.success = std::move(arg);
    return std::move(msg_);
  }

private:
  ::arduinobot_msgs::action::ArduinobotTask_Result msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::arduinobot_msgs::action::ArduinobotTask_Result>()
{
  return arduinobot_msgs::action::builder::Init_ArduinobotTask_Result_success();
}

}  // namespace arduinobot_msgs


namespace arduinobot_msgs
{

namespace action
{

namespace builder
{

class Init_ArduinobotTask_Feedback_percentage
{
public:
  Init_ArduinobotTask_Feedback_percentage()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::arduinobot_msgs::action::ArduinobotTask_Feedback percentage(::arduinobot_msgs::action::ArduinobotTask_Feedback::_percentage_type arg)
  {
    msg_.percentage = std::move(arg);
    return std::move(msg_);
  }

private:
  ::arduinobot_msgs::action::ArduinobotTask_Feedback msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::arduinobot_msgs::action::ArduinobotTask_Feedback>()
{
  return arduinobot_msgs::action::builder::Init_ArduinobotTask_Feedback_percentage();
}

}  // namespace arduinobot_msgs


namespace arduinobot_msgs
{

namespace action
{

namespace builder
{

class Init_ArduinobotTask_SendGoal_Request_goal
{
public:
  explicit Init_ArduinobotTask_SendGoal_Request_goal(::arduinobot_msgs::action::ArduinobotTask_SendGoal_Request & msg)
  : msg_(msg)
  {}
  ::arduinobot_msgs::action::ArduinobotTask_SendGoal_Request goal(::arduinobot_msgs::action::ArduinobotTask_SendGoal_Request::_goal_type arg)
  {
    msg_.goal = std::move(arg);
    return std::move(msg_);
  }

private:
  ::arduinobot_msgs::action::ArduinobotTask_SendGoal_Request msg_;
};

class Init_ArduinobotTask_SendGoal_Request_goal_id
{
public:
  Init_ArduinobotTask_SendGoal_Request_goal_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ArduinobotTask_SendGoal_Request_goal goal_id(::arduinobot_msgs::action::ArduinobotTask_SendGoal_Request::_goal_id_type arg)
  {
    msg_.goal_id = std::move(arg);
    return Init_ArduinobotTask_SendGoal_Request_goal(msg_);
  }

private:
  ::arduinobot_msgs::action::ArduinobotTask_SendGoal_Request msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::arduinobot_msgs::action::ArduinobotTask_SendGoal_Request>()
{
  return arduinobot_msgs::action::builder::Init_ArduinobotTask_SendGoal_Request_goal_id();
}

}  // namespace arduinobot_msgs


namespace arduinobot_msgs
{

namespace action
{

namespace builder
{

class Init_ArduinobotTask_SendGoal_Response_stamp
{
public:
  explicit Init_ArduinobotTask_SendGoal_Response_stamp(::arduinobot_msgs::action::ArduinobotTask_SendGoal_Response & msg)
  : msg_(msg)
  {}
  ::arduinobot_msgs::action::ArduinobotTask_SendGoal_Response stamp(::arduinobot_msgs::action::ArduinobotTask_SendGoal_Response::_stamp_type arg)
  {
    msg_.stamp = std::move(arg);
    return std::move(msg_);
  }

private:
  ::arduinobot_msgs::action::ArduinobotTask_SendGoal_Response msg_;
};

class Init_ArduinobotTask_SendGoal_Response_accepted
{
public:
  Init_ArduinobotTask_SendGoal_Response_accepted()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ArduinobotTask_SendGoal_Response_stamp accepted(::arduinobot_msgs::action::ArduinobotTask_SendGoal_Response::_accepted_type arg)
  {
    msg_.accepted = std::move(arg);
    return Init_ArduinobotTask_SendGoal_Response_stamp(msg_);
  }

private:
  ::arduinobot_msgs::action::ArduinobotTask_SendGoal_Response msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::arduinobot_msgs::action::ArduinobotTask_SendGoal_Response>()
{
  return arduinobot_msgs::action::builder::Init_ArduinobotTask_SendGoal_Response_accepted();
}

}  // namespace arduinobot_msgs


namespace arduinobot_msgs
{

namespace action
{

namespace builder
{

class Init_ArduinobotTask_SendGoal_Event_response
{
public:
  explicit Init_ArduinobotTask_SendGoal_Event_response(::arduinobot_msgs::action::ArduinobotTask_SendGoal_Event & msg)
  : msg_(msg)
  {}
  ::arduinobot_msgs::action::ArduinobotTask_SendGoal_Event response(::arduinobot_msgs::action::ArduinobotTask_SendGoal_Event::_response_type arg)
  {
    msg_.response = std::move(arg);
    return std::move(msg_);
  }

private:
  ::arduinobot_msgs::action::ArduinobotTask_SendGoal_Event msg_;
};

class Init_ArduinobotTask_SendGoal_Event_request
{
public:
  explicit Init_ArduinobotTask_SendGoal_Event_request(::arduinobot_msgs::action::ArduinobotTask_SendGoal_Event & msg)
  : msg_(msg)
  {}
  Init_ArduinobotTask_SendGoal_Event_response request(::arduinobot_msgs::action::ArduinobotTask_SendGoal_Event::_request_type arg)
  {
    msg_.request = std::move(arg);
    return Init_ArduinobotTask_SendGoal_Event_response(msg_);
  }

private:
  ::arduinobot_msgs::action::ArduinobotTask_SendGoal_Event msg_;
};

class Init_ArduinobotTask_SendGoal_Event_info
{
public:
  Init_ArduinobotTask_SendGoal_Event_info()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ArduinobotTask_SendGoal_Event_request info(::arduinobot_msgs::action::ArduinobotTask_SendGoal_Event::_info_type arg)
  {
    msg_.info = std::move(arg);
    return Init_ArduinobotTask_SendGoal_Event_request(msg_);
  }

private:
  ::arduinobot_msgs::action::ArduinobotTask_SendGoal_Event msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::arduinobot_msgs::action::ArduinobotTask_SendGoal_Event>()
{
  return arduinobot_msgs::action::builder::Init_ArduinobotTask_SendGoal_Event_info();
}

}  // namespace arduinobot_msgs


namespace arduinobot_msgs
{

namespace action
{

namespace builder
{

class Init_ArduinobotTask_GetResult_Request_goal_id
{
public:
  Init_ArduinobotTask_GetResult_Request_goal_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::arduinobot_msgs::action::ArduinobotTask_GetResult_Request goal_id(::arduinobot_msgs::action::ArduinobotTask_GetResult_Request::_goal_id_type arg)
  {
    msg_.goal_id = std::move(arg);
    return std::move(msg_);
  }

private:
  ::arduinobot_msgs::action::ArduinobotTask_GetResult_Request msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::arduinobot_msgs::action::ArduinobotTask_GetResult_Request>()
{
  return arduinobot_msgs::action::builder::Init_ArduinobotTask_GetResult_Request_goal_id();
}

}  // namespace arduinobot_msgs


namespace arduinobot_msgs
{

namespace action
{

namespace builder
{

class Init_ArduinobotTask_GetResult_Response_result
{
public:
  explicit Init_ArduinobotTask_GetResult_Response_result(::arduinobot_msgs::action::ArduinobotTask_GetResult_Response & msg)
  : msg_(msg)
  {}
  ::arduinobot_msgs::action::ArduinobotTask_GetResult_Response result(::arduinobot_msgs::action::ArduinobotTask_GetResult_Response::_result_type arg)
  {
    msg_.result = std::move(arg);
    return std::move(msg_);
  }

private:
  ::arduinobot_msgs::action::ArduinobotTask_GetResult_Response msg_;
};

class Init_ArduinobotTask_GetResult_Response_status
{
public:
  Init_ArduinobotTask_GetResult_Response_status()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ArduinobotTask_GetResult_Response_result status(::arduinobot_msgs::action::ArduinobotTask_GetResult_Response::_status_type arg)
  {
    msg_.status = std::move(arg);
    return Init_ArduinobotTask_GetResult_Response_result(msg_);
  }

private:
  ::arduinobot_msgs::action::ArduinobotTask_GetResult_Response msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::arduinobot_msgs::action::ArduinobotTask_GetResult_Response>()
{
  return arduinobot_msgs::action::builder::Init_ArduinobotTask_GetResult_Response_status();
}

}  // namespace arduinobot_msgs


namespace arduinobot_msgs
{

namespace action
{

namespace builder
{

class Init_ArduinobotTask_GetResult_Event_response
{
public:
  explicit Init_ArduinobotTask_GetResult_Event_response(::arduinobot_msgs::action::ArduinobotTask_GetResult_Event & msg)
  : msg_(msg)
  {}
  ::arduinobot_msgs::action::ArduinobotTask_GetResult_Event response(::arduinobot_msgs::action::ArduinobotTask_GetResult_Event::_response_type arg)
  {
    msg_.response = std::move(arg);
    return std::move(msg_);
  }

private:
  ::arduinobot_msgs::action::ArduinobotTask_GetResult_Event msg_;
};

class Init_ArduinobotTask_GetResult_Event_request
{
public:
  explicit Init_ArduinobotTask_GetResult_Event_request(::arduinobot_msgs::action::ArduinobotTask_GetResult_Event & msg)
  : msg_(msg)
  {}
  Init_ArduinobotTask_GetResult_Event_response request(::arduinobot_msgs::action::ArduinobotTask_GetResult_Event::_request_type arg)
  {
    msg_.request = std::move(arg);
    return Init_ArduinobotTask_GetResult_Event_response(msg_);
  }

private:
  ::arduinobot_msgs::action::ArduinobotTask_GetResult_Event msg_;
};

class Init_ArduinobotTask_GetResult_Event_info
{
public:
  Init_ArduinobotTask_GetResult_Event_info()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ArduinobotTask_GetResult_Event_request info(::arduinobot_msgs::action::ArduinobotTask_GetResult_Event::_info_type arg)
  {
    msg_.info = std::move(arg);
    return Init_ArduinobotTask_GetResult_Event_request(msg_);
  }

private:
  ::arduinobot_msgs::action::ArduinobotTask_GetResult_Event msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::arduinobot_msgs::action::ArduinobotTask_GetResult_Event>()
{
  return arduinobot_msgs::action::builder::Init_ArduinobotTask_GetResult_Event_info();
}

}  // namespace arduinobot_msgs


namespace arduinobot_msgs
{

namespace action
{

namespace builder
{

class Init_ArduinobotTask_FeedbackMessage_feedback
{
public:
  explicit Init_ArduinobotTask_FeedbackMessage_feedback(::arduinobot_msgs::action::ArduinobotTask_FeedbackMessage & msg)
  : msg_(msg)
  {}
  ::arduinobot_msgs::action::ArduinobotTask_FeedbackMessage feedback(::arduinobot_msgs::action::ArduinobotTask_FeedbackMessage::_feedback_type arg)
  {
    msg_.feedback = std::move(arg);
    return std::move(msg_);
  }

private:
  ::arduinobot_msgs::action::ArduinobotTask_FeedbackMessage msg_;
};

class Init_ArduinobotTask_FeedbackMessage_goal_id
{
public:
  Init_ArduinobotTask_FeedbackMessage_goal_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ArduinobotTask_FeedbackMessage_feedback goal_id(::arduinobot_msgs::action::ArduinobotTask_FeedbackMessage::_goal_id_type arg)
  {
    msg_.goal_id = std::move(arg);
    return Init_ArduinobotTask_FeedbackMessage_feedback(msg_);
  }

private:
  ::arduinobot_msgs::action::ArduinobotTask_FeedbackMessage msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::arduinobot_msgs::action::ArduinobotTask_FeedbackMessage>()
{
  return arduinobot_msgs::action::builder::Init_ArduinobotTask_FeedbackMessage_goal_id();
}

}  // namespace arduinobot_msgs

#endif  // ARDUINOBOT_MSGS__ACTION__DETAIL__ARDUINOBOT_TASK__BUILDER_HPP_
