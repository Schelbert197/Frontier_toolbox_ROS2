//This node is an example for working with the Nav2 stack to command
//the Jackal to a certain pose in the map.

#include <exception>
#include <vector>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_srvs/srv/empty.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "nav_client_cpp/srv/nav_to_pose.hpp"

//https://stackoverflow.com/questions/11714325/how-to-get-enum-item-name-from-its-value
#define STATES \
X(IDLE, "IDLE") \
X(SEND_GOAL, "SEND_GOAL") \
X(WAIT_FOR_GOAL_RESPONSE, "WAIT_FOR_GOAL_RESPONSE") \
X(WAIT_FOR_MOVEMENT_COMPLETE, "WAIT_FOR_MOVEMENT_COMPLETE")

#define X(state, name) state,
enum class State : size_t {STATES};
#undef X

#define X(state, name) name,
std::vector<std::string> STATE_NAMES = {STATES};
#undef X

//https://stackoverflow.com/questions/11421432/how-can-i-output-the-value-of-an-enum-class-in-c11
template <typename Enumeration>
auto to_value(Enumeration const value)
  -> typename std::underlying_type<Enumeration>::type
{
  return static_cast<typename std::underlying_type<Enumeration>::type>(value);
}

auto get_state_name(State state) {
  return STATE_NAMES[to_value(state)];
}

std::tuple<double, double, double> quaternion_to_rpy(const geometry_msgs::msg::Quaternion & q);
geometry_msgs::msg::Quaternion rpy_to_quaternion(double roll, double pitch, double yaw);

using namespace std::chrono_literals;

class NavToPose : public rclcpp::Node
{
public:
  NavToPose()
  : Node("nav_to_pose")
  {

    // Parameters
    auto param = rcl_interfaces::msg::ParameterDescriptor{};
    param.description = "The frame in which poses are sent.";
    declare_parameter("pose_frame", "map", param);
    goal_msg_.pose.header.frame_id = get_parameter("pose_frame").get_parameter_value().get<std::string>();

    // Timers
    timer_ = create_wall_timer(
      static_cast<std::chrono::milliseconds>(static_cast<int>(interval_ * 1000.0)), 
      std::bind(&NavToPose::timer_callback, this));

    // Services
    srv_nav_to_pose_ = create_service<nav_client_cpp::srv::NavToPose>(
      "jackal_nav_to_pose",
      std::bind(&NavToPose::srv_nav_to_pose_callback, this,
                std::placeholders::_1, std::placeholders::_2));
    srv_cancel_nav_ = create_service<std_srvs::srv::Empty>(
      "jackal_cancel_nav",
      std::bind(&NavToPose::cancel_nav_callback, this,
                std::placeholders::_1, std::placeholders::_2));

    // Publishers
    pub_waypoint_goal_ = create_publisher<std_msgs::msg::String>("jackal_goal", 10);

    // Action Clients
    act_nav_to_pose_ = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(
      this, "navigate_to_pose");

    RCLCPP_INFO_STREAM(get_logger(), "nav_to_pose node started");
  }

private:
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Service<nav_client_cpp::srv::NavToPose>::SharedPtr srv_nav_to_pose_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr srv_cancel_nav_;
  rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr act_nav_to_pose_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_waypoint_goal_;

  double rate_ = 100.0; //Hz
  double interval_ = 1.0 / rate_; //seconds
  State state_ = State::IDLE;
  State state_last_ = state_;
  State state_next_ = state_;
  std_msgs::msg::String jackal_goal_msg_ = std_msgs::msg::String();
  rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::Goal goal_msg_ {};
  bool goal_response_received_ = false;
  rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr goal_handle_ {};
  std::shared_ptr<const nav2_msgs::action::NavigateToPose::Feedback> feedback_ = nullptr;
  std::shared_ptr<rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::WrappedResult>
  result_ = nullptr;
  

  void srv_nav_to_pose_callback(
    const std::shared_ptr<nav_client_cpp::srv::NavToPose::Request> request,
    std::shared_ptr<nav_client_cpp::srv::NavToPose::Response>
  ) {

    // // Check if there is an active goal and cancel it before sending a new one
    // if (goal_handle_ && (
    //   goal_handle_->get_status() == rclcpp_action::GoalStatus::STATUS_ACCEPTED ||
    //   goal_handle_->get_status() == rclcpp_action::GoalStatus::STATUS_EXECUTING)) {
    //   RCLCPP_INFO(this->get_logger(), "Cancelling previous goal before sending a new one.");
    //   auto cancel_result_future = act_nav_to_pose_->async_cancel_goal(goal_handle_);

    //   // Wait for the cancel to complete before sending the new goal
    //   if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), cancel_result_future) 
    //       == rclcpp::FutureReturnCode::SUCCESS) {
    //     RCLCPP_INFO(this->get_logger(), "Previous goal cancelled successfully.");
    //   } else {
    //     RCLCPP_ERROR(this->get_logger(), "Failed to cancel previous goal.");
    //   }
    // }
    //Store requested pose
    goal_msg_.pose.pose.position.x = request->x;
    goal_msg_.pose.pose.position.y = request->y;
    goal_msg_.pose.pose.orientation = rpy_to_quaternion(0.0, 0.0, request->theta);
    RCLCPP_INFO(this->get_logger(), "Quaternion: x:%f, y:%f, z:%f, w:%f", goal_msg_.pose.pose.orientation.x, goal_msg_.pose.pose.orientation.y, goal_msg_.pose.pose.orientation.z, goal_msg_.pose.pose.orientation.w);

    //Initiate action call
    state_next_ = State::SEND_GOAL;
  }

  void cancel_nav_callback(
    const std::shared_ptr<std_srvs::srv::Empty::Request>,
    std::shared_ptr<std_srvs::srv::Empty::Response>
  ) {
    RCLCPP_INFO_STREAM(get_logger(), "Cancelling navigation.");
    act_nav_to_pose_->async_cancel_all_goals();
    state_next_ = State::IDLE;
  }

  void goal_response_callback(
    const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr & goal_handle
  ) {
    goal_response_received_ = true;
    goal_handle_ = goal_handle;
    RCLCPP_INFO_STREAM(get_logger(), "Goal response");
  }

  void feedback_callback(
    rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr,
    const std::shared_ptr<const nav2_msgs::action::NavigateToPose::Feedback> feedback
  ) {
    //Store result for later use
    feedback_ = feedback;

    if (feedback_) {
      auto [roll, pitch, yaw] = quaternion_to_rpy(feedback_->current_pose.pose.orientation);

      RCLCPP_DEBUG_STREAM(get_logger(), "x = " << feedback_->current_pose.pose.position.x
                                  << ", y = " << feedback_->current_pose.pose.position.y
                                  << ", theta = " << yaw
      );
    }
  }

  void result_callback(
    const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::WrappedResult & result
  ) {
    switch (result.code) {
      case rclcpp_action::ResultCode::SUCCEEDED:
        RCLCPP_INFO(this->get_logger(), "Goal succeeded");
        jackal_goal_msg_.data = "Succeeded";
        pub_waypoint_goal_->publish(jackal_goal_msg_);
        break;
      case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
        jackal_goal_msg_.data = "Aborted";
        pub_waypoint_goal_->publish(jackal_goal_msg_);
        return;
      case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
        jackal_goal_msg_.data = "Canceled";
        pub_waypoint_goal_->publish(jackal_goal_msg_);
        return;
      default:
        RCLCPP_ERROR(this->get_logger(), "Unknown result code");
        jackal_goal_msg_.data = "Unknown";
        pub_waypoint_goal_->publish(jackal_goal_msg_);
        return;
    }

    //Store result for later use
    result_ = std::make_shared<rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::WrappedResult>();
    *result_ = result;
  }

  void timer_callback()
  {
    state_ = state_next_;
    auto new_state = state_ != state_last_;

    if (new_state) {
      RCLCPP_INFO_STREAM(get_logger(), "nav_to_pose state changed to " << get_state_name(state_));

      state_last_ = state_;
    }
    switch(state_) {
      case State::IDLE:
      {
        break;
      }
      case State::SEND_GOAL:
      {
        if(act_nav_to_pose_->wait_for_action_server(0s)) {
          //Reset status flags and pointers
          goal_response_received_ = false;
          goal_handle_ = rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr {};
          result_ = nullptr;

          //Construct and send goal
          auto send_goal_options = rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions();
          send_goal_options.goal_response_callback = 
            std::bind(&NavToPose::goal_response_callback, this, std::placeholders::_1);
          send_goal_options.feedback_callback =
            std::bind(&NavToPose::feedback_callback, this, std::placeholders::_1, std::placeholders::_2);
          send_goal_options.result_callback =
            std::bind(&NavToPose::result_callback, this, std::placeholders::_1);
          act_nav_to_pose_->async_send_goal(goal_msg_, send_goal_options);

          state_next_ = State::WAIT_FOR_GOAL_RESPONSE;
        } else {
          RCLCPP_ERROR_STREAM(get_logger(), "Action server not available, aborting.");
          state_next_ = State::IDLE;
        }

        break;
      }
      case State::WAIT_FOR_GOAL_RESPONSE:
      {
        //TODO add timeout
        if (goal_response_received_) {
          if (goal_handle_) {
            RCLCPP_INFO(get_logger(), "Goal accepted by server, waiting for result");
            state_next_ = State::WAIT_FOR_MOVEMENT_COMPLETE;
          } else {
            RCLCPP_ERROR_STREAM(get_logger(), "Goal was rejected by server");
            jackal_goal_msg_.data = "Rejected";
            pub_waypoint_goal_->publish(jackal_goal_msg_);
            state_next_ = State::IDLE;
          }
        }

        break;
      }
      case State::WAIT_FOR_MOVEMENT_COMPLETE:
      {
        if (result_) {
          state_next_ = State::IDLE;
        }
        break;
      }
      default:
        auto msg = "Unhandled state: " + get_state_name(state_);
        RCLCPP_ERROR_STREAM(get_logger(), msg);
        throw std::logic_error(msg);
        break;
    }

  }
};

std::tuple<double, double, double> quaternion_to_rpy(const geometry_msgs::msg::Quaternion & q) {
  //https://answers.ros.org/question/339528/quaternion-to-rpy-ros2/
  tf2::Quaternion q_temp;
  tf2::fromMsg(q, q_temp);
  tf2::Matrix3x3 m(q_temp);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);
  return {roll, pitch, yaw};
}
geometry_msgs::msg::Quaternion rpy_to_quaternion(double roll, double pitch, double yaw) {
  tf2::Quaternion q;
  q.setRPY(roll, pitch, yaw);
  return tf2::toMsg(q);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<NavToPose>());
  rclcpp::shutdown();
  return 0;
}