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
#include "visualization_msgs/msg/marker.hpp"
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
X(WAIT_FOR_MOVEMENT_COMPLETE, "WAIT_FOR_MOVEMENT_COMPLETE") \
X(CANCEL_CURRENT_GOAL, "CANCEL_CURRENT_GOAL")

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
    pose_frame_ = get_parameter("pose_frame").get_parameter_value().get<std::string>();
    goal_msg_.pose.header.frame_id = pose_frame_;
    auto repub = rcl_interfaces::msg::ParameterDescriptor{};
    repub.description = "The frame in which poses are sent.";
    declare_parameter("republish_same_goal", true, repub);
    repub_same_goal_ = get_parameter("republish_same_goal").as_bool();

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
    pub_goal_marker_ = create_publisher<visualization_msgs::msg::Marker>("/client_goal", 10);

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
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_goal_marker_;

  double rate_ = 100.0; //Hz
  double interval_ = 1.0 / rate_; //seconds
  std::string pose_frame_;
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
  std::shared_future<std::shared_ptr<action_msgs::srv::CancelGoal_Response>> cancel_result_future_;
  bool cancel_and_replan_ = false;
  bool repub_same_goal_;
  
  bool are_goals_equal(const geometry_msgs::msg::Pose& pose1, const geometry_msgs::msg::Pose& pose2) {
  const double epsilon = 1e-5;  // Define tolerance level
  return std::fabs(pose1.position.x - pose2.position.x) < epsilon &&
         std::fabs(pose1.position.y - pose2.position.y) < epsilon &&
         std::fabs(pose1.orientation.z - pose2.orientation.z) < epsilon &&  // Checking only z and w
         std::fabs(pose1.orientation.w - pose2.orientation.w) < epsilon;
  }

  void publish_goal_marker(const double & x, const double & y, const geometry_msgs::msg::Quaternion & orientation) {
    visualization_msgs::msg::Marker goal_arrow;
    goal_arrow.header.frame_id = pose_frame_;
    goal_arrow.header.stamp = get_clock()->now();
    goal_arrow.id = 0;
    goal_arrow.type = visualization_msgs::msg::Marker::ARROW;
    goal_arrow.action = visualization_msgs::msg::Marker::ADD;
    goal_arrow.pose.position.x = x;
    goal_arrow.pose.position.y = y;
    goal_arrow.pose.position.z = 0.01;
    goal_arrow.pose.orientation = orientation;
    goal_arrow.scale.x = 0.5;
    goal_arrow.scale.y = 0.1;
    goal_arrow.scale.z = 0.1;
    goal_arrow.color.r = 1.0f;
    goal_arrow.color.g = 0.5f;
    goal_arrow.color.b = 0.0f;
    goal_arrow.color.a = 1.0;

    pub_goal_marker_->publish(goal_arrow);
  }

  void srv_nav_to_pose_callback(
    const std::shared_ptr<nav_client_cpp::srv::NavToPose::Request> request,
    std::shared_ptr<nav_client_cpp::srv::NavToPose::Response>
  ) {

    // Skip if goal is the same
    geometry_msgs::msg::Pose new_pose;
    new_pose.position.x = request->x;
    new_pose.position.y = request->y;
    new_pose.orientation = rpy_to_quaternion(0.0, 0.0, request->theta);
    
    if (goal_handle_ && !repub_same_goal_ && are_goals_equal(goal_msg_.pose.pose, new_pose)) {
      RCLCPP_INFO(this->get_logger(), "Received goal is the same as the current one. Continuing...");
      return;
    }

    // Check if there is an active goal and cancel it before sending a new one
    if (goal_handle_ && (
      goal_handle_->get_status() == rclcpp_action::GoalStatus::STATUS_ACCEPTED ||
      goal_handle_->get_status() == rclcpp_action::GoalStatus::STATUS_EXECUTING)) {

      //Store requested pose
      goal_msg_.pose.pose.position.x = request->x;
      goal_msg_.pose.pose.position.y = request->y;
      goal_msg_.pose.pose.orientation = rpy_to_quaternion(0.0, 0.0, request->theta);
      RCLCPP_INFO(this->get_logger(), "Quaternion: x:%f, y:%f, z:%f, w:%f", 
                  goal_msg_.pose.pose.orientation.x, 
                  goal_msg_.pose.pose.orientation.y, 
                  goal_msg_.pose.pose.orientation.z, 
                  goal_msg_.pose.pose.orientation.w);

      //Initiate action call
      cancel_and_replan_ = true;
      state_next_ = State::CANCEL_CURRENT_GOAL;
    } else {
      //Store requested pose
      goal_msg_.pose.pose.position.x = request->x;
      goal_msg_.pose.pose.position.y = request->y;
      goal_msg_.pose.pose.orientation = rpy_to_quaternion(0.0, 0.0, request->theta);
      RCLCPP_INFO(this->get_logger(), "Quaternion: x:%f, y:%f, z:%f, w:%f", 
                  goal_msg_.pose.pose.orientation.x, 
                  goal_msg_.pose.pose.orientation.y, 
                  goal_msg_.pose.pose.orientation.z, 
                  goal_msg_.pose.pose.orientation.w);

      //Initiate action call
      state_next_ = State::SEND_GOAL;
    }
  }

  void cancel_nav_callback(
    const std::shared_ptr<std_srvs::srv::Empty::Request>,
    std::shared_ptr<std_srvs::srv::Empty::Response>
  ) {
    RCLCPP_INFO_STREAM(get_logger(), "Cancelling navigation.");
    act_nav_to_pose_->async_cancel_all_goals();
    cancel_and_replan_ = false;
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
      publish_goal_marker(goal_msg_.pose.pose.position.x,
                          goal_msg_.pose.pose.position.y,
                          goal_msg_.pose.pose.orientation);
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
        if (cancel_and_replan_) {
          RCLCPP_WARN(this->get_logger(), "Previous goal canceled. Planing next...");
          jackal_goal_msg_.data = "Canceled"; // Maybe change this to replanned???
          state_next_ = State::SEND_GOAL;
        } else {
          RCLCPP_WARN(this->get_logger(), "Goal was canceled");
          jackal_goal_msg_.data = "Canceled";
        }
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
      case State::CANCEL_CURRENT_GOAL:
      {
        if (!goal_handle_) {
          RCLCPP_ERROR(this->get_logger(), "No goal handle to cancel.");
          state_next_ = State::IDLE;
          return;
        }
        if (new_state) {
          RCLCPP_INFO(this->get_logger(), "Cancelling the current goal.");
        }
        // Initiate a request to cancel all active goals
        cancel_result_future_ = act_nav_to_pose_->async_cancel_all_goals();

        // Check the result of the cancel request
        if (cancel_result_future_.wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
          auto result = cancel_result_future_.get();
          if (result->return_code == action_msgs::srv::CancelGoal::Response::ERROR_NONE) {
            RCLCPP_INFO(this->get_logger(), "All goals successfully canceled.");
            goal_handle_.reset(); // Clear the goal handle as all goals are canceled
            state_next_ = State::SEND_GOAL; // Or set to IDLE if you don’t want to resend immediately
          } else {
            RCLCPP_WARN(this->get_logger(), "Failed to cancel all goals; error code: %d", result->return_code);
            state_next_ = State::IDLE;
          }
        }
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

          cancel_and_replan_ = false;
          state_next_ = State::WAIT_FOR_GOAL_RESPONSE;
        } else {
          RCLCPP_ERROR_STREAM(get_logger(), "Action server not available, aborting.");
          cancel_and_replan_ = false;
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