#include <functional>
#include <memory>
#include <thread>

#include "control_msgs/action/follow_joint_trajectory.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"

#include "realtime_tools/realtime_publisher.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"

#include "tf2/exceptions.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2/utils.hpp"

#include "angles/angles.h"

namespace clearpath
{
    constexpr auto DEFAULT_COMMAND_TOPIC = "cmd_vel";

    class MobileBaseActionServer : public rclcpp::Node
    {
    public:
        using JointTrajectory = control_msgs::action::FollowJointTrajectory;
        using GoalHandleJointTrajectory = rclcpp_action::ServerGoalHandle<JointTrajectory>;

        explicit MobileBaseActionServer(const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
            : Node("clearpath_mobile_base", options)
        {
            this->declare_parameter("Kp_linear", 1.0);
            this->declare_parameter("Kp_angular", 1.0);
            this->declare_parameter("num_joints", 9);

            using namespace std::placeholders;

            this->action_server_ = rclcpp_action::create_server<JointTrajectory>(
                this,
                this->get_name() + std::string("/follow_joint_trajectory"),
                std::bind(&MobileBaseActionServer::handle_goal, this, _1, _2),
                std::bind(&MobileBaseActionServer::handle_cancel, this, _1),
                std::bind(&MobileBaseActionServer::handle_accepted, this, _1));

            velocity_command_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>(DEFAULT_COMMAND_TOPIC, rclcpp::SystemDefaultsQoS());
            realtime_velocity_command_publisher_ = std::make_shared<realtime_tools::RealtimePublisher<geometry_msgs::msg::Twist>>(velocity_command_publisher_);

            tf_buffer_ =
                std::make_unique<tf2_ros::Buffer>(this->get_clock());
            tf_listener_ =
                std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        }

    private:
        rclcpp_action::Server<JointTrajectory>::SharedPtr action_server_;

        std::shared_ptr<rclcpp::Publisher<geometry_msgs::msg::Twist>> velocity_command_publisher_ = nullptr;
        std::shared_ptr<realtime_tools::RealtimePublisher<geometry_msgs::msg::Twist>> realtime_velocity_command_publisher_ = nullptr;

        std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
        std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

        void cmd_vel_publish_helper(const double vx, const double wz)
        {
            // Helper to publish a Twist safely
            if (realtime_velocity_command_publisher_->trylock())
            {
                auto &msg = realtime_velocity_command_publisher_->msg_;
                msg.linear.x = vx;
                msg.angular.z = wz;
                realtime_velocity_command_publisher_->unlockAndPublish();
            }
        }

        rclcpp_action::GoalResponse handle_goal(
            const rclcpp_action::GoalUUID &uuid,
            std::shared_ptr<const JointTrajectory::Goal> goal)
        {
            if(!goal->trajectory.points.empty()){
                const auto num_joints = goal->trajectory.points[0].positions.size();
                const auto expected_num_joints = this->get_parameter("num_joints").as_int();

                 RCLCPP_ERROR(this->get_logger(), "There are %ld joint data, expected %ld", num_joints, expected_num_joints);


                 if(num_joints != expected_num_joints){
                    return rclcpp_action::GoalResponse::REJECT;
                 }
            }
            RCLCPP_INFO(this->get_logger(), "Goal request accepted with %ld points!", goal->trajectory.points.size());

            (void)uuid;
            return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        }

        rclcpp_action::CancelResponse handle_cancel(
            const std::shared_ptr<GoalHandleJointTrajectory> goal_handle)
        {
            RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
            (void)goal_handle;

            RCLCPP_INFO(this->get_logger(), "Stopping robot");
            cmd_vel_publish_helper(0.0f, 0.0f);

            return rclcpp_action::CancelResponse::ACCEPT;
        }

        void handle_accepted(const std::shared_ptr<GoalHandleJointTrajectory> goal_handle)
        {
            using namespace std::placeholders;
            // this needs to return quickly to avoid blocking the executor, so spin up a new thread
            std::thread{std::bind(&MobileBaseActionServer::execute, this, _1), goal_handle}.detach();
        }

        void execute(const std::shared_ptr<GoalHandleJointTrajectory> goal_handle)
        {
            // TODO Send the rest of the joints as an action command for the robot arm to 
            RCLCPP_INFO(this->get_logger(), "Executing goal");

            const auto goal = goal_handle->get_goal();

            auto feedback = std::make_shared<JointTrajectory::Feedback>();

            auto result = std::make_shared<JointTrajectory::Result>();

            auto points = goal->trajectory.points;

            rclcpp::Time t_start = this->now();

            geometry_msgs::msg::TransformStamped tf;

            constexpr auto toFrameRel = "odom";
            constexpr auto fromFrameRel = "base_link";

            RCLCPP_INFO(this->get_logger(), "The number of points is %ld", goal->trajectory.points.size());

            std::ostringstream ss;
            ss << "Trajectory has " << points.size() << " points:\n";

            for (size_t i = 0; i < points.size(); ++i) {
            ss << "  Point " << i << ": [";
            for (size_t j = 0; j < points[i].positions.size(); ++j) {
                ss << points[i].positions[j];
                if (j + 1 < points[i].positions.size()) {
                ss << ", ";
                }
            }
            ss << "]\n";
            }

            RCLCPP_INFO(this->get_logger(), "%s", ss.str().c_str());

            for (const auto &point : goal->trajectory.points)
            {
                // If the process is terminated
                if (!rclcpp::ok())
                {
                    break;
                }

                // Check if there is a cancel request
                if (goal_handle->is_canceling())
                {
                    //     result->sequence = sequence;
                    goal_handle->canceled(result);
                    RCLCPP_INFO(this->get_logger(), "Goal canceled-stopping robot");
                    // publish zero
                    cmd_vel_publish_helper(0.0f, 0.0f);
                    return;
                }

                {
                    rclcpp::Time target = t_start + point.time_from_start;
                    auto wait = target - this->now();
                    if (wait > rclcpp::Duration(0, 0))
                    {
                        rclcpp::sleep_for(std::chrono::nanoseconds(wait.nanoseconds()));
                    }
                }

                const double x_d = point.positions[0];
                const double y_d = point.positions[1];
                const double yaw_d = point.positions[2];

                try
                {
                    tf = tf_buffer_->lookupTransform(
                        toFrameRel, fromFrameRel,
                        tf2::TimePointZero);
                }
                catch (const tf2::TransformException &ex)
                {
                    RCLCPP_INFO(
                        this->get_logger(), "Could not transform %s to %s: %s",
                        toFrameRel, fromFrameRel, ex.what());
                    return;
                }

                const double x = tf.transform.translation.x;
                const double y = tf.transform.translation.y;
                const double yaw = tf2::getYaw(tf.transform.rotation);

                double ex = x_d - x;
                double ey = y_d - y;
                double eyaw = angles::shortest_angular_distance(yaw, yaw_d);

                double e_forward = std::cos(yaw) * ex + std::sin(yaw) * ey;

                const double Kp_lin = this->get_parameter("Kp_linear").as_double();
                const double Kp_ang = this->get_parameter("Kp_angular").as_double();

                double v_fb = Kp_lin * e_forward;
                double w_fb = Kp_ang * eyaw;

                double v_ff = 0.0, w_ff = 0.0;
                if (point.velocities.size() >= 3)
                {
                    v_ff = point.velocities[0];
                    w_ff = point.velocities[2];
                }
                double v_cmd = v_fb + v_ff;
                double w_cmd = w_fb + w_ff;

                feedback->desired.positions = {x_d, y_d, yaw_d};
                feedback->actual.positions = {x, y, yaw};
                feedback->error.positions = {ex, ey, eyaw};

                cmd_vel_publish_helper(v_cmd, w_cmd);

                //   Update sequence
                goal_handle->publish_feedback(feedback);
                RCLCPP_INFO(this->get_logger(), "Publish feedback");
            }

            // Check if goal is done
            if (rclcpp::ok())
            {
                RCLCPP_INFO(this->get_logger(), "Stopping robot");
                cmd_vel_publish_helper(0.0, 0.0);

                //   result->sequence = sequence;
                goal_handle->succeed(result);
                RCLCPP_INFO(this->get_logger(), "Goal succeeded");
            }
        }
    }; // class MobileBaseActionServer

} // namespace clearpath

RCLCPP_COMPONENTS_REGISTER_NODE(clearpath::MobileBaseActionServer)