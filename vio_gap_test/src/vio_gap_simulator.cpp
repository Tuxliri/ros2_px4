#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <chrono>

class VIOGapSimulator : public rclcpp::Node
{
public:
    VIOGapSimulator() : Node("vio_gap_simulator"),
                        gap_duration_(rclcpp::Duration::from_nanoseconds(0)),
                        gap_start_delay_(rclcpp::Duration::from_nanoseconds(0)),
                        gap_active_(false)
    {
        // Parameters
        this->declare_parameter("input_topic", "odometry_in");
        this->declare_parameter("output_topic", "odometry_out");
        this->declare_parameter("gap_milliseconds", 5000.0);
        this->declare_parameter("gap_start_delay", 10000.0);

        input_topic_ = this->get_parameter("input_topic").as_string();
        output_topic_ = this->get_parameter("output_topic").as_string();
        auto gap_ms = this->get_parameter("gap_milliseconds").as_double();
        auto delay_ms = this->get_parameter("gap_start_delay").as_double();
        
        // Now properly set the durations
        gap_duration_ = rclcpp::Duration::from_nanoseconds(static_cast<int64_t>(gap_ms * 1e6));
        gap_start_delay_ = rclcpp::Duration::from_nanoseconds(static_cast<int64_t>(delay_ms * 1e6));

        // Set up recurring gap timer
        auto gap_start_period = std::chrono::nanoseconds(gap_start_delay_.nanoseconds());
        gap_start_timer_ = this->create_wall_timer(
            gap_start_period,
            std::bind(&VIOGapSimulator::start_gap, this));

        // Subscribe/publish
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            input_topic_, 10,
            std::bind(&VIOGapSimulator::odometry_callback, this, std::placeholders::_1));
        odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(output_topic_, 10);

        RCLCPP_INFO(this->get_logger(), "VIO Gap Simulator started");
        RCLCPP_INFO(this->get_logger(), "Input odometry topic: %s", input_topic_.c_str());
        RCLCPP_INFO(this->get_logger(), "Output odometry topic: %s", output_topic_.c_str());
        RCLCPP_INFO(this->get_logger(),
                    "Gap configuration: %.1f seconds gap every %.1f seconds",
                    gap_duration_.seconds(), gap_start_delay_.seconds());
    }

private:
    void odometry_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        auto current_time = this->get_clock()->now();

        // Only publish odometry if not in a gap
        if (!gap_active_) {
            auto output_msg = *msg;
            output_msg.header.stamp = current_time;
            odom_pub_->publish(output_msg);

            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                                 "Publishing odometry: pos=[%.3f, %.3f, %.3f]",
                                 msg->pose.pose.position.x,
                                 msg->pose.pose.position.y,
                                 msg->pose.pose.position.z);
        } else {
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                 "VIO gap active – not publishing odometry data");
        }
    }

    void start_gap()
    {
        // Prevent overlapping gaps
        if (gap_active_) {
            return;
        }

        gap_active_ = true;
        RCLCPP_WARN(this->get_logger(),
                    "Starting VIO gap simulation – no odometry will be published for %.1f seconds",
                    gap_duration_.seconds());

        // Schedule a one-shot timer to end the gap
        auto gap_end_period = std::chrono::nanoseconds(gap_duration_.nanoseconds());
        gap_end_timer_ = this->create_wall_timer(
            gap_end_period,
            [this]() {
                gap_active_ = false;
                RCLCPP_WARN(this->get_logger(),
                            "VIO gap simulation ended – resuming odometry publication");
                gap_end_timer_->cancel();  // ensure it only fires once
            });
    }

    // ROS2 components
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;

    // Gap simulation variables
    rclcpp::Duration gap_duration_;
    rclcpp::Duration gap_start_delay_;
    bool gap_active_;

    // Timers for periodic gap simulation
    rclcpp::TimerBase::SharedPtr gap_start_timer_;
    rclcpp::TimerBase::SharedPtr gap_end_timer_;

    // Configuration
    std::string input_topic_;
    std::string output_topic_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<VIOGapSimulator>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}