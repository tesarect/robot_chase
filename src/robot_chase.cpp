#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <tf2/exceptions.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <cmath>

class RobotChase : public rclcpp::Node
{
public:
    RobotChase() : Node("initial_approach_robot_chase")
    {
        // Initialize TF2 buffer and listener
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        
        // Create publisher for Rick's velocity commands
        velocity_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>(
            "/rick/cmd_vel", 10);
        
        // // Subscribe to Morty's velocity commands to detect circular motion
        // morty_vel_subscriber_ = this->create_subscription<geometry_msgs::msg::Twist>(
        //     "/morty/cmd_vel", 10,
        //     std::bind(&RobotChase::morty_velocity_callback, this, std::placeholders::_1));
        
        // Create timer for periodic chase updates (10 Hz)
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&RobotChase::chase_callback, this));
        
        // Control gains (proportional constants)
        kp_distance_ = 0.5;  // Linear velocity gain
        kp_yaw_ = 1.0;       // Angular velocity gain
        
        // Safety limits
        max_linear_vel_ = 0.5;   // Maximum linear velocity (m/s)
        max_angular_vel_ = 1.0;  // Maximum angular velocity (rad/s)
        min_distance_ = 0.359;     // Stop distance with a bump - (fractionally lesser that chassis diameter)
        
        RCLCPP_INFO(this->get_logger(), "Robot Chase node started. Rick will follow Morty!");
    }

private:
    void chase_callback()
    {
        geometry_msgs::msg::TransformStamped transform_stamped;
        
        try {
            // Get the latest transform from rick/base_link to morty/base_link
            transform_stamped = tf_buffer_->lookupTransform(
                "rick/base_link",    // Target frame (Rick's perspective)
                "morty/base_link",   // Source frame (Morty's position)
                tf2::TimePointZero); // Latest available transform
        }
        catch (const tf2::TransformException & ex) {
            RCLCPP_WARN(this->get_logger(), "Could not transform rick/base_link to morty/base_link: %s", ex.what());
            return;
        }
        
        // Extract translation (position difference)
        double dx = transform_stamped.transform.translation.x;
        double dy = transform_stamped.transform.translation.y;
        
        // Calculate distance and angle errors
        double error_distance = sqrt(dx * dx + dy * dy);  // Euclidean distance
        double error_yaw = atan2(dy, dx);                 // Angle to target
        
        // Create Twist message for velocity commands
        auto twist_msg = geometry_msgs::msg::Twist();
        
        // Calculate velocities using proportional control
        if (error_distance > min_distance_) {
            double linear_vel;
            double angular_vel;

            if (error_distance > 0.7) {
                // Linear & Angular velocity: when farther apart
                linear_vel = ( kp_distance_ + 0.09 ) * error_distance;
                angular_vel = (kp_yaw_ * 0.01 ) * error_yaw;
                // angular_vel = (kp_yaw_ * kp_yaw_ ) * error_yaw;
            }
            else {
                // Linear & Angular velocity: when nearer
                linear_vel = kp_distance_ * error_distance;
                angular_vel = kp_yaw_ * error_yaw;

                // Apply velocity limits
                linear_vel = std::min(linear_vel, max_linear_vel_);
                linear_vel = std::max(linear_vel, -max_linear_vel_);
                
                angular_vel = std::min(angular_vel, max_angular_vel_);
                angular_vel = std::max(angular_vel, -max_angular_vel_);
            }
            
            // Set velocities
            twist_msg.linear.x = linear_vel;
            twist_msg.angular.z = angular_vel;
            
            RCLCPP_INFO(this->get_logger(), 
                "Distance: %.2f, Angle: %.2f, Lin_vel: %.2f, Ang_vel: %.2f", 
                error_distance, error_yaw, linear_vel, angular_vel);
        }
        else {
            // Stop when close enough to Morty
            twist_msg.linear.x = 0.0;
            twist_msg.angular.z = 0.0;
            RCLCPP_INFO(this->get_logger(), "Close enough to Morty! Stopping.");
        }
        
        // Publish velocity command
        velocity_publisher_->publish(twist_msg);
    }
    
    // Member variables
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr velocity_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    
    // Control parameters
    double kp_distance_;    // Proportional gain for linear velocity
    double kp_yaw_;         // Proportional gain for angular velocity
    double max_linear_vel_; // Maximum linear velocity
    double max_angular_vel_;// Maximum angular velocity
    double min_distance_;   // Minimum distance to maintain
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RobotChase>());
    rclcpp::shutdown();
    return 0;
}