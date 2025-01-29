#include "dqpose.hpp"
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <memory>
#include <signal.h>
#include <thread>

using dof_size_t = uint8_t;
using scalar_t = double;
constexpr dof_size_t DOF = 6;
volatile sig_atomic_t kill_this_node = 0;

void signal_handler([[maybe_unused]] int signum) {
    kill_this_node = 1;
}

class ConsciousNode : public rclcpp::Node
{
private:
    rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr _pub_target_poses;
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr _sub_poses;
    rclcpp::TimerBase::SharedPtr _timer;
    geometry_msgs::msg::Pose _cached_target_pose;
    geometry_msgs::msg::Pose _cached_target_pose_msg;
    bool _initialized = false;

    
    void _callback_poses(const geometry_msgs::msg::Pose::SharedPtr msg)
    {
        using namespace timr::dqpose;
        static size_t iteration = 0;
        static Pose<scalar_t> initial_pose;
        static Translation<scalar_t> initial_translation;
        static Rotation<scalar_t> initial_rotation;
        constexpr scalar_t RADIUS = 0.1;
        constexpr scalar_t RAD_SPEED = 0.003;
        // Store initial pose when first message arrives
        if (!_initialized) {
            initial_pose = Pose<scalar_t>(
                Rotation<scalar_t>(msg->orientation.w, msg->orientation.x, msg->orientation.y, msg->orientation.z),
                Translation<scalar_t>(msg->position.x, msg->position.y, msg->position.z)
            );
            initial_translation = Translation<scalar_t>(0.2663, 0, 0.2985);  // Same as original
            initial_rotation = initial_pose.rotation();
            _initialized = true;
            RCLCPP_INFO(this->get_logger(), "Initialized with initial pose");
        }

        // Calculate circular motion
        Translation<scalar_t> target_translation = initial_translation + 
            Translation<scalar_t>(0, RADIUS * cos(RAD_SPEED * iteration), 
                    RADIUS * sin(RAD_SPEED * iteration));
        
        // Convert to ROS message
        _cached_target_pose_msg.position.x = target_translation.x();
        _cached_target_pose_msg.position.y = target_translation.y();
        _cached_target_pose_msg.position.z = target_translation.z();
        _cached_target_pose_msg.orientation.w = initial_rotation.w();
        _cached_target_pose_msg.orientation.x = initial_rotation.x();
        _cached_target_pose_msg.orientation.y = initial_rotation.y();
        _cached_target_pose_msg.orientation.z = initial_rotation.z();

        ++iteration;
    }

    void _callback_timer()
    {
        if (!_initialized) {
            return;
        }
        _pub_target_poses->publish(_cached_target_pose_msg);
    }

public:
    ConsciousNode() : Node("conscious_node")
    {
    }

    bool initialize()
    {
        RCLCPP_INFO(this->get_logger(), "Initializing conscious node...");
        // Initialize subscriber for poses
        _sub_poses = this->create_subscription<geometry_msgs::msg::Pose>(
            "poses", 10,
            std::bind(&ConsciousNode::_callback_poses, this, std::placeholders::_1));
        while (!_initialized && !kill_this_node) {
            rclcpp::spin_some(shared_from_this());
            std::this_thread::sleep_for(std::chrono::milliseconds(1000));
            if (kill_this_node) {
                RCLCPP_INFO(this->get_logger(), "Received interrupt, shutting down...");
                return false;
            }
        }      

        // Initialize publisher for target poses
        _pub_target_poses = this->create_publisher<geometry_msgs::msg::Pose>(
            "target_poses", 10);

            
        _timer = this->create_wall_timer(
            std::chrono::milliseconds(4),
            std::bind(&ConsciousNode::_callback_timer, this));
        
        RCLCPP_INFO(this->get_logger(), "Conscious node initialized");
        return true;
    }

};


int main(int argc, char** argv)
{
    signal(SIGINT, signal_handler);
    
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ConsciousNode>();
    if (!node->initialize()) {
        RCLCPP_INFO(node->get_logger(), "Failed to initialize conscious node.");
        rclcpp::shutdown();
        return 1;
    }
    while (rclcpp::ok() && !kill_this_node) {
        rclcpp::spin_some(node);
    }
    
    RCLCPP_INFO(node->get_logger(), "Shutting down conscious node...");
    rclcpp::shutdown();
    return 0;
}
