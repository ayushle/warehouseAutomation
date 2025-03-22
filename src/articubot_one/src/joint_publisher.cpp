// ros2 topic pub /joint_trajectory_controller/joint_trajectory trajectory_msgs/JointTrajectory '
// joint_names: ["linear_actuator"]
// points:
// - positions: [0.0]
//   time_from_start: {sec: 2, nanosec: 0}'





#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <cmath> 

class position_publisher : public rclcpp::Node

{
public:
    position_publisher() : Node("arm_position_publisher")
    {
        // Subscriber to /arm_pos topic
        arm_pos_sub_ = this->create_subscription<std_msgs::msg::Float64>(
            "/arm_pos", 10, std::bind(&position_publisher::arm_pos_callback, this, std::placeholders::_1));

        goal_pos_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
            "/goal_pos", 10, std::bind(&position_publisher::goal_pos_callback, this, std::placeholders::_1));
        
        // Publisher to /joint_states topic
        joint_traj_pub_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>("/joint_trajectory_controller/joint_trajectory", 10);
        goal_pos_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/goal_pose", 10);
        
        // joint_states_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);
        // cmd_publisher_=this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    }

private:
    void arm_pos_callback(const std_msgs::msg::Float64::SharedPtr msg)
    {
        // auto joint_msg = sensor_msgs::msg::JointState();
        // joint_msg.header.stamp = this->now();
        // joint_msg.name = {"left_wheel_joint", "right_wheel_joint", "linear_actuator"};
        // joint_msg.position = {msg->data, msg->data, msg->data};
        
        // joint_states_pub_->publish(joint_msg);

       

        auto joint_msg = trajectory_msgs::msg::JointTrajectory();
        joint_msg.joint_names = {"linear_actuator"};  

        
        trajectory_msgs::msg::JointTrajectoryPoint point;
        point.positions = {msg->data};  
        point.time_from_start = rclcpp::Duration::from_seconds(0.01);  

        
        joint_msg.points.push_back(point);

        
        joint_traj_pub_->publish(joint_msg);

        // auto cmd_vel_msg = geometry_msgs::msg::Twist();
        // cmd_vel_msg.linear.x = msg->data; // Mapping arm position value to linear velocity
        // cmd_publisher_->publish(cmd_vel_msg);
    }
    void goal_pos_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
    {
       

        if (msg->data.size() < 3) {
            RCLCPP_WARN(this->get_logger(), "Received goal position has less than 3 values!");
            return;
        }

        auto goal_msg = geometry_msgs::msg::PoseStamped();
        goal_msg.header.stamp = this->now();
        goal_msg.header.frame_id = "map"; 

        goal_msg.pose.position.x = msg->data[0];
        goal_msg.pose.position.y = msg->data[1];
        goal_msg.pose.position.z=0.0;

        float orient = msg->data[2];
        orient=(orient/180)*3.147;
        goal_msg.pose.orientation.x = 0.0;
        goal_msg.pose.orientation.y = 0.0;
        goal_msg.pose.orientation.z = sin(orient/2);
        goal_msg.pose.orientation.w = cos(orient/2);

        goal_pos_pub_->publish(goal_msg);

        
    }

    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr arm_pos_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr goal_pos_sub_;
    
    // rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_states_pub_;
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr joint_traj_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pos_pub_;


    // rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_publisher_;

};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<position_publisher>());
    rclcpp::shutdown();
    return 0;
}
