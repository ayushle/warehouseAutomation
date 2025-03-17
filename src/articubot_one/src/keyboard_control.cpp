#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include <termios.h>
#include <unistd.h>
#include <iostream>

class KeyboardControl : public rclcpp::Node
{
public:
    KeyboardControl() : Node("keyboard_control"), arm_position_(0.0)
    {
        arm_pos_pub_ = this->create_publisher<std_msgs::msg::Float64>("/arm_pos", 10);
        RCLCPP_INFO(this->get_logger(), "Keyboard control started. Use UP/DOWN arrow keys to control.");
        run();
    }

private:
    double arm_position_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr arm_pos_pub_;

    int getKey()
    {
        struct termios oldt, newt;
        int ch;
        tcgetattr(STDIN_FILENO, &oldt);
        newt = oldt;
        newt.c_lflag &= ~(ICANON | ECHO);
        tcsetattr(STDIN_FILENO, TCSANOW, &newt);
        ch = getchar();
        tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
        return ch;
    }

    void run()
    {
        while (rclcpp::ok())
        {
            int key = getKey();
            if (key == 27) // Arrow keys start with escape sequence
            {
                if (getKey() == 91) // Detect '['
                {
                    key = getKey(); // Get actual arrow key

                    if (key == 65) // UP arrow
                    {
                        arm_position_ += 0.01;
                        publishPosition();
                    }
                    else if (key == 66) // DOWN arrow
                    {
                        arm_position_ -= 0.01;
                        publishPosition();
                    }
                }
            }
        }
    }

    void publishPosition()
    {
        auto msg = std_msgs::msg::Float64();
        msg.data = arm_position_;
        arm_pos_pub_->publish(msg);
        RCLCPP_INFO(this->get_logger(), "Arm Position: %.2f", arm_position_);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    std::make_shared<KeyboardControl>();
    rclcpp::shutdown();
    return 0;
}
