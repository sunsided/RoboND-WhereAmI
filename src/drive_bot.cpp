#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

// generated from DriveToTarget.srv
#include "ball_chaser/DriveToTarget.h"


class DriveBot {
public:
    DriveBot() {
        const auto queue_size = 10;
        _pub = _n.advertise<geometry_msgs::Twist>("/cmd_vel", queue_size);
        _server = _n.advertiseService("/ball_chaser/command_robot", &DriveBot::handle_drive_request, this);
        ROS_INFO("Ready to send wheel commands");
    }

private:
    bool handle_drive_request(ball_chaser::DriveToTarget::Request &req, ball_chaser::DriveToTarget::Response &res) {
        const auto linearX = static_cast<float>(req.linear_x);
        const auto angularZ = static_cast<float>(req.angular_z);

        ROS_INFO("DriveToTarget request received - linear_x:%1.2f, angular_z:%1.2f", linearX, angularZ);

        // Generate the Twist message and publish it.
        geometry_msgs::Twist motor_command;
        motor_command.linear.x = linearX;
        motor_command.angular.z = angularZ;
        _pub.publish(motor_command);

        res.msg_feedback = "Velocities set - linear_x: " + std::to_string(linearX) +
                           ", angular_z: " + std::to_string(angularZ);
        ROS_INFO_STREAM(res.msg_feedback);

        return true;
    }

private:
    ros::NodeHandle _n;
    ros::Publisher _pub;
    ros::ServiceServer _server;
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "drive_bot");
    DriveBot DriveBotNode;
    ros::spin();
    return 0;
}
