#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

// generated from DriveToTarget.srv
#include <ball_chaser/DriveToTarget.h>


class ProcessImage {
public:
    ProcessImage() {
        _client = _n.serviceClient<ball_chaser::DriveToTarget>("/ball_chaser/command_robot");
        _sub = _n.subscribe("/camera/rgb/image_raw", 10, &ProcessImage::follow_white_ball, this);

        _n.param("forward_vel", _linear_velocity, 0.2F);
        _n.param("angular_vel", _angular_velocity, 1.5707963267948966F);
    }

private:
    bool velocities_changed(const float linear_velocity, const float angular_velocity) const {
        const auto linear_changed = linear_velocity != _prev_linear;
        const auto angular_changed = angular_velocity != _prev_angular;
        return linear_changed || angular_changed;
    }

    void drive_robot(const float linear_velocity, const float angular_velocity) {
        // Stops us from spamming 0/0 controls when the ball is out of view.
        if (!velocities_changed(linear_velocity, angular_velocity)) {
            return;
        }

        ball_chaser::DriveToTarget srv;
        srv.request.linear_x = linear_velocity;
        srv.request.angular_z = angular_velocity;

        if (_client.call(srv)) {
            _prev_linear = linear_velocity;
            _prev_angular = angular_velocity;
        } else {
            ROS_ERROR("Failed to call service drive_to_target");
        }
    }

    int32_t find_white_ball_column(const sensor_msgs::Image& img) const {
        // See http://docs.ros.org/melodic/api/sensor_msgs/html/msg/Image.html
        assert(sensor_msgs::image_encodings::isColor(img.encoding));
        assert(3 == sensor_msgs::image_encodings::numChannels(img.encoding));
        const std::size_t bytes_per_pixel = 3;

        // The ball is set to be ambient, specular and emissive white, so will
        // be strictly and plainly white in the image. It's a horrible assumption
        // to make that any white pixel resembles the ball, but here goes nothing ...
        const auto stride_bytes = img.step;
        const auto num_pixels = img.height * stride_bytes;
        for (auto i = 0; i < num_pixels; i += bytes_per_pixel) {
            // Channel ordering doesn't matter since we're looking for plain 255.
            const auto r = static_cast<uint8_t>(img.data[i]);
            const auto g = static_cast<uint8_t>(img.data[i + 1]);
            const auto b = static_cast<uint8_t>(img.data[i + 2]);
            if (r == 255 && g == 255 && b == 255) {
                // Return the current column.
                return (i % stride_bytes) / bytes_per_pixel;
            }
        }

        return -1;
    }

    void follow_white_ball(const sensor_msgs::Image& img) {
        const auto column = find_white_ball_column(img);

        // If the ball wasn't found, stop the robot.
        if (column < 0) {
            drive_robot(0.0F, 0.0F);
            return;
        }

        // If the white ball is off the center, steer in that direction.
        const auto center = static_cast<float>(img.width) * 0.5F;
        const auto deviation = (center - static_cast<float>(column)) / static_cast<float>(img.width);
        const auto angular_velocity = deviation * _angular_velocity;

        // If the ball is in the center, move forward.
        const auto left_threshold = img.width / 3;
        const auto right_threshold = img.width * 2 / 3;
        auto linear_velocity = 0.0F;
        if (column >= left_threshold && column <= right_threshold) {
            linear_velocity = _linear_velocity;
        }

        drive_robot(linear_velocity, angular_velocity);
    }

private:
    ros::ServiceClient _client;
    ros::NodeHandle _n;
    ros::Subscriber _sub;

    float _linear_velocity;
    float _angular_velocity;

    float _prev_linear = 0.0F;
    float _prev_angular = 0.0F;
};

int main(int argc, char **argv) {
        ros::init(argc, argv, "process_image");
    ProcessImage ProcessImageNode;
    ros::spin();
    return 0;
}
