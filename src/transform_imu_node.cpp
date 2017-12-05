#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2/convert.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <boost/array.hpp>

#include <memory>
#include <utility>
#include <thread>
#include <functional>

class SubscriberCallbackHandler {
public:
    SubscriberCallbackHandler(const ros::Publisher &publisher) noexcept
        : publisher_ptr_{ new ros::Publisher{ publisher } } { }

    void callback(const sensor_msgs::Imu::ConstPtr &message_ptr) const {
        const auto &message = *message_ptr;

        const auto orientation =
            transform_quaternion(message_to_quaternion(message.orientation));
        const auto orientation_covariance =
            transform_covariance_matrix(
                array_to_matrix(message.orientation_covariance)
            );

        const auto angular_velocity =
            transform_vector(message_to_vector(message.angular_velocity));
        const auto angular_velocity_covariance =
            transform_covariance_matrix(
                array_to_matrix(message.angular_velocity_covariance)
            );

        const auto linear_acceleration =
            transform_vector(message_to_vector(message.linear_acceleration));
        const auto linear_acceleration_covariance =
            transform_covariance_matrix(
                array_to_matrix(message.linear_acceleration_covariance)
            );

        const auto transformed_message =
            make_message(message, orientation, orientation_covariance,
                         angular_velocity, angular_velocity_covariance,
                         linear_acceleration, linear_acceleration_covariance);

        publisher_ptr_->publish(transformed_message);
    }

private:
    inline static sensor_msgs::Imu make_message(
        const sensor_msgs::Imu &transforming,
        const tf2::Quaternion &orientation,
        const tf2::Matrix3x3 &orientation_covariance,
        const tf2::Vector3 &angular_velocity,
        const tf2::Matrix3x3 &angular_velocity_covariance,
        const tf2::Vector3 &linear_acceleration,
        const tf2::Matrix3x3 &linear_acceleration_covariance
    ) {
        auto transformed = sensor_msgs::Imu{ };

        transformed.header = transforming.header;

        transformed.orientation = tf2::toMsg(orientation);
        transformed.orientation_covariance =
            matrix_to_array(orientation_covariance);

        transformed.angular_velocity = tf2::toMsg(angular_velocity);
        transformed.angular_velocity_covariance =
            matrix_to_array(angular_velocity_covariance);

        transformed.linear_acceleration = tf2::toMsg(linear_acceleration);
        transformed.linear_acceleration_covariance =
            matrix_to_array(linear_acceleration_covariance);

        return transformed;
    }

    inline static tf2::Quaternion message_to_quaternion(
        const geometry_msgs::Quaternion &message
    ) {
        auto converted = tf2::Quaternion{ };
        tf2::fromMsg(message, converted);
        return converted;
    }

    inline static tf2::Vector3 message_to_vector(
        const geometry_msgs::Vector3 &message
    ) {
        auto converted = tf2::Vector3{ };
        tf2::fromMsg(message, converted);
        return converted;
    }

    inline static boost::array<double, 9> matrix_to_array(
        const tf2::Matrix3x3 &matrix
    ) noexcept {
        return { { matrix[0][0], matrix[0][1], matrix[0][2],
                   matrix[1][0], matrix[1][1], matrix[1][2],
                   matrix[2][0], matrix[2][1], matrix[2][2] } };
    }

    inline static tf2::Matrix3x3 array_to_matrix(
        const boost::array<double, 9> &array
    ) {
        return { array[0], array[1], array[2],
                 array[3], array[4], array[5],
                 array[6], array[7], array[8] };
    }

    // swaps x and y rows + columns, keeps z the same since covariance is positive
    inline static tf2::Matrix3x3 transform_covariance_matrix(
        const tf2::Matrix3x3 &matrix
    ) {
        return { matrix[1][1], matrix[1][0], matrix[1][2],
                 matrix[0][1], matrix[0][0], matrix[0][2],
                 matrix[2][1], matrix[2][0], matrix[2][2] };
    }

    inline static tf2::Vector3 transform_vector(const tf2::Vector3 &vector) {
        return transform_ * vector;
    }

    inline static tf2::Quaternion transform_quaternion(
        const tf2::Quaternion &quaternion
    ) {
        return transform_ * quaternion;
    }

    // this is a shared_ptr for thread safety
    std::shared_ptr<ros::Publisher> publisher_ptr_;

    // this is a orthonormal transformation R^3 -> R^3
    // it will map:
    // (1, 0, 0) -> (0, 1, 0)
    // (0, 1, 0) -> (1, 0, 0)
    // (0, 0, 1) -> (0, 0, -1)
    // such that it swaps x and y and negates z
    static const tf2::Transform transform_;
};

const tf2::Transform SubscriberCallbackHandler::transform_ =
    tf2::Transform{ tf2::Matrix3x3{ 0,  1,  0,
                                    1,  0,  0,
                                    0,  0, -1 } };

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "transform_imu_node");
    ros::NodeHandle handle;

    auto publisher =
        handle.advertise<sensor_msgs::Imu>("imu/data_transformed", 1000);
    auto handler = SubscriberCallbackHandler{ publisher };
    auto subscriber =
        handle.subscribe<sensor_msgs::Imu>("imu/data", 1000,
                                           &SubscriberCallbackHandler::callback,
                                           &handler);

    auto spinner =
        ros::AsyncSpinner{ std::thread::hardware_concurrency() };
    spinner.start();
    ros::waitForShutdown();
}
