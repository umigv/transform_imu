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
#include <tuple>

inline tf2::Quaternion quaternion_from_message(
    const geometry_msgs::Quaternion &message
) {
    auto converted = tf2::Quaternion{ };
    tf2::fromMsg(message, converted);
    return converted;
}

inline tf2::Vector3 vector_from_message(
    const geometry_msgs::Vector3 &message
) {
    auto converted = tf2::Vector3{ };
    tf2::fromMsg(message, converted);
    return converted;
}

inline boost::array<double, 9> array_from_matrix(
    const tf2::Matrix3x3 &matrix
) {
    return { { matrix[0][0], matrix[0][1], matrix[0][2],
               matrix[1][0], matrix[1][1], matrix[1][2],
               matrix[2][0], matrix[2][1], matrix[2][2] } };
}

inline tf2::Matrix3x3 matrix_from_array(
    const boost::array<double, 9> &array
) {
    return { array[0], array[1], array[2],
             array[3], array[4], array[5],
             array[6], array[7], array[8] };
}

inline tf2::Matrix3x3 rotate_covariance_matrix(
    const tf2::Matrix3x3 &matrix
) {
    return { matrix[1][1], matrix[1][0], matrix[1][2],
             matrix[0][1], matrix[0][0], matrix[0][2],
             matrix[2][1], matrix[2][0], matrix[2][2] };
}

inline tf2::Quaternion rotation_matrix_to_quaternion(
    const tf2::Matrix3x3 &matrix
) {
    auto to_return = tf2::Quaternion{ };
    matrix.getRotation(to_return);
    return to_return;
}

class SubscriberCallbackHandler {
public:
    SubscriberCallbackHandler(
        const std::shared_ptr<ros::Publisher> &publisher
    ) noexcept : publisher_{ publisher } { }

    void callback(const sensor_msgs::Imu::ConstPtr &data_ptr) const {
        // this is a linear transformation R^3 -> R^3 that will map linear
        // coordinates to their proper places
        static const auto rotation_matrix = tf2::Matrix3x3{ 0, 1, 0,
                                                            1, 0, 0,
                                                            0, 0, -1 };

        const auto orientation =
            quaternion_from_message(data_ptr->orientation);
        const auto orientation_covariance =
            matrix_from_array(data_ptr->orientation_covariance);

        const auto angular_velocity =
            vector_from_message(data_ptr->angular_velocity);
        const auto angular_velocity_covariance =
            matrix_from_array(data_ptr->angular_velocity_covariance);

        const auto linear_acceleration =
            vector_from_message(data_ptr->linear_acceleration);
        const auto linear_acceleration_covariance =
            matrix_from_array(data_ptr->linear_acceleration_covariance);

        // transformed variables
        const auto orientation_transformed =
            static_cast<tf2::Quaternion>(
                orientation
                * rotation_matrix_to_quaternion(rotation_matrix)
            );
        const auto orientation_covariance_transformed =
            rotate_covariance_matrix(orientation_covariance);

        const auto angular_velocity_transformed =
            static_cast<tf2::Vector3>(rotation_matrix * angular_velocity);
        const auto angular_velocity_covariance_transformed =
            rotate_covariance_matrix(angular_velocity_covariance);

        const auto linear_acceleration_transformed =
            static_cast<tf2::Vector3>(
                rotation_matrix * linear_acceleration
            );
        const auto linear_acceleration_covariance_transformed =
            rotate_covariance_matrix(linear_acceleration_covariance);

        auto transformed_message = sensor_msgs::Imu{ };

        transformed_message.header = data_ptr->header;

        transformed_message.orientation =
            tf2::toMsg(orientation_transformed);
        transformed_message.orientation_covariance =
            array_from_matrix(orientation_covariance_transformed);

        transformed_message.angular_velocity =
            tf2::toMsg(angular_velocity_transformed);
        transformed_message.angular_velocity_covariance =
            array_from_matrix(angular_velocity_covariance_transformed);

        transformed_message.linear_acceleration =
            tf2::toMsg(linear_acceleration_transformed);
        transformed_message.linear_acceleration_covariance =
            array_from_matrix(linear_acceleration_covariance_transformed);

        publisher_->publish(transformed_message);
    }

private:
    // this is a shared_ptr for thread safety
    std::shared_ptr<ros::Publisher> publisher_;
};

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "transform_imu_node");
    ros::NodeHandle handle;

    auto publisher =
        handle.advertise<sensor_msgs::Imu>("imu/data_transformed", 1000);
    auto publisher_ptr =
        std::shared_ptr<ros::Publisher>{
            new ros::Publisher{ publisher }
        };
    auto handler = SubscriberCallbackHandler{ publisher_ptr };
    auto subscriber = handle.subscribe<sensor_msgs::Imu>(
        "imu/data",
        1000,
        &SubscriberCallbackHandler::callback,
        &handler
    );

    auto spinner =
        ros::AsyncSpinner{ std::thread::hardware_concurrency() };
    spinner.start();
    ros::waitForShutdown();
}
