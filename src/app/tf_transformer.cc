#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>
#include "livox_ros_driver2/msg/custom_msg.hpp"

class SensorTFTransform : public rclcpp::Node{
public:
    SensorTFTransform() : Node("Sensor_tf_transformer"){
        // TF2缓冲区
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        lidar_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/transformed/lidar", 10);
        livox_pub_ = this->create_publisher<livox_ros_driver2::msg::CustomMsg>("/transformed/livox", 10);
        imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("/transformed/imu", 10);

        lidar_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/livox/lidar/pointcloud2", 10, std::bind(&SensorTFTransform::lidar_callback, this, std::placeholders::_1)
        );
        livox_sub_ = this->create_subscription<livox_ros_driver2::msg::CustomMsg>(
            "/livox/lidar", 10, std::bind(&SensorTFTransform::livox_callback, this, std::placeholders::_1)
        );
        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/livox/imu", 10, std::bind(&SensorTFTransform::imu_callback, this, std::placeholders::_1)
        );

        RCLCPP_INFO(this->get_logger(), "SensorTFTransformer Node has been started.");

    }

private:

    void lidar_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg){
        try{
            // 查找lidar_link到base_link的tf
            geometry_msgs::msg::TransformStamped transform = tf_buffer_->lookupTransform(
                "base_link", "lidar_link", msg->header.stamp, rclcpp::Duration::from_seconds(0.1));

            // 转换点云数据
            sensor_msgs::msg::PointCloud2 transform_msg;
            tf2::doTransform(*msg, transform_msg, transform);
            transform_msg.header.frame_id = "base_link";

            // 发布转换后的点云数据
            lidar_pub_->publish(transform_msg);
        }
        catch(tf2::TransformException &ex){
            RCLCPP_WARN(this->get_logger(), "Could not transform point cloud: %s", ex.what());
        }
    }

    // TODO:当数据为livox_ros_driver2::msg::CustomMsg时的回调函数
    void livox_callback(const livox_ros_driver2::msg::CustomMsg::SharedPtr msg){
        geometry_msgs::msg::TransformStamped transform;
        try{
            transform = tf_buffer_->lookupTransform("base_link", "lidar_link", msg->header.stamp, rclcpp::Duration::from_seconds(0.1));
        }
        catch(tf2::TransformException &ex){
            RCLCPP_WARN(this->get_logger(), "Could not transform livox cloud: %s", ex.what());
        }
        
        tf2::Transform tf;
        tf2::fromMsg(transform.transform, tf);

        livox_ros_driver2::msg::CustomMsg msg_out;
        msg_out = *msg;
        msg_out.header.frame_id = "base_link";

        tf2::Vector3 p_livox;
        tf2::Vector3 p_base;
        for(auto& pt : msg_out.points){
            p_livox.setValue(pt.x, pt.y, pt.z);
            p_base = tf * p_livox;

            pt.x = p_base.x();
            pt.y = p_base.y();
            pt.z = p_base.z();
        }
        livox_pub_->publish(msg_out);
        
    }

    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg){
        try{
            // 查找lidar_link到base_link的tf
            geometry_msgs::msg::TransformStamped transform = tf_buffer_->lookupTransform(
                "base_link", "imu_link", msg->header.stamp, rclcpp::Duration::from_seconds(0.1));

            sensor_msgs::msg::Imu transform_msg = *msg;
            // 首先处理的了IMU数据中的 orientation 部分，但是从IMU拿到的数据中，通常只有角速度和线加速度，所以这个部分的转换可能没有实际意义
            // tf2::Quaternion q_in;
            tf2::Quaternion q_trans;
            // tf2::Quaternion q_out;

            // tf2::fromMsg(msg->orientation, q_in);
            tf2::fromMsg(transform.transform.rotation, q_trans);
            // q_out = q_trans * q_in;
            // q_out.normalize();
            // tf2::toMsg(q_out, transform_msg.orientation);

            // 处理角速度和线加速度
            tf2::Vector3 ang_in(msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z);
            tf2::Vector3 acc_in(msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z);

            tf2::Vector3 ang_out = tf2::quatRotate(q_trans, ang_in); // 这里执行了角速度向量的旋转
            tf2::Vector3 acc_out = tf2::quatRotate(q_trans, acc_in); // 这里执行了线加速度向量的旋转

            transform_msg.angular_velocity.x = ang_out.x();
            transform_msg.angular_velocity.y = ang_out.y();
            transform_msg.angular_velocity.z = ang_out.z();

            transform_msg.linear_acceleration.x = acc_out.x();
            transform_msg.linear_acceleration.y = acc_out.y();
            transform_msg.linear_acceleration.z = acc_out.z();

            transform_msg.header.frame_id = "base_link";

            imu_pub_->publish(transform_msg);
        }
        catch(tf2::TransformException &ex){
            RCLCPP_WARN(this->get_logger(), "Could not transform IMU: %s", ex.what());
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_sub_;
    rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>::SharedPtr livox_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_pub_;
    rclcpp::Publisher<livox_ros_driver2::msg::CustomMsg>::SharedPtr livox_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;

    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

int main(int argc, char** argv){
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SensorTFTransform>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}