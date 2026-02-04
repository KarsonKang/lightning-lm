#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

class StaticTFBroadcaster : public rclcpp::Node {
public: 
    StaticTFBroadcaster() : Node("static_tf_broadcaster") {
        // 初始化静态TF发布器
        static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

        // 初始化参数
        init_params();

        // 发布静态TF
        publishStaticTF(body_frame_id_, imu_frame_id_, imu2body_pos_, imu2body_ori_);
        publishStaticTF(body_frame_id_, lidar_frame_id_, lidar2body_pos_, lidar2body_ori_);
        publishStaticTF(body_frame_id_, realsense_frame_id_, realsense2body_pos_, realsense2body_ori_);
    }
        
private:
    struct Vec3 {double x, y, z;};
    struct Quat {double x, y, z, w;};

    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_broadcaster_;

    // TF 参数
    std::string body_frame_id_;
    std::string lidar_frame_id_, realsense_frame_id_, imu_frame_id_;

    Vec3 imu2body_pos_, lidar2body_pos_, realsense2body_pos_;
    Quat imu2body_ori_, lidar2body_ori_, realsense2body_ori_;

    void init_params(){
        // 声明参数
        this->declare_parameter<std::string>("robot_body_frame_id", "base_link");

        this->declare_parameter<std::string>("imu_frame_id", "imu_link");
        this->declare_parameter<double>("imu2body_pos_x", 0.0);
        this->declare_parameter<double>("imu2body_pos_y", 0.0);
        this->declare_parameter<double>("imu2body_pos_z", 1.7);
        this->declare_parameter<double>("imu2body_ori_x", 1.0);
        this->declare_parameter<double>("imu2body_ori_y", 0.0);
        this->declare_parameter<double>("imu2body_ori_z", 0.0);
        this->declare_parameter<double>("imu2body_ori_w", 0.0);

        this->declare_parameter<std::string>("lidar_frame_id", "lidar_link");
        this->declare_parameter<double>("lidar2body_pos_x", 0.0);
        this->declare_parameter<double>("lidar2body_pos_y", 0.0);
        this->declare_parameter<double>("lidar2body_pos_z", 1.7);
        this->declare_parameter<double>("lidar2body_ori_x", 1.0);
        this->declare_parameter<double>("lidar2body_ori_y", 0.0);
        this->declare_parameter<double>("lidar2body_ori_z", 0.0);
        this->declare_parameter<double>("lidar2body_ori_w", 0.0);

        this->declare_parameter<std::string>("realsense_frame_id", "camera_link");
        this->declare_parameter<double>("realsense2body_pos_x", 0.0);
        this->declare_parameter<double>("realsense2body_pos_y", 0.0);
        this->declare_parameter<double>("realsense2body_pos_z", 0.0);
        this->declare_parameter<double>("realsense2body_ori_x", 0.0);
        this->declare_parameter<double>("realsense2body_ori_y", 0.0);
        this->declare_parameter<double>("realsense2body_ori_z", 0.0);
        this->declare_parameter<double>("realsense2body_ori_w", 1.0);

        // 读取参数
        body_frame_id_ = this->get_parameter("robot_body_frame_id").as_string();

        imu_frame_id_ = this->get_parameter("imu_frame_id").as_string();
        imu2body_pos_ = {
                        get_parameter("imu2body_pos_x").as_double(),
                        get_parameter("imu2body_pos_y").as_double(),
                        get_parameter("imu2body_pos_z").as_double()
                    };
        imu2body_ori_ = {
                        get_parameter("imu2body_ori_x").as_double(),
                        get_parameter("imu2body_ori_y").as_double(),
                        get_parameter("imu2body_ori_z").as_double(),
                        get_parameter("imu2body_ori_w").as_double()
                    };

        lidar_frame_id_ = this->get_parameter("lidar_frame_id").as_string();
        lidar2body_pos_ = {
                        get_parameter("lidar2body_pos_x").as_double(),
                        get_parameter("lidar2body_pos_y").as_double(),
                        get_parameter("lidar2body_pos_z").as_double()
                    };
        lidar2body_ori_ = {
                        get_parameter("lidar2body_ori_x").as_double(),
                        get_parameter("lidar2body_ori_y").as_double(),
                        get_parameter("lidar2body_ori_z").as_double(),
                        get_parameter("lidar2body_ori_w").as_double()
                    };

        realsense_frame_id_ = this->get_parameter("realsense_frame_id").as_string();
        realsense2body_pos_ = {
                            get_parameter("realsense2body_pos_x").as_double(),
                            get_parameter("realsense2body_pos_y").as_double(),
                            get_parameter("realsense2body_pos_z").as_double()
                        };
        realsense2body_ori_ = {
                            get_parameter("realsense2body_ori_x").as_double(),
                            get_parameter("realsense2body_ori_y").as_double(),
                            get_parameter("realsense2body_ori_z").as_double(),
                            get_parameter("realsense2body_ori_w").as_double()
                        };
    }

    void publishStaticTF(const std::string& parent_frame, const std::string& child_frame, const Vec3& pos, const Quat& ori) {
        geometry_msgs::msg::TransformStamped tf;

        tf.header.stamp = this->now();
        tf.header.frame_id = parent_frame;
        tf.child_frame_id = child_frame;

        tf.transform.translation.x = pos.x;
        tf.transform.translation.y = pos.y;
        tf.transform.translation.z = pos.z;
        tf.transform.rotation.x = ori.x;
        tf.transform.rotation.y = ori.y;
        tf.transform.rotation.z = ori.z;
        tf.transform.rotation.w = ori.w;

        static_broadcaster_->sendTransform(tf);

        RCLCPP_INFO(this->get_logger(),
                "Static TF published: %s -> %s",
                parent_frame.c_str(), child_frame.c_str());

    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    auto node = std::make_shared<StaticTFBroadcaster>();

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}