// IMU -> velocity command node.
//
// Subscribes : /imu/data         (sensor_msgs/msg/Imu)
// Publishes  : /input/command    (geometry_msgs/msg/Twist)
//
// Search for "TODO" to find the parts you need to write.

#include <memory>

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"

namespace {
constexpr char IMU_TOPIC[] = "/imu/data";
constexpr char CMD_TOPIC[] = "/input/command";
constexpr size_t QUEUE_SIZE = 10;
}  // namespace

// Plain container for the raw IMU values we care about.
struct ImuData {
  // Linear acceleration [m/s^2]
  double accel_x = 0.0;
  double accel_y = 0.0;
  double accel_z = 0.0;
  // Angular velocity [rad/s]
  double gyro_x = 0.0;
  double gyro_y = 0.0;
  double gyro_z = 0.0;
};

class ImuControllerNode : public rclcpp::Node {
 public:
  ImuControllerNode() : Node("imu_controller") {
    // TODO 1: Create a subscription to IMU_TOPIC and store it in imu_sub_.
    //   - message type: sensor_msgs::msg::Imu
    //   - queue size:   QUEUE_SIZE
    //   - callback:     imu_callback
    //   Hint: this->create_subscription<...>(topic, qos, callback)
    //         For the callback, look up std::bind(..., this,
    //         std::placeholders::_1) or use a lambda that captures `this`.

    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
        IMU_TOPIC, QUEUE_SIZE,
        std::bind(&ImuControllerNode::imu_callback, this,
                  std::placeholders::_1));

    // TODO 2: Create a publisher on CMD_TOPIC and store it in cmd_pub_.
    //   - message type: geometry_msgs::msg::Twist
    //   - queue size:   QUEUE_SIZE
    //   Hint: this->create_publisher<...>(topic, qos)
    cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(CMD_TOPIC,
                                                                 QUEUE_SIZE);

    RCLCPP_INFO(get_logger(), "imu_controller started: %s -> %s", IMU_TOPIC,
                CMD_TOPIC);
  }

 private:
  // Called every time a new IMU message arrives.
  void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg) {
    // TODO 3: Copy the raw readings from `msg` into imu_data_.
    //   Hint: look at the Imu message definition:
    //     ros2 interface show sensor_msgs/msg/Imu
    //   You want msg->linear_acceleration.{x,y,z} and
    //   msg->angular_velocity.{x,y,z}.

    // Copy linear acceleration [m/s^2]
    imu_data_.accel_x = msg->linear_acceleration.x;
    imu_data_.accel_y = msg->linear_acceleration.y;
    imu_data_.accel_z = msg->linear_acceleration.z;

    // Copy angular velocity [rad/s]
    imu_data_.gyro_x = msg->angular_velocity.x;
    imu_data_.gyro_y = msg->angular_velocity.y;
    imu_data_.gyro_z = msg->angular_velocity.z;

    geometry_msgs::msg::Twist cmd = compute_command(imu_data_);

    // TODO 5: Publish `cmd` using the publisher you created in TODO 2.
    cmd_pub_->publish(cmd);
  }

  // The fun part: decide how the vehicle should move given the IMU reading.
  //
  // Twist fields:
  //   cmd.linear.x   surge  (forward / backward)
  //   cmd.linear.y   sway   (left / right)
  //   cmd.linear.z   heave  (up / down)
  //   cmd.angular.x  roll
  //   cmd.angular.y  pitch
  //   cmd.angular.z  yaw
  //
  // Anything you leave untouched stays 0.0.
  geometry_msgs::msg::Twist compute_command(const ImuData& imu) {
    geometry_msgs::msg::Twist cmd;

    // TODO 4: Design your own mapping from `imu` to `cmd`.
    //   See the README for ideas. Keep the output bounded and
    //   think about what should happen when the IMU is sitting still.

    // 1. Deadband thresholds (ignore tiny sensor noise)
    constexpr double ACCEL_DEADBAND = 0.5;  // m/s^2
    constexpr double GYRO_DEADBAND = 0.1;   // rad/s

    // 2. Scaling constants to keep movement reasonable
    constexpr double LINEAR_SCALE = 0.2;
    constexpr double ANGULAR_SCALE = 0.5;

    // Surge (Forward / Backward)
    if (std::abs(imu.accel_x) > ACCEL_DEADBAND) {
      cmd.linear.x = imu.accel_x * LINEAR_SCALE;
    }

    // Sway (Left / Right)
    if (std::abs(imu.accel_y) > ACCEL_DEADBAND) {
      cmd.linear.y = imu.accel_y * LINEAR_SCALE;
    }

    // Yaw (Spinning)
    if (std::abs(imu.gyro_z) > GYRO_DEADBAND) {
      cmd.angular.z = imu.gyro_z * ANGULAR_SCALE;
    }

    // 3. Clamp output values to safe range [-1.0, 1.0]
    auto clamp = [](double val, double min_val, double max_val) {
      return std::max(min_val, std::min(val, max_val));
    };

    cmd.linear.x = clamp(cmd.linear.x, -1.0, 1.0);
    cmd.linear.y = clamp(cmd.linear.y, -1.0, 1.0);
    cmd.angular.z = clamp(cmd.angular.z, -1.0, 1.0);

    return cmd;
  }

  ImuData imu_data_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ImuControllerNode>());
  rclcpp::shutdown();
  return 0;
}
