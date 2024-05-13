#include <gtsam/geometry/Pose2.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>

#include <chrono>
#include <tuple>

#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace gtsam;
using namespace geometry_msgs;
using namespace std::placeholders;
using namespace std::chrono_literals;

class SensorFusion : public rclcpp::Node {
 public:
  SensorFusion();

 private:
  void optim_pose_callback();
  void relative_pose_callback_1(const msg::PoseWithCovarianceStamped::SharedPtr msg);
  void relative_pose_callback_2(const msg::PoseWithCovarianceStamped::SharedPtr msg);
  void absolute_pose_callback(const msg::PoseWithCovarianceStamped::SharedPtr msg);
  std::tuple<double, double, double> get_variance(const msg::PoseWithCovarianceStamped::SharedPtr msg);

  rclcpp::TimerBase::SharedPtr optim_pose_timer;
  rclcpp::Publisher<msg::PoseWithCovarianceStamped>::SharedPtr optim_pose_publisher;
  rclcpp::Subscription<msg::PoseWithCovarianceStamped>::SharedPtr relative_pose_subscriber_1;
  rclcpp::Subscription<msg::PoseWithCovarianceStamped>::SharedPtr relative_pose_subscriber_2;
  rclcpp::Subscription<msg::PoseWithCovarianceStamped>::SharedPtr absolute_pose_subscriber;

  constexpr static std::chrono::milliseconds OPTIM_POSE_RATE = 1s;

  unsigned int key = 1;

  NonlinearFactorGraph graph;
  Values initial;

  Pose2 relative_pose_1;
  noiseModel::Diagonal::shared_ptr relative_noise_1;
  bool new_relative_pose_1 = false;

  Pose2 relative_pose_2;
  noiseModel::Diagonal::shared_ptr relative_noise_2;
  bool new_relative_pose_2 = false;

  Pose2 absolute_pose;
  noiseModel::Diagonal::shared_ptr absolute_noise;
  bool new_absolute_pose = false;
};

SensorFusion::SensorFusion() : Node("sensor_fusion") {
  optim_pose_timer = this->create_wall_timer(OPTIM_POSE_RATE, std::bind(&SensorFusion::optim_pose_callback, this));

  optim_pose_publisher = this->create_publisher<msg::PoseWithCovarianceStamped>("optim_pose", 1);

  relative_pose_subscriber_1 =
      this->create_subscription<msg::PoseWithCovarianceStamped>("relative_pose_1", 1, std::bind(&SensorFusion::relative_pose_callback_1, this, _1));
  relative_pose_subscriber_2 =
      this->create_subscription<msg::PoseWithCovarianceStamped>("relative_pose_2", 1, std::bind(&SensorFusion::relative_pose_callback_2, this, _1));
  absolute_pose_subscriber =
      this->create_subscription<msg::PoseWithCovarianceStamped>("absolute_pose", 1, std::bind(&SensorFusion::absolute_pose_callback, this, std::placeholders::_1));
}

void SensorFusion::optim_pose_callback() {
  if (new_relative_pose_1 && new_relative_pose_2 && new_absolute_pose) {
    new_relative_pose_1 = false;
    new_relative_pose_2 = false;
    new_absolute_pose = false;

    // TODO: OPTIMIZATION HERE
    // TODO: Krzysztof, please implement the optimization here based on the MAIN branch
    (void)relative_pose_1; // ODOMETRY LIKE DATA
    (void)relative_noise_1; // ODOMETRY LIKE DATA
    (void)absolute_pose; // GNSS LIKE DATA

    }
  }

  void SensorFusion::relative_pose_callback_1(const msg::PoseWithCovarianceStamped::SharedPtr msg) {
    RCLCPP_INFO(this->get_logger(), "Received relative pose 1");

    auto [var_x, var_y, var_theta] = get_variance(msg);
    relative_noise_1 = noiseModel::Diagonal::Sigmas(Vector3(var_x, var_y, var_theta));
    relative_pose_1 = Pose2(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.orientation.z);
    new_relative_pose_1 = true;
  }

  void SensorFusion::relative_pose_callback_2(const msg::PoseWithCovarianceStamped::SharedPtr msg) {
    RCLCPP_INFO(this->get_logger(), "Received relative pose 2");

    auto [var_x, var_y, var_theta] = get_variance(msg);
    relative_noise_2 = noiseModel::Diagonal::Sigmas(Vector3(var_x, var_y, var_theta));
    relative_pose_2 = Pose2(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.orientation.z);
    new_relative_pose_2 = true;
  }

  void SensorFusion::absolute_pose_callback(const msg::PoseWithCovarianceStamped::SharedPtr msg) {
    RCLCPP_INFO(this->get_logger(), "Received absolute pose");

    auto [var_x, var_y, var_theta] = get_variance(msg);
    absolute_noise = noiseModel::Diagonal::Sigmas(Vector3(var_x, var_y, var_theta));
    absolute_pose = Pose2(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.orientation.z);
    new_absolute_pose = true;
  }

  std::tuple<double, double, double> SensorFusion::get_variance(const msg::PoseWithCovarianceStamped::SharedPtr msg) {
    // Extract from the diagonal of the covariance matrix
    double cov_x = msg->pose.covariance[0];
    double cov_y = msg->pose.covariance[7];
    double cov_theta = msg->pose.covariance[35];
    return std::make_tuple(cov_x, cov_y, cov_theta);
  }

  int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SensorFusion>());
    rclcpp::shutdown();
    return 0;
  }
