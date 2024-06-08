#include <gtsam/geometry/Pose2.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>

#include <chrono>
#include <tuple>

#include "geometry_msgs/msg/pose_with_covariance.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"


using namespace gtsam;
using namespace geometry_msgs;
using namespace std::placeholders;
using namespace std::chrono_literals;


class UnaryFactor : public NoiseModelFactor1<Pose2> {
 private:
  double measuredX_, measuredY_, measuredTheta_;

 public:
  UnaryFactor(Key key, double x, double y, double theta, const SharedNoiseModel& model)
      : NoiseModelFactor1<Pose2>(model, key), measuredX_(x), measuredY_(y), measuredTheta_(theta) {}

  Vector evaluateError(const Pose2& estimatedPose, boost::optional<Matrix&> H = boost::none) const {
    // const Rot2& R = estimatedPose.rotation(); // unused?

    // clang-format off
    // Jacobian
    if (H) (*H) = (gtsam::Matrix(3, 3) << 
                  1.0, 0.0, 0.0,
                  0.0, 1.0, 0.0,
                  0.0, 0.0, 1.0
                  ).finished();
                  
    // cost function
    return (Vector(3) << 
                  estimatedPose.x() - measuredX_, 
                  estimatedPose.y() - measuredY_,
                  estimatedPose.theta() - measuredTheta_
                  ).finished();
    // clang-format on
  }
};


class SensorFusion : public rclcpp::Node {
 public:
  SensorFusion();

 private:
  void optim_pose_callback();
  void relative_pose_callback_1(const msg::PoseWithCovariance::SharedPtr msg);
  void relative_pose_callback_2(const msg::PoseWithCovariance::SharedPtr msg);
  void absolute_pose_callback(const msg::PoseWithCovariance::SharedPtr msg);
  std::tuple<double, double, double> get_variance(const msg::PoseWithCovariance::SharedPtr msg);

  rclcpp::TimerBase::SharedPtr optim_pose_timer;
  rclcpp::Publisher<msg::Pose>::SharedPtr optim_pose_publisher;
  rclcpp::Subscription<msg::PoseWithCovariance>::SharedPtr relative_pose_subscriber_1;
  rclcpp::Subscription<msg::PoseWithCovariance>::SharedPtr relative_pose_subscriber_2;
  rclcpp::Subscription<msg::PoseWithCovariance>::SharedPtr absolute_pose_subscriber;

  constexpr static std::chrono::milliseconds OPTIM_POSE_RATE = 1ms;

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

  optim_pose_publisher = this->create_publisher<msg::Pose>("optim_pose", 1);

  relative_pose_subscriber_1 =
      this->create_subscription<msg::PoseWithCovariance>("odom_relative_pose", 1, std::bind(&SensorFusion::relative_pose_callback_1, this, _1));
  relative_pose_subscriber_2 =
      this->create_subscription<msg::PoseWithCovariance>("imu_relative_pose", 1, std::bind(&SensorFusion::relative_pose_callback_2, this, _1));
  absolute_pose_subscriber =
      this->create_subscription<msg::PoseWithCovariance>("gps_pose", 1, std::bind(&SensorFusion::absolute_pose_callback, this, std::placeholders::_1));
}

void SensorFusion::optim_pose_callback() {
  if (new_relative_pose_1 && new_relative_pose_2 && new_absolute_pose) {
    new_relative_pose_1 = false;
    new_relative_pose_2 = false;
    new_absolute_pose = false;

    graph.add(boost::make_shared<UnaryFactor>(key, absolute_pose.x(), absolute_pose.y(),  absolute_pose.theta(), absolute_noise));
    initial.insert(key, absolute_pose);

    auto optimPose = LevenbergMarquardtOptimizer(graph, initial).optimize().at<Pose2>(key);

    initial.update(key, optimPose);
    graph.add(BetweenFactor<Pose2>(key, key + 1, relative_pose_1, relative_noise_1));
    graph.add(BetweenFactor<Pose2>(key, key + 1, relative_pose_2, relative_noise_2));

    // Publish the optimized pose
    RCLCPP_INFO(this->get_logger(), "Published optim pose %d", key);
    msg::Pose optim_pose_msg;
    optim_pose_msg.position.x = optimPose.x();
    optim_pose_msg.position.y = optimPose.y();
    optim_pose_msg.orientation.z = optimPose.theta();
    optim_pose_publisher->publish(optim_pose_msg);

    key++;
    }
  }

  void SensorFusion::relative_pose_callback_1(const msg::PoseWithCovariance::SharedPtr msg) {
    RCLCPP_INFO(this->get_logger(), "Received relative pose 1");

    auto [var_x, var_y, var_theta] = get_variance(msg);
    relative_noise_1 = noiseModel::Diagonal::Sigmas(Vector3(var_x, var_y, var_theta));
    relative_pose_1 = Pose2(msg->pose.position.x, msg->pose.position.y, 0);
    new_relative_pose_1 = true;
  }

  void SensorFusion::relative_pose_callback_2(const msg::PoseWithCovariance::SharedPtr msg) {
    RCLCPP_INFO(this->get_logger(), "Received relative pose 2");

    auto [var_x, var_y, var_theta] = get_variance(msg);
    relative_noise_2 = noiseModel::Diagonal::Sigmas(Vector3(var_x, var_y, var_theta));
    relative_pose_2 = Pose2(msg->pose.position.x, msg->pose.position.y, 0);
    new_relative_pose_2 = true;
  }

  void SensorFusion::absolute_pose_callback(const msg::PoseWithCovariance::SharedPtr msg) {
    RCLCPP_INFO(this->get_logger(), "Received absolute pose");

    auto [var_x, var_y, var_theta] = get_variance(msg);
    absolute_noise = noiseModel::Diagonal::Sigmas(Vector3(var_x, var_y, var_theta));
    absolute_pose = Pose2(msg->pose.position.x, msg->pose.position.y, 0);
    new_absolute_pose = true;
  }

  std::tuple<double, double, double> SensorFusion::get_variance(const msg::PoseWithCovariance::SharedPtr msg) {
    // Extract from the diagonal of the covariance matrix
    double cov_x = msg->covariance[0];
    double cov_y = msg->covariance[7];
    double cov_theta = msg->covariance[35];
    return std::make_tuple(cov_x, cov_y, cov_theta);
  }

  int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SensorFusion>());
    rclcpp::shutdown();
    return 0;
  }
