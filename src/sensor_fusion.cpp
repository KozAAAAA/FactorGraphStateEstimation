#include <gtsam/geometry/Pose2.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>

#include <tuple>

using namespace gtsam;

/*
---------------------------------------------------------

   gnss    gnss    gnss
    *       *       *
    |  odom |  odom |
    O---*---O---*---O
    |       |       |
    |---*---|---*---|
       imu     imu

---------------------------------------------------------
Pose2 is a 2D pose (x, y, theta)
Values can be interpreted as meters and radians

*/

class UnaryFactor : public NoiseModelFactor1<Pose2> {
 private:
  double measuredX_, measuredY_;

 public:
  UnaryFactor(Key key, double x, double y, const SharedNoiseModel& model) : NoiseModelFactor1<Pose2>(model, key), measuredX_(x), measuredY_(y) {}

  // clang-format off
  // return cost function
  Vector evaluateError(const Pose2& estimatedPose, boost::optional<Matrix&> H = boost::none) const {
    const Rot2& R = estimatedPose.rotation();
    
    if (H) (*H) = (gtsam::Matrix(2, 3) << R.c(), -R.s(), 0.0, 
                                     R.s(),  R.c(), 0.0).finished();
    
    return (Vector(2) << estimatedPose.x() - measuredX_, 
                         estimatedPose.y() - measuredY_).finished();
  }
  // clang-format on
};

// -------------------------- USED FOR MOCKING DATA ---------------------------
static constexpr int N_KEYS = 5;

std::tuple<float, float> getGnss() {
  static int i = -1;
  static std::array<float, N_KEYS> x = {0.0, 1.0, 3.0, 4.0, 7.0};
  static std::array<float, N_KEYS> y = {0.0, 1.0, 3.0, 4.0, 7.0};
  i++;
  return std::make_tuple(x[i], y[i]);
}

std::tuple<float, float, float> getOdometry() {
  static int i = -1;
  static std::array<float, N_KEYS> x = {1.0, 2.0, 1.0, 3.0, 1.0};
  static std::array<float, N_KEYS> y = {1.0, 2.0, 1.0, 3.0, 1.0};
  static std::array<float, N_KEYS> theta = {0.0, 0.0, 0.0, 0.0, 0.0};
  i++;
  return std::make_tuple(x[i], y[i], theta[i]);
}

std::tuple<float, float, float> getImu() {
  static int i = -1;
  static std::array<float, N_KEYS> x = {1.0, 2.0, 1.0, 3.0, 1.0};
  static std::array<float, N_KEYS> y = {1.0, 2.0, 1.0, 3.0, 1.0};
  static std::array<float, N_KEYS> theta = {0.0, 0.0, 0.0, 0.0, 0.0};
  i++;
  return std::make_tuple(x[i], y[i], theta[i]);
}
// ----------------------------------------------------------------------------

int main() {
  const noiseModel::Diagonal::shared_ptr gnssNoise = noiseModel::Diagonal::Sigmas(Vector2(0.1, 0.1));
  const noiseModel::Diagonal::shared_ptr odometryNoise = noiseModel::Diagonal::Sigmas(Vector3(0.2, 0.2, 0.1));
  const noiseModel::Diagonal::shared_ptr imuNoise = noiseModel::Diagonal::Sigmas(Vector3(0.3, 0.3, 0.3));

  NonlinearFactorGraph graph;
  Values initial;

  for (int key = 1; key < N_KEYS + 1; key++) {
    auto [gnssX, gnssY] = getGnss();                 // fake gnss data
    auto [odomX, odomY, odomTheta] = getOdometry();  // fake odometry data
    auto [imuX, imuY, imuTheta] = getImu();          // fake imu data

    graph.add(boost::make_shared<UnaryFactor>(key, gnssX, gnssY, gnssNoise)); // What about theta ???
    initial.insert(key, Pose2(gnssX, gnssY, 0.0));  // WHY IS IT THERE??? How to handle theta ???

    Values result = LevenbergMarquardtOptimizer(graph, initial).optimize();
    Pose2 optimPose = result.at<Pose2>(key);
    initial.update(key, optimPose);

    graph.add(BetweenFactor<Pose2>(key, key + 1, Pose2(odomX, odomY, odomTheta), odometryNoise));
    graph.add(BetweenFactor<Pose2>(key, key + 1, Pose2(imuX, imuY, imuTheta), imuNoise));

    optimPose.print("Optimized Pose: ");
  }
}
