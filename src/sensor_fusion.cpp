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
  double measuredX_, measuredY_, measuredTheta_;

 public:
  UnaryFactor(Key key, double x, double y, double theta, const SharedNoiseModel& model)
      : NoiseModelFactor1<Pose2>(model, key), measuredX_(x), measuredY_(y), measuredTheta_(theta) {}

  Vector evaluateError(const Pose2& estimatedPose, boost::optional<Matrix&> H = boost::none) const {
    const Rot2& R = estimatedPose.rotation();

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

// -------------------------- USED FOR MOCKING DATA ---------------------------
static constexpr int N_KEYS = 5;

std::tuple<float, float, float> getGlobal() {
  static int i = -1;
  static std::array<float, N_KEYS> x = {0.0, 1.0, 2.0, 3.0, 4.0};
  static std::array<float, N_KEYS> y = {0.0, 1.0, 2.0, 3.0, 4.0};
  static std::array<float, N_KEYS> theta = {0.0, 0.0, 0.0, 0.0, 0.0};
  i++;
  return std::make_tuple(x[i], y[i], theta[i]);
}

std::tuple<float, float, float> getRelative1() {
  static int i = -1;
  static std::array<float, N_KEYS> x = {1.0, 1.0, 1.0, 1.0, 1.0};
  static std::array<float, N_KEYS> y = {1.0, 1.0, 1.0, 1.0, 1.0};
  static std::array<float, N_KEYS> theta = {0.0, 0.0, 0.0, 0.0, 0.0};
  i++;
  return std::make_tuple(x[i], y[i], theta[i]);
}

std::tuple<float, float, float> getRelative2() {
  static int i = -1;
  static std::array<float, N_KEYS> x = {1.0, 1.0, 1.0, 1.0, 1.0};
  static std::array<float, N_KEYS> y = {1.0, 1.0, 1.0, 1.0, 1.0};
  static std::array<float, N_KEYS> theta = {0.0, 0.0, 0.0, 0.0, 0.0};
  i++;
  return std::make_tuple(x[i], y[i], theta[i]);
}

// ----------------------------------------------------------------------------

// TODO: Theta doesn't work - optimPose gets worse with each iteration (but only if theta changes)
int main() {
  const noiseModel::Diagonal::shared_ptr globalNoise = noiseModel::Diagonal::Sigmas(Vector3(0.1, 0.1, 0.02));
  const noiseModel::Diagonal::shared_ptr relativeNoise = noiseModel::Diagonal::Sigmas(Vector3(0.1, 0.1, 0.02));

  NonlinearFactorGraph graph;
  Values initial;

  for (int key = 1; key < N_KEYS + 1; key++) {
    auto [globalX, globalY, globalTheta] = getGlobal();               // mock global measurements
    auto [relativeX1, relativeY1, reltaiveTheta1] = getRelative1();   // mock relative measurements
    auto [relativeX2, relativeY2, reltaiveTheta2] = getRelative2();   // mock relative measurements

    graph.add(boost::make_shared<UnaryFactor>(key, globalX, globalY, globalTheta, globalNoise));
    initial.insert(key, Pose2(globalX, globalY, globalTheta));

    auto optimPose = LevenbergMarquardtOptimizer(graph, initial).optimize().at<Pose2>(key);

    initial.update(key, optimPose);
    graph.add(BetweenFactor<Pose2>(key, key + 1, Pose2(relativeX1, relativeY1, reltaiveTheta1), relativeNoise));
    graph.add(BetweenFactor<Pose2>(key, key + 1, Pose2(relativeX2, relativeY2, reltaiveTheta2), relativeNoise));

    optimPose.print("Optimized pose at key " + std::to_string(key) + ": ");
  }
}
