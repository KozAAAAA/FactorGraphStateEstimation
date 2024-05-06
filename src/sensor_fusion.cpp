#include <gtsam/geometry/Pose2.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>

using namespace gtsam;

/*
---------------------------------------------------------


    fg1     fg2     fg3
    *       *       *
    |  fo1  |  fo2  |
    O---*---O---*---O
    x1      x2      x3

---------------------------------------------------------
Pose2 is a 2D pose (x, y, theta)
Values can be interpreted as meters and radians

*/

class UnaryFactor : public NoiseModelFactor1<Pose2> {
 private:
  double measuredX_, measuredY_;  ///< X and Y measurements

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

int main() {
  const noiseModel::Diagonal::shared_ptr unaryNoise = noiseModel::Diagonal::Sigmas(Vector2(0.1, 0.1));          // 10cm std on x,y
  const noiseModel::Diagonal::shared_ptr odometryNoise = noiseModel::Diagonal::Sigmas(Vector3(0.2, 0.2, 0.1));  // uncertainty of the odometry measurement
  
  float gps_x = 0.0;

  float y = 0.0;
  float theta = 0.0;

  float x_move[] = {1.0, 2.0, 1.0, 3.0, 1.0};
  
  NonlinearFactorGraph graph;
  Values initial;
  

  for (int i = 0; i < sizeof(x_move)/sizeof(x_move[0]); i++) {
    float current_x_move = x_move[i];
    std::cout << "Current x move: " << current_x_move << std::endl;
    int key = i + 1;

    graph.add(boost::make_shared<UnaryFactor>(key, gps_x, y, unaryNoise));
    initial.insert(key, Pose2(gps_x, y, theta));
    
    Values result = LevenbergMarquardtOptimizer(graph, initial).optimize();
    Pose2 optimized_pose = result.at<Pose2>(key);
    initial.update(key, optimized_pose);
    optimized_pose.print("Optimized pose: ");

    gps_x += current_x_move; // fake gps measurement
    float odometry_x = current_x_move; // fake odometry measurement
    
    graph.add(BetweenFactor<Pose2>(key, key+1, Pose2(odometry_x, y, theta), odometryNoise));
  }
}
