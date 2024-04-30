#include <gtsam/geometry/Pose2.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>

using namespace gtsam;

/*
---------------------------------------------------------

This example demonstrates how to create a simple factor graph with unary factors.
Unary factors can be interpreted as GNSS measurements, for example.
Error of this measurement does not depend on any other variables in the graph.
Therefore error doesn't accumulate over time.

    f1      f2      f3
    *       *       *
    |       |       |
    O---*---O---*---O
    x1      x2      x3

---------------------------------------------------------
Pose2 is a 2D pose (x, y, theta)
Values can be interpreted as meters and radians

*/

class UnaryFactor : public NoiseModelFactor1<Pose2> {
  double mx_, my_;  ///< X and Y measurements

 public:
  UnaryFactor(Key j, double x, double y, const SharedNoiseModel& model) : NoiseModelFactor1<Pose2>(model, j), mx_(x), my_(y) {}

  Vector evaluateError(const Pose2& q, boost::optional<Matrix&> H = boost::none) const {
    const Rot2& R = q.rotation();
    if (H) {
      (*H) = (gtsam::Matrix(2, 3) << R.c(), -R.s(), 0.0, R.s(), R.c(), 0.0).finished();
    }
    return (Vector(2) << q.x() - mx_, q.y() - my_).finished();
  }
};

int main() {
  const noiseModel::Diagonal::shared_ptr unaryNoise = noiseModel::Diagonal::Sigmas(Vector2(0.1, 0.1));  // 10cm std on x,y

  NonlinearFactorGraph graph;

  graph.add(boost::make_shared<UnaryFactor>(1, 0.0, 0.0, unaryNoise));
  graph.add(boost::make_shared<UnaryFactor>(2, 2.0, 0.0, unaryNoise));
  graph.add(boost::make_shared<UnaryFactor>(3, 4.0, 0.0, unaryNoise));

  graph.print("Factor Graph:\n");

  Values initial;
  initial.insert(1, Pose2(0.5, 0.0, 0.2));
  initial.insert(2, Pose2(2.3, 0.1, -0.2));
  initial.insert(3, Pose2(4.1, 0.1, 0.1));
  initial.print("Initial:\n");

  Values result = LevenbergMarquardtOptimizer(graph, initial).optimize();
  result.print("Final result:\n");
}
