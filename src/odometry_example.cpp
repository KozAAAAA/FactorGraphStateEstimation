#include <gtsam/geometry/Pose2.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>

using namespace gtsam;

/*
---------------------------------------------------------
This example demonstrates how to create a simple factor graph with binary factors.
Binary factors can be interpreted as odometry measurements, for example.
Error of this measurement depends on two variables in the graph.
Therefore error accumulates over time.

f1      f2      f3
*---O---*---O---*---O
    x1      x2      x3

f1 = PriorFactor<Pose2>(1, priorMean, priorNoise)
f2 = BetweenFactor<Pose2>(1, 2, odometry1, odometryNoise)
f3 = BetweenFactor<Pose2>(2, 3, odometry2, odometryNoise)

x1 - estimated pose at time 1
x2 - estimated pose at time 2
x3 - estimated pose at time 3

---------------------------------------------------------
Pose2 is a 2D pose (x, y, theta)
Values can be interpreted as meters and radians

*/

int main() {
  const noiseModel::Diagonal::shared_ptr priorNoise = noiseModel::Diagonal::Sigmas(Vector3(0.3, 0.3, 0.1));     // uncertainty of the initial guess
  const noiseModel::Diagonal::shared_ptr odometryNoise = noiseModel::Diagonal::Sigmas(Vector3(0.2, 0.2, 0.1));  // uncertainty of the odometry measurement
  // Create an empty nonlinear factor graph
  NonlinearFactorGraph graph;

  Pose2 priorMean(0.0, 0.0, 0.0);  // initial guess
  graph.add(PriorFactor<Pose2>(1, priorMean, priorNoise));

  Pose2 odometry1(2.0, 0.0, 0.0);  // first odometry measurement
  graph.add(BetweenFactor<Pose2>(1, 2, odometry1, odometryNoise));

  Pose2 odometry2(2.0, 0.0, 0.0);  // second odometry measurement
  graph.add(BetweenFactor<Pose2>(2, 3, odometry2, odometryNoise));
  graph.print("Factor Graph:\n");

  Values initial;
  initial.insert(1, Pose2(0.5, 0.0, 0.2));
  initial.insert(2, Pose2(2.3, 0.1, -0.2));
  initial.insert(3, Pose2(4.1, 0.1, 0.1));
  initial.print("Initial:\n");

  Values result = LevenbergMarquardtOptimizer(graph, initial).optimize();
  result.print("Final result:\n");
}
