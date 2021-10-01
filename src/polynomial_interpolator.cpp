/**
 * @file   trajectory_interpolator.cpp
 * @author Luca Bartolomei, V4RL
 * @date   15.06.2020
 */

#include "mav_simple_planner/polynomial_interpolator.hpp"

#include <mav_trajectory_generation/polynomial_optimization_nonlinear.h>
#include <mav_trajectory_generation/trajectory_sampling.h>

namespace mav_simple_planner {

PolynomialInterpolator::PolynomialInterpolator(const double max_v,
                                               const double max_a,
                                               const double max_ang_v,
                                               const double max_ang_a,
                                               const double sampling_dt)
    : max_v_(max_v),
      max_a_(max_a),
      max_ang_v_(max_ang_v),
      max_ang_a_(max_ang_a),
      sampling_dt_(sampling_dt) {}

PolynomialInterpolator::~PolynomialInterpolator() {}

bool PolynomialInterpolator::interpolate(
    const mav_msgs::EigenTrajectoryPoint::Vector &path,
    const Eigen::Vector3d &initial_velocity,
    const Eigen::Vector3d &final_velocity) {
  WaypointsList waypoints;
  for (const auto &pose : path) {
    waypoints.push_back(Eigen::Vector4d(pose.position_W.x(),
                                        pose.position_W.y(),
                                        pose.position_W.z(), pose.getYaw()));
  }

  return interpolate(waypoints, initial_velocity, final_velocity);
}

bool PolynomialInterpolator::interpolate(
    const WaypointsList &waypoints, const Eigen::Vector3d &initial_velocity,
    const Eigen::Vector3d &final_velocity) {
  if (waypoints.empty()) {
    return false;
  }

  // Step 1: generate yaws and initialize the vector with the current robot yaw
  std::vector<double> yaws;
  for (size_t i = 0; i < waypoints.size(); ++i) {
    yaws.push_back(waypoints[i](3));
  }

  // Step 2: generate the vertices for the interpolation
  mav_trajectory_generation::Vertex::Vector vertices, vertices_yaw;
  const int dimension = 3;
  
  // Set the optimization derivative costs
  const int derivative_to_optimize =
      mav_trajectory_generation::derivative_order::ACCELERATION;
  const int derivative_to_optimize_yaw =
      mav_trajectory_generation::derivative_order::ANGULAR_ACCELERATION;

  // Generate initial vertices for position and yaw
  mav_trajectory_generation::Vertex vertex(dimension), vertex_yaw(1);
  vertex.makeStartOrEnd(waypoints[0].head(3), derivative_to_optimize);
  vertex.addConstraint(mav_trajectory_generation::derivative_order::VELOCITY,
                       initial_velocity);
  double yaw_old = yaws[0];
  vertex_yaw.makeStartOrEnd(yaws[0], derivative_to_optimize_yaw);
  
  // Add vertex to the trajectory vertices
  vertices.push_back(vertex);
  vertices_yaw.push_back(vertex_yaw);

  // Add the rest of the waypoints as position and orientation constraints
  for (size_t i = 1; i < waypoints.size() - 1; ++i) {
    mav_trajectory_generation::Vertex vertex_m(dimension), vertex_yaw_m(1);
    
    // Position
    vertex_m.addConstraint(
        mav_trajectory_generation::derivative_order::POSITION,
        waypoints[i].head(3));

    // Rotation (yaw)

    // Keep angle between 0 and 2*pi for positive and negative values
    if (std::abs(yaws[i] + 2 * M_PI - yaw_old) < std::abs(yaws[i] - yaw_old)) {
      yaw_old = yaws[i] + 2 * M_PI;
    } else if (std::abs(yaws[i] - 2 * M_PI - yaw_old) <
               std::abs(yaws[i] - yaw_old)) {
      yaw_old = yaws[i] - 2 * M_PI;
    } else {
      yaw_old = yaws[i];
    }
    vertex_yaw_m.addConstraint(
        mav_trajectory_generation::derivative_order::ORIENTATION, yaw_old);
    
    // Add vertex to the trajectory vertices
    vertices.push_back(vertex_m);
    vertices_yaw.push_back(vertex_yaw_m);
  }

  // Generate final vertices for position and yaw
  vertex.makeStartOrEnd(waypoints.back().head(3), derivative_to_optimize);
  vertex.addConstraint(mav_trajectory_generation::derivative_order::VELOCITY,
                       final_velocity);

  if (std::abs(yaws.back() + 2 * M_PI - yaw_old) <
      std::abs(yaws.back() - yaw_old)) {
    yaw_old = yaws.back() + 2 * M_PI;
  } else if (std::abs(yaws.back() - 2 * M_PI - yaw_old) <
             std::abs(yaws.back() - yaw_old)) {
    yaw_old = yaws.back() - 2 * M_PI;
  } else {
    yaw_old = yaws.back();
  }
  vertex_yaw.makeStartOrEnd(yaw_old, derivative_to_optimize_yaw);
  vertices.push_back(vertex);
  vertices_yaw.push_back(vertex_yaw);

  // Step 3: obtain interpolated trajectory

  // Obtain segment times
  std::vector<double> segment_times =
      mav_trajectory_generation::estimateSegmentTimes(vertices, max_v_, max_a_);
  std::vector<double> segment_times_yaw{
      mav_trajectory_generation::estimateSegmentTimes(vertices_yaw, max_ang_v_,
                                                      max_ang_a_)};

  // Check segment times for consistency
  for (int i = 0; i < segment_times.size(); ++i) {
    if (segment_times_yaw[i] > segment_times[i])
      segment_times[i] = segment_times_yaw[i];

    if (segment_times[i] < sampling_dt_) 
      segment_times[i] = sampling_dt_;
  }

  // Apply polynomial optimization for position and orientation
  const int N = 10;
  mav_trajectory_generation::PolynomialOptimization<N> opt(dimension);
  opt.setupFromVertices(vertices, segment_times, derivative_to_optimize);
  opt.solveLinear();

  mav_trajectory_generation::PolynomialOptimization<N> opt_yaw(1);
  opt_yaw.setupFromVertices(vertices_yaw, segment_times,
                            derivative_to_optimize_yaw);
  opt_yaw.solveLinear();

  // Get trajectories and combine
  mav_trajectory_generation::Trajectory trajectory;
  mav_trajectory_generation::Trajectory trajectory_yaw;
  opt.getTrajectory(&trajectory);
  opt_yaw.getTrajectory(&trajectory_yaw);
  trajectory.getTrajectoryWithAppendedDimension(trajectory_yaw,
                                                &trajectory_with_yaw_);
  return true;
}

}  // end namespace planning_utils
