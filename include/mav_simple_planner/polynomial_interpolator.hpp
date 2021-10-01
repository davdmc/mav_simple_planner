/**
 * @file   trajectory_interpolator.h
 * @brief  Class that interpolates a trajectory for a UAV given the waypoints
 * @author Luca Bartolomei, V4RL
 * @date   15.06.2020
 */

#pragma once

#include <mav_msgs/eigen_mav_msgs.h>
#include <mav_trajectory_generation/trajectory.h>

namespace mav_simple_planner {

using PolygonVertices = std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d>>;
using WaypointsList = std::vector<Eigen::Vector4d, Eigen::aligned_allocator<Eigen::Vector4d>>;


class PolynomialInterpolator {
 public:

   /**
     * @brief Class constructor
     * @param[in] max_v : max allowed velocity
     * @param[in] max_a : max allowed acceleration
     * @param[in] max_ang_v : max allowed angular velocity
     * @param[in] max_ang_a : max allowed angular acceleration
     * @param[in] sampling_dt : time step for the interpolation (must be smaller than the algorith that makes use of the trajectory)
     */
  PolynomialInterpolator(const double max_v, const double max_a,
                         const double max_ang_v, const double max_ang_a,
                         const double sampling_dt);
   /**
   * @brief Class destructor
   */
  ~PolynomialInterpolator();

 /**
   * @brief Interpolates positiona and rotation (yaw) of a waypoint path
   * using polynomial optimization (N=10) and with max vel/acc minimizing 
   * the accelerations.
   * @param[in] waypoints : waypoints for the interpolation
   * @param[in] initial_velocity : starting velocity
   * @param[in] final_velocity : final velocity
   * @return True if the interpolation was succesful, false otherwise
   */
  bool interpolate(
      const WaypointsList &waypoints,
      const Eigen::Vector3d &initial_velocity = Eigen::Vector3d::Zero(),
      const Eigen::Vector3d &final_velocity = Eigen::Vector3d::Zero());

  bool interpolate(
      const mav_msgs::EigenTrajectoryPoint::Vector &path,
      const Eigen::Vector3d &initial_velocity = Eigen::Vector3d::Zero(),
      const Eigen::Vector3d &final_velocity = Eigen::Vector3d::Zero());

  mav_trajectory_generation::Trajectory getTrajectory() const {
    return trajectory_with_yaw_;
  }

 protected:
  double max_v_;  //[m/s]
  double max_a_; //[rad/s]
  double max_ang_v_; //[m/s2]
  double max_ang_a_; //[rad/s2]
  double sampling_dt_; //[s]

  // Final interpolated trajectory
  mav_trajectory_generation::Trajectory trajectory_with_yaw_;
};  // end class PolynomialInterpolator

}  // end namespace mav_simple_planner
