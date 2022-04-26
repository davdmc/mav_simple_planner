/**
 * @file    mav_simple_planner.h
 * @brief   Simple planner that offers services for
 *          different simple trajectory plans.
 * @author  David Morilla Cabello, V4RL
 * @date    30.07.2021
 */

#pragma once

#include <ros/ros.h>

#include "mav_simple_planner/ServiceCommandCircle.h"
#include "mav_simple_planner/ServiceCommandPoint.h"
#include "mav_simple_planner/ServiceCommandInit.h"
#include "mav_simple_planner/ServiceCommandCoverage.h"
#include "mav_simple_planner/ServiceCommandCoverageFromBB.h"
#include <mav_simple_planner/polynomial_interpolator.hpp>
#include <mutex>

namespace mav_simple_planner {
/*!
*  Main class for the high level simple planner.
*/
class Planner
{

public:
    /**
     * @brief Class constructor
     * @param[in] nh : ROS node handle
     * @param[in] nh_private : private ROS node handle
     */
    Planner(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private);
        
    /**
     * @brief Class destructor
     */
    ~Planner();

private:
    /**
   * @brief Utilities set/get
   */

    /**
   * @brief Method to read the parameters to set up the ROS node from ROS server
   * @return True if all parameters were found; False otherwise
   */
    bool readParamsFromServer();

    /**
   * @brief Method that initializes the ROS communication (pub, sub, srv, timer)
   */
  void initRos();

  void commandCoverageFromBB(double x_min, double x_max, double y_min, double y_max, double z_min, double z_max);
  /**
   * @brief Series of callbacks
   */
  bool commandCircleCallback(mav_simple_planner::ServiceCommandCircle::Request& req, mav_simple_planner::ServiceCommandCircle::Response& res);
  bool commandPointCallback(mav_simple_planner::ServiceCommandPoint::Request& req, mav_simple_planner::ServiceCommandPoint::Response& res);
  bool commandInitCallback(mav_simple_planner::ServiceCommandInit::Request& req, mav_simple_planner::ServiceCommandInit::Response& res);
  bool commandCoverageCallback(mav_simple_planner::ServiceCommandCoverage::Request& req, mav_simple_planner::ServiceCommandCoverage::Response& res);
  bool commandCoverageFromBBCallback(mav_simple_planner::ServiceCommandCoverageFromBB::Request& req, mav_simple_planner::ServiceCommandCoverageFromBB::Response& res);

  void commandTimerCallback(const ros::TimerEvent&);

  void startPlanning();

protected:
  // ROS-related variables
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  ros::Publisher command_pub_;
  ros::Publisher traj_markers_pub_;


// TODO: Change to ROS action
//   ros::ServiceServer start_srv_;
//   ros::ServiceServer stop_srv_;

//   ros::ServiceServer command_point_srv_;
  ros::ServiceServer command_circle_srv_;
  ros::ServiceServer command_point_srv_;
  ros::ServiceServer command_init_srv_;
  ros::ServiceServer command_coverage_srv_;
  ros::ServiceServer command_coverage_from_bb_srv_;
//   ros::ServiceServer command_zig_zag_srv_;

  // Timer to sample waypoints and publish them 
  ros::Timer publish_timer_;

  // Current requested path
  WaypointsList path_waypoints_;
  bool running_;
  
  // Interpolator of the path once a coverage plan has been found
  std::unique_ptr<PolynomialInterpolator> poly_interpolator_;

  // Goals mutex
  std::recursive_mutex goal_mutex_;

  // Parameters for the trajectory interpolation
  double dt_; //[s] this should be greater or equal to interpolation/sampling_dt
  double max_v_; //[m/s]
  double max_a_; //[rad/s]
  double max_ang_v_; //[m/s2]
  double max_ang_a_; //[rad/s2]
  double sampling_dt_; //[s]

  ros::Time current_plan_id_;
  
  // Time at currently published trajectory sample.
  double current_sample_time_;
  // Trajectory obtained from the optimization
  mav_trajectory_generation::Trajectory trajectory_with_yaw_; 

};

}