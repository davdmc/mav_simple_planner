#include "mav_simple_planner/simple_planner.hpp"

#include <std_msgs/Time.h>

#include <visualization_msgs/MarkerArray.h>

#include <trajectory_msgs/MultiDOFJointTrajectory.h>

#include <mav_msgs/default_topics.h>
#include <mav_msgs/eigen_mav_msgs.h>
#include <mav_msgs/conversions.h>

#include <mav_trajectory_generation/trajectory_sampling.h>
#include <mav_trajectory_generation_ros/ros_visualization.h>

namespace mav_simple_planner {
Planner::Planner(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private):nh_(nh),nh_private_(nh_private){

if(!readParamsFromServer()){
    ROS_FATAL("[Simple planner] Could not read parameters from server!");
    ros::requestShutdown();
}

initRos();

// TODO: init trajectory interpolator
poly_interpolator_ = std::unique_ptr<PolynomialInterpolator>(new PolynomialInterpolator(max_v_, max_a_, max_ang_v_, max_ang_a_, sampling_dt_));
// Inform the user
ROS_INFO("[Simple planner] Initialization complete");

}

Planner::~Planner(){}

bool Planner::readParamsFromServer(){

    /***************** Trajectory interpolation *****************/
    if(!nh_private_.getParam("planner/dt", dt_)) {
        ROS_WARN("[Simple planner] Parameter planner/dt not specified");
        return false;
    }
    if(!nh_private_.getParam("interpolation/max_v", max_v_)) {
        ROS_WARN("[Simple planner] Parameter planner/max_v not specified");
        return false;
    }      
    if(!nh_private_.getParam("interpolation/max_a", max_a_)) {
        ROS_WARN("[Simple planner] Parameter planner/max_a not specified");
        return false;
    }
    if(!nh_private_.getParam("interpolation/max_ang_v", max_ang_v_)) {
        ROS_WARN("[Simple planner] Parameter planner/max_ang_v not specified");
        return false;
    }
    if(!nh_private_.getParam("interpolation/max_ang_a", max_ang_a_)) {
        ROS_WARN("[Simple planner] Parameter planner/max_ang_a not specified");
        return false;
    }
    if(!nh_private_.getParam("interpolation/sampling_dt", sampling_dt_)) {
        ROS_WARN("[Simple planner] Parameter planner/sampling_dt not specified");
        return false;
    }

    return true;
}

void Planner::initRos() {
    // Initialize subscribers

    // Initialize publishers
    command_pub_ = nh_.advertise<trajectory_msgs::MultiDOFJointTrajectory>(
        mav_msgs::default_topics::COMMAND_TRAJECTORY, 1);

    traj_markers_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("trajectory_markers",1);

    // Initialize services
    command_circle_srv_ = nh_.advertiseService("command_circle", &Planner::commandCircleCallback, this);
    command_init_srv_ = nh_.advertiseService("command_init", &Planner::commandInitCallback, this);
    command_point_srv_ = nh_.advertiseService("command_point", &Planner::commandPointCallback, this);
    command_coverage_srv_ = nh_.advertiseService("command_coverage", &Planner::commandCoverageCallback, this);

    // Initialize timers

    // Command publisher by sampling the previously interpolated trajectory
    publish_timer_ = nh_.createTimer(ros::Duration(dt_),
                                &Planner::commandTimerCallback, this,
                                false, false);
}

void Planner::startPlanning() {

  if (!poly_interpolator_->interpolate(path_waypoints_,Eigen::Vector3d(0.0,0.0,0.0), Eigen::Vector3d(0.0,0.0,0.0))) {
    ROS_ERROR("[Simple planner] Interpolator failed");
    return;
  } else {
    ROS_INFO("[Simple planner] Interpolation was successfull");
    trajectory_with_yaw_ = poly_interpolator_->getTrajectory();
  }

  current_sample_time_ = 0.0;
  //current_plan_id_ = ros::Time::now();

  // Publish the visualization of the trajectory
  if (traj_markers_pub_.getNumSubscribers() > 0) {
    visualization_msgs::MarkerArray traj_markers;
    double distance = 1.0;
    std::string frame_id = "world";

    mav_trajectory_generation::drawMavTrajectory(trajectory_with_yaw_, distance,
                                                 frame_id, &traj_markers);
    ROS_INFO("[Simple planner] Published trajectory markers");
    traj_markers_pub_.publish(traj_markers);
  }

  // Start timer
  ROS_INFO("[Simple planner] Starting timer...");
  publish_timer_.start();


  ROS_INFO("[Simple planner] Initialization complete!");
}

bool Planner::commandCircleCallback(mav_simple_planner::ServiceCommandCircleRequest& req, mav_simple_planner::ServiceCommandCircleResponse& res) {
    double center_x = req.x;
    double center_y = req.y;
    double center_z = req.z;
    double radius = req.radius;
    int num_points = req.num_points;
    
    const double yaw = 0;
    const double delta_angle = 2*M_PI/num_points;

    path_waypoints_.clear();

    for (int i = 0; i < num_points; ++i)
    {
        path_waypoints_.push_back(Eigen::Vector4d(
            center_x + std::cos(i*delta_angle) * radius, 
            center_y + std::sin(i*delta_angle) * radius, 
            center_z, 
            yaw));
    }

    path_waypoints_.push_back(Eigen::Vector4d(
            center_x + radius, 
            center_y, 
            center_z, 
            yaw));

    startPlanning();

    return true;
}

bool Planner::commandInitCallback(mav_simple_planner::ServiceCommandInitRequest& req, mav_simple_planner::ServiceCommandInitResponse& res) {
    double center_x = req.x;
    double center_y = req.y;
    double center_z = req.z;
    double distance = req.dist;

    const double yaw = 0;

    path_waypoints_.clear();

    path_waypoints_.push_back(Eigen::Vector4d(
        center_x, 
        center_y, 
        center_z, 
        yaw));

    path_waypoints_.push_back(Eigen::Vector4d(
        center_x, 
        center_y, 
        center_z + distance, 
        yaw));

    path_waypoints_.push_back(Eigen::Vector4d(
        center_x, 
        center_y, 
        center_z, 
        yaw));

    path_waypoints_.push_back(Eigen::Vector4d(
        center_x - distance, 
        center_y, 
        center_z, 
        yaw));

    path_waypoints_.push_back(Eigen::Vector4d(
        center_x,
        center_y,
        center_z,
        yaw));

    path_waypoints_.push_back(Eigen::Vector4d(
        center_x,
        center_y + distance,
        center_z,
        yaw));

    path_waypoints_.push_back(Eigen::Vector4d(
        center_x,
        center_y,
        center_z,
        yaw));

    startPlanning();

    return true;
}

bool Planner::commandPointCallback(mav_simple_planner::ServiceCommandPointRequest& req, mav_simple_planner::ServiceCommandPointResponse& res) {
    double center_x = req.x;
    double center_y = req.y;
    double center_z = req.z;
    
    const double yaw = 0;

    path_waypoints_.clear();

    path_waypoints_.push_back(Eigen::Vector4d(
        center_x,
        center_y,
        center_z, 
        yaw));
     
    startPlanning();

    return true;
}


bool Planner::commandCoverageCallback(mav_simple_planner::ServiceCommandCoverageRequest& req, mav_simple_planner::ServiceCommandCoverageResponse& res){
    double center_x = req.x;
    double center_y = req.y;
    double center_z = req.z;
    double side_x = req.side_x;
    double side_y = req.side_y;
    double interval = req.interval;
    int softness = 5;
    double inter_point = (2 * side_y) / softness;

    double yaw = 0;

    int num_paths = int(side_x / (interval));

    path_waypoints_.clear();

    for (size_t i = 0; i <= num_paths/2; i++)
    {

        for(size_t j=0; j<= softness; j++)
        {
            path_waypoints_.push_back(Eigen::Vector4d(
                center_x - side_x + interval * 2*i,
                center_y - side_y + j*inter_point,
                center_z,
                yaw));
        }

        for(size_t j=0; j<= softness; j++)
        {
            path_waypoints_.push_back(Eigen::Vector4d(
                center_x - side_x + interval * (2*i+1),
                center_y + side_y - j*inter_point,
                center_z,
                yaw));
        }
    }

    startPlanning();
    
}

void Planner::commandTimerCallback(const ros::TimerEvent&) {

    if (current_sample_time_ <= trajectory_with_yaw_.getMaxTime()) {
        trajectory_msgs::MultiDOFJointTrajectory msg;
        mav_msgs::EigenTrajectoryPoint trajectory_point;
        bool success = mav_trajectory_generation::sampleTrajectoryAtTime(
            trajectory_with_yaw_, current_sample_time_, &trajectory_point);
        
        // Stop commands if sampling fails!
        if (!success) {
            ROS_WARN("[Simple Planner] Stopping command timer! Could not sample trajectory");
            publish_timer_.stop();
            std_msgs::Time time_msg;
            time_msg.data.fromSec(0);
            //plan_id_pub_.publish(time_msg);
        }
        
        std_msgs::Time time_msg;
        time_msg.data = current_plan_id_;
        //plan_id_pub_.publish(time_msg);

        current_sample_time_ += dt_;
        mav_msgs::msgMultiDofJointTrajectoryFromEigen(trajectory_point, &msg);
        msg.points[0].time_from_start = ros::Duration(current_sample_time_);
        command_pub_.publish(msg);

    } else {
        // Trajectory finished
        ROS_WARN("[Simple Planner] Stopping command timer! Sampled all trajectory");
        publish_timer_.stop();
        std_msgs::Time time_msg;
        time_msg.data.fromSec(0);
        //plan_id_pub_.publish(time_msg);
    }
}


}