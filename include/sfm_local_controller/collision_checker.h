#ifndef COLLISION_CHECKER_H_
#define COLLISION_CHECKER_H_

#include <mutex>
#include <ros/ros.h>
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <sys/time.h>
#include <vector>

#include <geometry_msgs/Point.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Range.h>
#include <tf2/impl/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

namespace Upo {
namespace Navigation {
namespace sfm_controller {

class CollisionChecker {

public:
  struct range {
    std::string id;
    float range;
    float polar_angle;
    float polar_dist;
    float fov;
    float min_dist;
    float max_dist;
  };

  CollisionChecker(ros::NodeHandle *n, tf2_ros::Buffer *tf, double max_lv,
                   double max_av, double lin_acc, double ang_acc,
                   double r_radius, std::string base_frame,
                   std::string planner_frame);

  ~CollisionChecker();

  void setup();

  /**
   * @brief  Generate and check a single trajectory
   * @param cvx The current x velocity of the robot
   * @param cvy The current y velocity of the robot
   * @param cvth The current angular velocity of the robot
   * @param tvx The x velocity used to seed the trajectory
   * @param tvy The y velocity used to seed the trajectory
   * @param tvth The theta velocity used to seed the trajectory
   * @param dt The time for project the movement
   * @return True if the trajectory is legal, false otherwise
   */
  bool checkCommand(double cvx, double cvy, double cvth, double tvx, double tvy,
                    double tvth, double dt);

  // void saturateVelocities(geometry_msgs::Twist *twist);

  void setVelParams(double max_lin_vel, double max_vel_theta,
                    double max_lin_acc, double max_theta_acc) {
    params_mutex_.lock();
    max_lin_vel_ = max_lin_vel;
    max_ang_vel_ = max_vel_theta;
    max_lin_acc_ = max_lin_acc;
    max_ang_acc_ = max_theta_acc;
    params_mutex_.unlock();
  }

  void laserCallback(const sensor_msgs::LaserScan::ConstPtr &msg);

  std::vector<geometry_msgs::Point>
  laser_polar2euclidean(const sensor_msgs::LaserScan *scan);

  void transformPoints(std::vector<geometry_msgs::Point> &points,
                       std::string input_frame);

  void rangeCallback(const sensor_msgs::Range::ConstPtr &msg);
  void initiateRanges();

  bool inRangeCollision(float x, float y);
  bool inLaserCollision(float x, float y,
                        const std::vector<geometry_msgs::Point> &scanpoints);
  bool inCollision(float x, float y,
                   const std::vector<geometry_msgs::Point> &scanpoints);

private:
  float inline normalizeAngle(float val, float min, float max) {
    float norm = 0.0;
    if (val >= min)
      norm = min + fmod((val - min), (max - min));
    else
      norm = max - fmod((min - val), (max - min));

    return norm;
  }

  /**
   * @brief  Compute x position based on velocity
   * @param  xi The current x position
   * @param  vx The current x velocity
   * @param  vy The current y velocity
   * @param  theta The current orientation
   * @param  dt The timestep to take
   * @return The new x position
   */
  inline double computeNewXPosition(double xi, double vx, double vy,
                                    double theta, double dt) {
    return xi + (vx * cos(theta) + vy * cos(M_PI_2 + theta)) * dt;
  }

  /**
   * @brief  Compute y position based on velocity
   * @param  yi The current y position
   * @param  vx The current x velocity
   * @param  vy The current y velocity
   * @param  theta The current orientation
   * @param  dt The timestep to take
   * @return The new y position
   */
  inline double computeNewYPosition(double yi, double vx, double vy,
                                    double theta, double dt) {
    return yi + (vx * sin(theta) + vy * sin(M_PI_2 + theta)) * dt;
  }

  /**
   * @brief  Compute orientation based on velocity
   * @param  thetai The current orientation
   * @param  vth The current theta velocity
   * @param  dt The timestep to take
   * @return The new orientation
   */
  inline double computeNewThetaPosition(double thetai, double vth, double dt) {
    return thetai + vth * dt;
  }

  // compute velocity based on acceleration
  /**
   * @brief  Compute velocity based on acceleration
   * @param vg The desired velocity, what we're accelerating up to
   * @param vi The current velocity
   * @param a_max An acceleration limit
   * @param  dt The timestep to take
   * @return The new velocity
   */
  inline double computeNewVelocity(double vg, double vi, double a_max,
                                   double dt) {
    if ((vg - vi) >= 0) {
      return std::min(vg, vi + a_max * dt);
    }
    return std::max(vg, vi - a_max * dt);
  }

  // tf::TransformListener* tf_;
  tf2_ros::Buffer *tf_;

  ros::NodeHandle *nh_; // Pointer to the node node handle
  ros::NodeHandle n_;

  std::string odom_topic_;
  std::string robot_frame_;
  std::string planner_frame_;

  double max_lin_vel_;
  double max_ang_vel_;
  double max_lin_acc_;
  double max_ang_acc_;
  // double sim_time_;
  double robot_radius_;
  double robot_radius_aug_;
  // double local_radius_;
  // double granularity_;
  std::mutex params_mutex_;

  // float max_lv_var_;
  // float max_av_var_;

  // tf::Stamped<tf::Pose> robot_vel_;

  // in case a laser range finder is also used
  bool use_laser_;
  ros::Subscriber laser_sub_;
  // sensor_msgs::LaserScan laser_scan_;
  std::vector<geometry_msgs::Point> scanpoints_;
  std::mutex laser_mutex_;

  // Sonar ranges employed
  bool use_range_;
  int num_ranges_;
  std::vector<std::string> range_topics_;
  std::vector<std::string> range_frames_;
  std::vector<ros::Subscriber> range_subscribers_;
  // ros::Subscriber 				range_sub_;
  std::vector<range> ranges_;
  std::mutex range_mutex_;
  std::vector<bool> ranges_initiated_;
  bool ranges_ready_;
};

} // namespace sfm_controller
} // namespace Navigation
} // namespace Upo
#endif
