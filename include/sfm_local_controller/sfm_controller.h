/**
 * Local robot controller based on the Social Force Model
 *
 * Author: Noé Pérez-Higueras
 * Service Robotics Lab, Pablo de Olavide University 2021
 *
 * Software License Agreement (BSD License)
 *
 */

#ifndef SFM_CONTROLLER_H_
#define SFM_CONTROLLER_H_

#include <sfm_local_controller/SFMLocalControllerConfig.h>
#include <sfm_local_controller/collision_checker.h>
#include <sfm_local_controller/sensor_interface.h>

#include <geometry_msgs/Twist.h>
#include <std_msgs/ColorRGBA.h>
#include <visualization_msgs/MarkerArray.h>

#include <algorithm> // for copy() and assign()
#include <iterator>  // for back_inserter

namespace Upo {
namespace Navigation {
namespace sfm_controller {
class SFMController {
public:
  /**
   * @brief  Default constructor
   */
  SFMController();
  /**
   * @brief  Constructor with parameters
   */
  SFMController(ros::NodeHandle *n, tf2_ros::Buffer *tfBuffer,
                SFMSensorInterface *sensor_iface, CollisionChecker *coll_check,
                float max_lin_vel, float min_lin_vel, float max_vel_theta,
                float min_vel_theta, float min_rot_in_place, float max_lin_acc,
                float max_theta_acc, float local_goal_dist,
                float sfm_goal_weight, float sfm_obstacle_weight,
                float sfm_people_weight, float robot_radius,
                float person_radius, float goal_tolerance, float yaw_tolerance,
                std::string robot_frame, std::string controller_frame, float a);
  /**
   * @brief  Default destructor
   */
  ~SFMController();

  /**
   * @brief Callback to update the local planner's parameters based on dynamic
   * reconfigure
   */
  void reconfigure(sfm_local_controller::SFMLocalControllerConfig &cfg);

  /**
   * @brief method to update the scenario situation
   * @param path global path to be followed by the local controller
   * @return True when the goal has been reached, False otherwise
   */
  bool update(std::vector<geometry_msgs::PoseStamped> path);
  /**
   * @brief method to compute the velocity command of the robot
   * @param cmd_vel velocity message to be filled
   * @return True if a command vel was found
   */
  bool computeAction(geometry_msgs::Twist &cmd_vel);

  /**
   * @brief check if the current scenario leads to a possible collision
   * @return True if a possible collision was detected, False otherwise
   */
  bool fastCollisioncheck();

private:
  /**
   * @brief inline method of normalize an angle
   * @param val the angle in rads to be normalized
   * @param min the min angle allowed
   * @param max the max angle allowed
   * @return the normalized angle value
   */
  float inline normalizeAngle(float val, float min, float max) {
    float norm = 0.0;
    if (val >= min)
      norm = min + fmod((val - min), (max - min));
    else
      norm = max - fmod((min - val), (max - min));

    return norm;
  }
  /**
   * @brief Compute an angular vel to be sent to the robot
   * @param max maximum angular vel allowed
   * @param exp_const parameter of the function
   * @param var the angle between the robot heading and the goal heading
   * @return the angular vel
   */
  float inline getVel(float max, float exp_const, float var) {
    return max * (1 - exp(-exp_const * fabs(var))) * var / fabs(var);
  };
  float a_;

  /**
   * @brief Publish an arrow marker in Rviz representing a force
   * @param index id of the marker
   * @param color RGB color of the marker
   * @param force force to be represented
   * @param markers markerArray in which the arrow will be added
   * @return none
   */
  void publishForceMarker(unsigned index, const std_msgs::ColorRGBA &color,
                          const utils::Vector2d &force,
                          visualization_msgs::MarkerArray &markers);
  /**
   * @brief Publish the set of SFM forces in RViz
   * @return none
   */
  void publishForces();
  /**
   * @brief fill a ColorRGBA message
   * @param r value of the red componet [0-1]
   * @param g value of the green component [0-1]
   * @param b value of the blue component [0-1]
   * @param a transparency of the color [0-1]
   * @return a ROS ColorRGBA message
   */
  std_msgs::ColorRGBA getColor(double r, double g, double b, double a);

  /**
   * @brief Publish in RViz the local goal followed by the controller
   * @param g goal position
   * @return none
   */
  void publishSFMGoal(const geometry_msgs::PoseStamped &g);

  SFMSensorInterface *sensor_iface_;
  CollisionChecker *collision_checker_;

  std::mutex configuration_mutex_;

  std::string planner_frame_;
  std::string robot_frame_;
  ros::Publisher g_plan_pub_, l_plan_pub_, robot_markers_pub_,
      sfm_goal_pub_; //, vel_pub_;

  sfm::Agent robot_;
  std::vector<sfm::Agent> agents_;
  geometry_msgs::Pose goal_;
  std::vector<geometry_msgs::PoseStamped> planner_path_;
  bool goal_reached_;
  bool rotate_;

  ros::Time last_command_time_;

  // Robot configuration parameters
  float max_lin_vel_, min_lin_vel_;
  float max_vel_theta_, min_vel_theta_;
  float min_rot_in_place_;
  float max_lin_acc_;
  float max_theta_acc_;
  float robot_radius_;
  float person_radius_;
  float goal_tolerance_;
  float yaw_tolerance_;
  float local_goal_dist_;

  // SFM weights
  float sfm_goal_weight_;
  float sfm_obstacle_weight_;
  float sfm_people_weight_;
}; // namespace sfm_controller
} // namespace sfm_controller
} // namespace Navigation
} // namespace Upo

#endif