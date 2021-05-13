/**
 * Implementation of the BaseLocalPlanner plugin
 * in order to use the sfm_local_controller in the
 * ROS move_base architecture.
 *
 * Author: Noé Pérez-Higueras
 * Service Robotics Lab, Pablo de Olavide University 2021
 *
 * Software License Agreement (BSD License)
 *
 */

#ifndef SFM_CONTROLLER_ROS_H_
#define SFM_CONTROLLER_ROS_H_

#include <costmap_2d/costmap_2d.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <ros/ros.h>
#include <sfm_local_controller/collision_checker.h>
#include <sfm_local_controller/sensor_interface.h>
#include <sfm_local_controller/sfm_controller.h>

#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>

#include <nav_core/base_local_planner.h>
#include <tf2_ros/buffer.h>

// Dynamic reconfigure
#include <dynamic_reconfigure/server.h>
#include <sfm_local_controller/SFMLocalControllerConfig.h>

#include <vector>

namespace Upo {
namespace Navigation {
namespace sfm_controller {
/**
 * @class SFMControllerROS
 * @brief A ROS wrapper for the sfm local controller that queries the param
 * server to construct a controller
 */
class SFMControllerROS : public nav_core::BaseLocalPlanner {
public:
  /**
   * @brief  Default constructor for the ros wrapper
   */
  SFMControllerROS();
  /**
   * @brief  Destructor for the wrapper
   */
  ~SFMControllerROS();
  /**
   * @brief  Constructs the ros wrapper
   * @param name The name to give this instance of the trajectory planner
   * @param tf A pointer to a transform listener
   * @param costmap The cost map to use for assigning costs to trajectories. NOT
   * USED
   */
  SFMControllerROS(std::string name, tf2_ros::Buffer *tf,
                   costmap_2d::Costmap2DROS *costmap_ros);
  /**
   * @brief  Constructs the ros wrapper
   * @param name The name to give this instance of the trajectory planner
   * @param tf A pointer to a transform listener
   * @param costmap The cost map to use for assigning costs to trajectories. NOT
   * USED
   */
  void initialize(std::string name, tf2_ros::Buffer *tf,
                  costmap_2d::Costmap2DROS *costmap_ros);
  /**
   * @brief  Given the current position, orientation, and velocity of the robot,
   * compute velocity commands to send to the base
   * @param cmd_vel Will be filled with the velocity command to be passed to the
   * robot base
   * @return True if a valid trajectory was found, false otherwise
   */
  bool computeVelocityCommands(geometry_msgs::Twist &cmd_vel);
  /**
   * @brief  Set the plan that the controller is following
   * @param orig_global_plan The plan to pass to the controller
   * @return True if the plan was updated successfully, false otherwise
   */
  bool setPlan(const std::vector<geometry_msgs::PoseStamped> &orig_global_plan);

  /**
   * @brief  Check if the goal pose has been achieved
   * @return True if achieved, false otherwise
   */
  bool isGoalReached();

  // void scanCb(const sensor_msgs::LaserScan::ConstPtr &laser);

private:
  /**
   * @brief Callback to update the local planner's parameters based on dynamic
   * reconfigure
   */
  void reconfigureCB(sfm_local_controller::SFMLocalControllerConfig &config,
                     uint32_t level);

  std::vector<geometry_msgs::PoseStamped>
  transformPlan(const std::vector<geometry_msgs::PoseStamped> &plan,
                std::string frame);

  // ROS parameters
  tf2_ros::Buffer *tf_buffer_;
  std::string planner_frame_;
  std::string robot_frame_;
  ros::Publisher g_plan_pub_, l_plan_pub_;

  // ros::Subscriber odom_sub_;
  // ros::Subscriber laser_sub_, people_sub_, dyn_obs_sub_, sonar_sub_;

  std::vector<geometry_msgs::PoseStamped> planner_path_;
  bool goal_reached_;

  // dynamic reconfigure
  dynamic_reconfigure::Server<sfm_local_controller::SFMLocalControllerConfig>
      *dsrv_;
  sfm_local_controller::SFMLocalControllerConfig default_config_;

  // Controller
  SFMController *sfm_controller_;
  SFMSensorInterface *sensor_iface_;
  CollisionChecker *collision_checker_;
  bool initialized_;
};

} // namespace sfm_controller
} // namespace Navigation
} // namespace Upo

#endif
