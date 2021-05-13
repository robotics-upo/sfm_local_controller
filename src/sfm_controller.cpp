/**
 * Local robot controller based on the Social Force Model
 *
 * Author: Noé Pérez-Higueras
 * Service Robotics Lab, Pablo de Olavide University 2021
 *
 * Software License Agreement (BSD License)
 *
 */

#include <sfm_local_controller/sfm_controller.h>

namespace Upo {
namespace Navigation {
namespace sfm_controller {

/**
 * @brief  Default constructor
 */
SFMController::SFMController()
    : max_lin_vel_(0.7), min_lin_vel_(0.1), max_vel_theta_(0.6),
      min_vel_theta_(0.1), min_rot_in_place_(0.4), max_lin_acc_(1.0),
      max_theta_acc_(1.0), local_goal_dist_(1.0), sfm_goal_weight_(2.0),
      sfm_obstacle_weight_(10.0), sfm_people_weight_(12.0), robot_radius_(0.35),
      person_radius_(0.35), goal_tolerance_(0.15), yaw_tolerance_(0.3),
      robot_frame_(std::string("base_link")),
      planner_frame_(std::string("odom")), a_(2.0) {}

/**
 * @brief  Constructor with parameters
 */
SFMController::SFMController(
    ros::NodeHandle *n, tf2_ros::Buffer *tfBuffer,
    SFMSensorInterface *sensor_iface, CollisionChecker *coll_check,
    float max_lin_vel, float min_lin_vel, float max_vel_theta,
    float min_vel_theta, float min_rot_in_place, float max_lin_acc,
    float max_theta_acc, float local_goal_dist, float sfm_goal_weight,
    float sfm_obstacle_weight, float sfm_people_weight, float robot_radius,
    float person_radius, float goal_tolerance, float yaw_tolerance,
    std::string robot_frame, std::string controller_frame, float a)
    : max_lin_vel_(max_lin_vel), min_lin_vel_(min_lin_vel),
      max_vel_theta_(max_vel_theta), min_vel_theta_(min_vel_theta),
      min_rot_in_place_(min_rot_in_place), max_lin_acc_(max_lin_acc),
      max_theta_acc_(max_theta_acc), local_goal_dist_(local_goal_dist),
      sfm_goal_weight_(sfm_goal_weight),
      sfm_obstacle_weight_(sfm_obstacle_weight),
      sfm_people_weight_(sfm_people_weight), robot_radius_(robot_radius),
      person_radius_(person_radius), goal_tolerance_(goal_tolerance),
      yaw_tolerance_(yaw_tolerance), robot_frame_(robot_frame),
      planner_frame_(controller_frame), a_(a) {
  // Initialize robot agent
  robot_.desiredVelocity = max_lin_vel_;
  robot_.radius = robot_radius_;
  robot_.cyclicGoals = false;
  robot_.teleoperated = true;
  robot_.params.forceFactorDesired = sfm_goal_weight_;
  robot_.params.forceFactorObstacle = sfm_obstacle_weight_;
  robot_.params.forceFactorSocial = sfm_people_weight_;

  std::cout << std::endl
            << "SFM_CONTROLLER INITIATED:" << std::endl
            << "max_lin_acc: " << max_lin_acc_ << std::endl
            << "max_rot_acc: " << max_theta_acc_ << std::endl
            << "min_lin_vel: " << min_lin_vel_ << std::endl
            << "max_lin_vel: " << max_lin_vel_ << std::endl
            << "min_rot_vel: " << min_vel_theta_ << std::endl
            << "max_rot_vel: " << max_vel_theta_ << std::endl
            << "local_goal_dist: " << local_goal_dist_ << std::endl
            << "goal_tolerance: " << goal_tolerance_ << std::endl
            << "robot_radius: " << robot_radius_ << std::endl
            << "person_radius: " << person_radius_ << std::endl
            << "robot_frame: " << robot_frame_ << std::endl
            << "planner_frame: " << planner_frame_ << std::endl
            << "sfm_goal_weight: " << sfm_goal_weight_ << std::endl
            << "sfm_obstacle_weight: " << sfm_obstacle_weight_ << std::endl
            << "sfm_social_weight: " << sfm_people_weight_ << std::endl
            << std::endl;

  // Advertise SFM related markers
  robot_markers_pub_ = n->advertise<visualization_msgs::MarkerArray>(
      "/sfm/markers/robot_forces", 1);

  // Adverstise SFM local goal
  sfm_goal_pub_ =
      n->advertise<visualization_msgs::Marker>("/sfm/markers/goal", 1);

  // Initialize sensor interface
  sensor_iface_ = sensor_iface;

  // Initialize collision checker
  collision_checker_ = coll_check;

  last_command_time_ = ros::Time::now();
  goal_reached_ = false;
  rotate_ = false;
}

/**
 * @brief  Default destructor
 */
SFMController::~SFMController() {
  delete sensor_iface_;
  delete collision_checker_;
}

/**
 * @brief Callback to update the local planner's parameters based on dynamic
 * reconfigure
 */
void SFMController::reconfigure(
    sfm_local_controller::SFMLocalControllerConfig &cfg) {
  sfm_local_controller::SFMLocalControllerConfig config(cfg);

  configuration_mutex_.lock();

  max_lin_acc_ = config.max_lin_acc;
  max_theta_acc_ = config.max_rot_acc;
  max_lin_vel_ = config.max_lin_vel;
  robot_.desiredVelocity = max_lin_vel_;
  min_lin_vel_ = config.min_lin_vel;
  max_vel_theta_ = config.max_rot_vel;
  min_vel_theta_ = config.min_rot_vel;

  sfm_goal_weight_ = config.sfm_goal_weight;
  sfm_obstacle_weight_ = config.sfm_obstacle_weight;
  sfm_people_weight_ = config.sfm_people_weight;
  robot_.params.forceFactorDesired = sfm_goal_weight_;
  robot_.params.forceFactorObstacle = sfm_obstacle_weight_;
  robot_.params.forceFactorSocial = sfm_people_weight_;

  // min_in_place_vel_th_ = config.min_in_place_rot_vel;
  // goal_lin_tolerance_ = config.xy_goal_tolerance;
  // goal_ang_tolerance_ = config.yaw_goal_tolerance;
  // wp_tolerance_ = config.wp_tolerance;

  sensor_iface_->setMaxLinVel(max_lin_vel_);
  collision_checker_->setVelParams(max_lin_vel_, max_vel_theta_, max_lin_acc_,
                                   max_theta_acc_);

  std::cout << std::endl
            << "SFM_CONTROLLER RECONFIGURED:" << std::endl
            << "max_lin_acc: " << max_lin_acc_ << std::endl
            << "max_rot_acc: " << max_theta_acc_ << std::endl
            << "min_lin_vel: " << min_lin_vel_ << std::endl
            << "max_lin_vel: " << max_lin_vel_ << std::endl
            << "min_rot_vel: " << min_vel_theta_ << std::endl
            << "max_rot_vel: " << max_vel_theta_ << std::endl
            << "sfm_goal_weight: " << sfm_goal_weight_ << std::endl
            << "sfm_obstacle_weight: " << sfm_obstacle_weight_ << std::endl
            << "sfm_social_weight: " << sfm_people_weight_ << std::endl
            << std::endl;

  configuration_mutex_.unlock();
}

/**
 * @brief method to update the scenario situation
 * @param path global path to be followed by the local controller
 * @return True when the goal has been reached, False otherwise
 */
bool SFMController::update(std::vector<geometry_msgs::PoseStamped> path) {

  std::vector<sfm::Agent> agents = sensor_iface_->getAgents();

  configuration_mutex_.lock();
  // update robot
  robot_.position = agents[0].position;
  robot_.yaw = agents[0].yaw;
  robot_.linearVelocity = agents[0].linearVelocity;
  robot_.angularVelocity = agents[0].angularVelocity;
  robot_.velocity = agents[0].velocity;
  robot_.obstacles1.clear();
  robot_.obstacles1 = agents[0].obstacles1;

  // Update the rest of agents
  agents_.clear();
  if (!agents.empty())
    agents_.assign(++agents.begin(), agents.end());

  // If we have to rotate, we do not look for a new goal
  if (rotate_) {
    sfm::Goal g;
    g.center.set(goal_.position.x, goal_.position.y);
    g.radius = goal_tolerance_;
    robot_.goals.clear();
    robot_.goals.push_back(g);
    if (goal_reached_) {
      goal_reached_ = false;
      rotate_ = false;
      configuration_mutex_.unlock();
      return true;
    }
    configuration_mutex_.unlock();
    return false;
  }

  // Update the robot goal
  geometry_msgs::PoseStamped min;
  float min_dist = 9999.0;
  bool goal_found = false;
  for (unsigned int i = path.size() - 1; i > 0; i--) {
    float dx = path[i].pose.position.x - robot_.position.getX();
    float dy = path[i].pose.position.y - robot_.position.getY();
    float d = sqrt(dx * dx + dy * dy);
    if (d < min_dist) {
      min_dist = d;
      min = path[i];
    }
    if (d <= local_goal_dist_) {
      if (d < goal_tolerance_) {
        configuration_mutex_.unlock();
        // goal reached! - rotate!
        printf("Update. Goal location reached. Rotating...\n");
        rotate_ = true;
        return false;
      }
      sfm::Goal g;
      goal_ = path[i].pose;
      g.center.set(path[i].pose.position.x, path[i].pose.position.y);
      g.radius = goal_tolerance_;
      robot_.goals.clear();
      robot_.goals.push_back(g);
      goal_found = true;
      goal_reached_ = false;
      // printf("Update. Goal found! x: %.2f, y: %.2f\n", g.center.getX(),
      //       g.center.getY());
      publishSFMGoal(path[i]);
      configuration_mutex_.unlock();
      return false;
    }
  }
  // printf("Goal not found closer than %.2f,\n", local_goal_dist_);
  // printf("the closest is %.2f m\n", min_dist);
  if (!path.empty()) {
    sfm::Goal g;
    goal_ = min.pose;
    g.center.set(min.pose.position.x, min.pose.position.y);
    g.radius = goal_tolerance_;
    robot_.goals.clear();
    robot_.goals.push_back(g);
    goal_reached_ = false;
    publishSFMGoal(min);
  } else {
    printf("Update. Goal not found. Received path size: %i\n",
           (int)path.size());
  }
  configuration_mutex_.unlock();
  return false;
}

/**
 * @brief method to compute the velocity command of the robot
 * @param cmd_vel velocity message to be filled
 * @return True if a command vel was found
 */
bool SFMController::computeAction(geometry_msgs::Twist &cmd_vel) {

  configuration_mutex_.lock();
  double dt = (ros::Time::now() - last_command_time_).toSec();
  // printf("dt: %.4f\n", dt);
  if (dt > 0.2)
    dt = 0.1;

  last_command_time_ = ros::Time::now();

  // We must rotate to reach the goal position
  if (rotate_) {
    float ang_diff = robot_.yaw.toRadian() - tf::getYaw(goal_.orientation);
    ang_diff = normalizeAngle(ang_diff, -M_PI, M_PI);
    if (fabs(ang_diff) < yaw_tolerance_) {
      printf("Angdiff (%.2f) < yaw_tolerance (%.2f)\n", fabs(ang_diff),
             yaw_tolerance_);
      cmd_vel.linear.x = 0.0;
      cmd_vel.linear.y = 0.0;
      cmd_vel.linear.z = 0.0;
      cmd_vel.angular.x = 0.0;
      cmd_vel.angular.y = 0.0;
      cmd_vel.angular.z = 0.0;
      goal_reached_ = true;
      configuration_mutex_.unlock();
      return true;
    } else if (ang_diff > 0.0) {
      cmd_vel.angular.z = -min_rot_in_place_;
      // printf("Rotating with vel: %.2f\n", cmd_vel.angular.z);
    } else {
      cmd_vel.angular.z = min_rot_in_place_;
      // printf("Rotating with vel: %.2f\n", cmd_vel.angular.z);
    }
    configuration_mutex_.unlock();
    return true;
  }

  // Compute Social Forces
  sfm::SFM.computeForces(robot_, agents_);

  // Compute velocity of the robot
  robot_.velocity += robot_.forces.globalForce * dt;
  if (robot_.velocity.norm() > robot_.desiredVelocity) {
    robot_.velocity.normalize();
    robot_.velocity *= robot_.desiredVelocity;
  }

  // The resultant total velocity is expressed in the odom frame. Transform
  // to robot_frame
  geometry_msgs::Vector3 velocity;
  velocity.x = robot_.velocity.getX();
  velocity.y = robot_.velocity.getY();
  geometry_msgs::Vector3 localV =
      sensor_iface_->transformVector(velocity, planner_frame_, robot_frame_);

  utils::Vector2d vel;
  vel.set(localV.x, localV.y);
  cmd_vel.linear.x = vel.norm();

  // Decrease speed to approach the goal
  float dx = goal_.position.x - robot_.position.getX();
  float dy = goal_.position.y - robot_.position.getY();
  float d = sqrt(dx * dx + dy * dy);
  if (d < 1.0)
    cmd_vel.linear.x =
        (vel.norm() * d) < min_lin_vel_ ? min_lin_vel_ : (vel.norm() * d);

  cmd_vel.linear.y = 0.0;
  cmd_vel.linear.z = 0.0;
  // Wz = std::atan2(localV.y,localV.x)/2.0;
  double angle = std::atan2(localV.y, localV.x);
  cmd_vel.angular.z = getVel(max_vel_theta_, a_, angle);
  cmd_vel.angular.x = 0.0;
  cmd_vel.angular.y = 0.0;

  // Prevent the robot for turning around:
  // If the angle difference between the desired force
  // and the global force is almost opossite,
  // stop the robot instead of turning around
  double angle_deg =
      robot_.forces.desiredForce.angleTo(robot_.forces.globalForce).toDegree();
  if ((180.0 - fabs(angle_deg)) < 25.0) {
    // printf("\nStopping robot. angle_deg: %.3f!!!!!\n", angle_deg);
    cmd_vel.linear.x = 0.0;
    cmd_vel.angular.z = 0.0;
  }

  publishForces();
  if (!collision_checker_->checkCommand(
          robot_.linearVelocity, 0.0, robot_.angularVelocity, cmd_vel.linear.x,
          0.0, cmd_vel.angular.z, 0.11)) {
    ROS_WARN("Possible collision detected! Sending zero vel!");
    cmd_vel.linear.x = 0.0;
    cmd_vel.angular.z = 0.0;
  }

  // ROS_INFO("LV: %f; AV: %f", cmd_vel.linear.x, cmd_vel.angular.z);
  configuration_mutex_.unlock();
  return true;
}

/**
 * @brief check if the current scenario leads to a possible collision
 * @return True if a possible collision was detected, False otherwise
 */
bool SFMController::fastCollisioncheck() {
  // return sensor_iface_->collisionCheck();
  return true;
}

/**
 * @brief Publish in RViz the local goal followed by the controller
 * @param g goal position
 * @return none
 */
void SFMController::publishSFMGoal(const geometry_msgs::PoseStamped &g) {
  visualization_msgs::Marker marker;
  marker.header.frame_id = g.header.frame_id;
  marker.header.stamp = ros::Time::now();
  marker.ns = "sfm_goal";
  marker.id = 1;
  marker.action = visualization_msgs::Marker::ADD;
  marker.type = visualization_msgs::Marker::ARROW;
  marker.color = getColor(0.0, 0.0, 1.0, 1.0);
  marker.lifetime = ros::Duration(0.15);
  marker.scale.x = 0.4;
  marker.scale.y = 0.1;
  marker.scale.z = 0.1;
  marker.pose = g.pose;
  sfm_goal_pub_.publish(marker);
}

/**
 * @brief Publish an arrow marker in Rviz representing a force
 * @param index id of the marker
 * @param color RGB color of the marker
 * @param force force to be represented
 * @param markers markerArray in which the arrow will be added
 * @return none
 */
void SFMController::publishForceMarker(
    unsigned index, const std_msgs::ColorRGBA &color,
    const utils::Vector2d &force, visualization_msgs::MarkerArray &markers) {
  visualization_msgs::Marker marker;
  marker.header.frame_id = planner_frame_;
  marker.header.stamp = ros::Time::now();
  marker.ns = "robot_forces";
  marker.id = index;
  marker.action = force.norm() > 1e-4 ? 0 : 2;
  marker.color = color;
  marker.lifetime = ros::Duration(1.0);
  marker.scale.x = std::max(1e-4, force.norm());
  marker.scale.y = 0.1;
  marker.scale.z = 0.1;
  marker.pose.position.x = robot_.position.getX();
  marker.pose.position.y = robot_.position.getY();
  marker.pose.position.z = 0;
  marker.pose.orientation =
      tf::createQuaternionMsgFromRollPitchYaw(0, 0, force.angle().toRadian());
  markers.markers.push_back(marker);
}

/**
 * @brief fill a ColorRGBA message
 * @param r value of the red componet [0-1]
 * @param g value of the green component [0-1]
 * @param b value of the blue component [0-1]
 * @param a transparency of the color [0-1]
 * @return a ROS ColorRGBA message
 */
std_msgs::ColorRGBA SFMController::getColor(double r, double g, double b,
                                            double a) {
  std_msgs::ColorRGBA color;
  color.r = r;
  color.g = g;
  color.b = b;
  color.a = a;
  return color;
}

/**
 * @brief Publish the set of SFM forces in RViz
 * @return none
 */
void SFMController::publishForces() {
  visualization_msgs::MarkerArray markers;
  publishForceMarker(0, getColor(1, 0, 0, 1), robot_.forces.obstacleForce,
                     markers);
  publishForceMarker(1, getColor(0, 0, 1, 1), robot_.forces.socialForce,
                     markers);
  // publishForceMarker(2, getColor(0, 1, 1, 1), robot_.forces.groupForce,
  //                   markers);
  publishForceMarker(3, getColor(0, 1, 0, 1), robot_.forces.desiredForce,
                     markers);
  publishForceMarker(4, getColor(1, 1, 1, 1), robot_.forces.globalForce,
                     markers);
  publishForceMarker(5, getColor(1, 1, 0, 1), robot_.velocity, markers);
  robot_markers_pub_.publish(markers);
  // ROS_INFO_STREAM("Goal: " << robot_.forces.desiredForce.norm()
  //                         << ", Obstacle: "
  //                         << robot_.forces.obstacleForce.norm() << ", People:
  //                         "
  //                         << robot_.forces.socialForce.norm() << std::endl);
}

} // namespace sfm_controller
} // namespace Navigation
} // namespace Upo