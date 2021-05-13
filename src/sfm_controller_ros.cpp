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

#include <sfm_local_controller/sfm_controller_ros.h>

//#include <sys/time.h>

//#include <ros/console.h>

#include <nav_msgs/Path.h>
#include <pluginlib/class_list_macros.h>

// register this planner as a BaseLocalPlanner plugin
PLUGINLIB_EXPORT_CLASS(Upo::Navigation::sfm_controller::SFMControllerROS,
                       nav_core::BaseLocalPlanner)

namespace Upo {
namespace Navigation {
namespace sfm_controller {

void SFMControllerROS::reconfigureCB(
    sfm_local_controller::SFMLocalControllerConfig &config, uint32_t level) {

  if (initialized_)
    sfm_controller_->reconfigure(config);
}

SFMControllerROS::SFMControllerROS()
    : initialized_(false), sfm_controller_(nullptr) {}

SFMControllerROS::SFMControllerROS(std::string name, tf2_ros::Buffer *tf,
                                   costmap_2d::Costmap2DROS *costmap_ros)
    : initialized_(false), sfm_controller_(nullptr) {
  // initialize the planner
  initialize(name, tf, costmap_ros);
}

void SFMControllerROS::initialize(std::string name, tf2_ros::Buffer *tf,
                                  costmap_2d::Costmap2DROS *costmap_ros) {
  if (!initialized_) {
    ros::NodeHandle private_nh("~/" + name);
    g_plan_pub_ = private_nh.advertise<nav_msgs::Path>("global_plan", 1);
    l_plan_pub_ = private_nh.advertise<nav_msgs::Path>("local_plan", 1);

    tf_buffer_ = tf;

    // Frames
    private_nh.param("robot_frame", robot_frame_, std::string("base_link"));
    private_nh.param("planner_frame", planner_frame_, std::string("odom"));

    // Robot Configuration Parameters
    float max_lin_vel_, min_lin_vel_;
    float max_vel_theta_, min_vel_theta_;
    float min_rot_in_place_;
    float max_lin_acc_;
    float max_theta_acc_;
    private_nh.param("max_lin_vel", max_lin_vel_, float(0.6));
    private_nh.param("min_lin_vel", min_lin_vel_, float(0.1));
    private_nh.param("max_rot_vel", max_vel_theta_, float(0.5));
    private_nh.param("min_rot_vel", min_vel_theta_, float(0.1));
    private_nh.param("min_rot_in_place", min_rot_in_place_, float(0.35));
    private_nh.param("max_lin_acc", max_lin_acc_, float(1.0));
    private_nh.param("max_rot_acc", max_theta_acc_, float(1.0));

    // Dimensions
    float robot_radius;
    float people_radius;
    private_nh.param("robot_radius", robot_radius, float(0.35));
    private_nh.param("people_radius", people_radius, float(0.35));
    float robot_goal_dist;
    private_nh.param("local_goal_dist", robot_goal_dist, float(1.0));

    float a;
    private_nh.param("a", a, float(3.0));

    // Goal tolerance parameters
    float goal_tolerance, yaw_tolerance;
    private_nh.param("goal_tolerance", goal_tolerance, float(0.2));
    private_nh.param("yaw_tolerance", yaw_tolerance, float(0.35));
    // private_nh.param("xy_goal_tolerance", xy_goal_tolerance_, 0.10);
    // private_nh.param("wp_tolerance", wp_tolerance_, 0.5);

    // SFM weights
    float sfm_goal_weight_;
    float sfm_obstacle_weight_;
    float sfm_people_weight_;
    private_nh.param("sfm_goal_weight", sfm_goal_weight_, float(2.0));
    private_nh.param("sfm_obstacle_weight", sfm_obstacle_weight_, float(20.0));
    private_nh.param("sfm_people_weight", sfm_people_weight_, float(12.0));

    // std::string laser_topic;
    // private_nh.param("laser_topic", laser_topic, std::string("scan"));
    // std::string people_topic;
    // private_nh.param("people_topic", people_topic, std::string("people"));
    // std::string obs_topic;
    // private_nh.param("dyn_obs_topic", obs_topic, std::string("obstacles"));
    // std::string odom_topic;
    // private_nh.param("odom_topic", odom_topic, std::string("odom"));

    sensor_iface_ =
        new SFMSensorInterface(&private_nh, tf, max_lin_vel_, robot_radius,
                               people_radius, robot_frame_, planner_frame_);

    collision_checker_ = new CollisionChecker(
        &private_nh, tf, max_lin_vel_, max_vel_theta_, max_lin_acc_,
        max_theta_acc_, robot_radius, robot_frame_, planner_frame_);

    // ros::NodeHandle n;
    // laser_sub_ = n.subscribe<sensor_msgs::LaserScan>(
    //     laser_topic.c_str(), 1, &SFMSensorInterface::laserCb, sensor_iface_);

    // people_sub_ = n.subscribe<people_msgs::People>(
    //     people_topic.c_str(), 1, &SFMSensorInterface::peopleCb,
    //     sensor_iface_);

    // dyn_obs_sub_ = n.subscribe<dynamic_obstacle_detector::DynamicObstacles>(
    //     obs_topic.c_str(), 1, &SFMSensorInterface::dynamicObsCb,
    //     sensor_iface_);

    // odom_sub_ = n.subscribe<nav_msgs::Odometry>(
    //     odom_topic.c_str(), 1, &SFMSensorInterface::odomCb, sensor_iface_);

    // std::cout << std::endl
    //           << "SFM SENSOR INTERFACE:" << std::endl
    //           << "laser_topic: " << laser_topic << std::endl
    //           << "people_topic: " << people_topic << std::endl
    //           << "dyn_obs_topic: " << obs_topic << std::endl
    //           << "odom_topic: " << odom_topic
    //           << std::endl
    //           //<< "max_obstacle_dist: " << max_obstacle_dist_ << std::endl
    //           //<< "naive_goal_time: " << naive_goal_time_ << std::endl
    //           //<< "people_velocity: " << people_velocity_ << std::endl
    //           << std::endl;

    sfm_controller_ = new SFMController(
        &private_nh, tf, sensor_iface_, collision_checker_, max_lin_vel_,
        min_lin_vel_, max_vel_theta_, min_vel_theta_, min_rot_in_place_,
        max_lin_acc_, max_theta_acc_, robot_goal_dist, sfm_goal_weight_,
        sfm_obstacle_weight_, sfm_people_weight_, robot_radius, people_radius,
        goal_tolerance, yaw_tolerance, robot_frame_, planner_frame_, a);

    // BE CAREFUL, this will load the values of cfg params
    // overwritting the read ones from the yaml file (if planner already
    // initialized).
    dsrv_ = new dynamic_reconfigure::Server<
        sfm_local_controller::SFMLocalControllerConfig>(private_nh);
    dynamic_reconfigure::Server<
        sfm_local_controller::SFMLocalControllerConfig>::CallbackType cb =
        boost::bind(&SFMControllerROS::reconfigureCB, this, _1, _2);
    dsrv_->setCallback(cb);

    initialized_ = true;
    goal_reached_ = false;

  } else {
    ROS_WARN("This planner has already been initialized, doing nothing");
  }
}

SFMControllerROS::~SFMControllerROS() {
  // make sure to clean things up
  delete dsrv_;

  if (sfm_controller_ != nullptr)
    delete sfm_controller_;
}

bool SFMControllerROS::isGoalReached() { return goal_reached_; }

bool SFMControllerROS::setPlan(
    const std::vector<geometry_msgs::PoseStamped> &orig_global_plan) {
  if (!initialized_) {
    ROS_ERROR("This planner has not been initialized, please call initialize() "
              "before using this local planner");
    return false;
  }

  planner_path_.clear();
  // Transform plan to planner_frame_ if necessary
  if (!orig_global_plan.empty()) {
    // if (orig_global_plan[0].header.frame_id != planner_frame_) {
    //  planner_path_ = transformPlan(orig_global_plan, planner_frame_);
    //} else {
    planner_path_ = orig_global_plan;
    // planner_path_ = transformPlan(planner_path_, planner_frame_);
    //}
  }
  // reset the goal flag
  goal_reached_ = false;

  return true;
}

std::vector<geometry_msgs::PoseStamped> SFMControllerROS::transformPlan(
    const std::vector<geometry_msgs::PoseStamped> &plan, std::string frame) {
  std::vector<geometry_msgs::PoseStamped> planner_plan;
  for (unsigned int i = 0; i < plan.size(); i++) {
    try {
      geometry_msgs::PoseStamped pose = tf_buffer_->transform(plan[i], frame);
      planner_plan.push_back(pose);
    } catch (tf2::TransformException &ex) {
      ROS_WARN("Could NOT transform plan pose to %s: %s", frame.c_str(),
               ex.what());
      break;
    }
  }
  return planner_plan;
}

bool SFMControllerROS::computeVelocityCommands(geometry_msgs::Twist &cmd_vel) {
  if (!initialized_) {
    ROS_ERROR("This planner has not been initialized, please call initialize() "
              "before using this planner");
    return false;
  }

  // if the global plan passed in is empty... we won't do anything
  if (planner_path_.empty()) {
    ROS_WARN("Planner path is empty. Not computing cmd vels!!!!");
    return false;
  }

  planner_path_ = transformPlan(planner_path_, planner_frame_);
  goal_reached_ = sfm_controller_->update(planner_path_);

  geometry_msgs::Twist drive_cmds;
  if (goal_reached_) {
    ROS_INFO("GOAL REACHED!\n\n");
    cmd_vel = drive_cmds;
    cmd_vel.linear.x = 0.0;
    cmd_vel.linear.y = 0.0;
    cmd_vel.angular.z = 0.0;
    return true;
  }
  // compute what trajectory to drive along
  bool ok = sfm_controller_->computeAction(drive_cmds);
  // pass along drive commands
  cmd_vel = drive_cmds;
  if (!ok) {
    ROS_DEBUG_NAMED("trajectory_planner_ros",
                    "The rollout planner failed to find a valid plan. This "
                    "means that the footprint of the robot was in collision "
                    "for all simulated trajectories.");
    // publishPlan(transformed_plan, g_plan_pub_);
    return false;
  }
  // publish information to the visualizer
  // publishPlan(transformed_plan, g_plan_pub_);

  return true;
}

} // namespace sfm_controller
} // namespace Navigation
} // namespace Upo