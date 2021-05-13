# sfm_local_controller 
A robot local controller based on the Social Force Model (SFM) (https://github.com/robotics-upo/lightsfm).
A set of attractive and repulsive forces determine the behavior of the planner. 

This local planner has been programmed as a BaseLocalPlanner plugin. So it can be used in the ROS move_base architecture. 
Although It receives a local costmap from move_base, the costmap is not used and a dedicated collision detector is employed instead. This detector uses laserscan input to check the possible collisions. 

![](https://github.com/robotics-upo/sfm_local_controller/blob/master/SFM.jpg)

## Parameters

* **Robot Configuration Parameters**
	- *max_lin_acc*. Maximum acceleration in translation (m/s^2).
  	- *max_rot_acc*. Maximum acceleration in rotation (rad/s^2).
  	- *max_lin_vel*. Maximum linear velocity (m/s).
  	- *min_lin_vel*. Minimum linear velocity (m/s).
  	- *max_rot_vel*. Maximum angular velocity (rad/s).
  	- *min_rot_vel*. Minimum angular velocity (rad/s).
  	- *min_rot_in_place*. Angular velocity of rotations in place (rad/s).
	- *a*. A factor that determines the angular velocity when the robot is turning. (Def: 10.0).
	- *planner_frame*. The frame in which the controller works. (Def: "odom").
	- *robot_frame*. The robot base frame. (Def: "base_link").
	- *robot_radius*. Radius (m) of the approximated circumference of the robot footprint. 
	- *people_radius*. Radius (m) of the approximated circumference that circumscribe a human footprint. (Def: 0.35).

* **Goal Tolerance Parameters**
	- *goal_tolerance*. Tolerance in euclidean distance (m) to consider that the goal has been reached.
	- *yaw_tolerance*. Tolerance in angular distance (rad) to consider that the goal has been reached.
	- *local_goal_dist*. Distance (m) from the robot to look for a local goal in the global path (Def: 1.0).
	
* **Sensor Interface Parameters**
The sensor interface is in charge of taking the sensory input and update the information of the surrounding social agents in the scene. 
	- *laser_topic*. Topic in which the laser range finder is being published [sensor_msgs/LaserScan].
	- *people_topic*. Topic in which the people detected are being published [people_msgs/People].
	- *dyn_obs_topic*. Topic in which the dynamic obstacles detected are being published [dynamic_obstacle_detector/DynamicObstacles].
	- *odom_topic*. Odometry topic [nav_msgs/Odometry].
	- *max_obstacle_dist*. Maximum distance (m) in which the obstacles are considered for the social force model.
	- *naive_goal_time*. Lookahead time (secs) to predict an approximate goal for the pedestrians.
	- *people_velocity*. Average velocity of the pedestrians (m/s). 

* **Social Force Model Parameters**
	- *sfm_goal_weight*. Weight of the attraction force to the goal.
	- *sfm_obstacle_weight*. Weight of the respulsive force of the obstacles.
	- *sfm_people_weight*. Weight of the respulsive force of the pedestrians.

* **Collision Checker Parameters**
	- *use_laser*. Boolean to indicate whether to use a laser scan for input. The scan topic indicated in the previous parameter *laser_topic* will be used. (Def: True).
	- *use_range*. Boolean to indicate whether to use sonar ranges for collision detection. If True, the topic names of the different sonars must be indicated by the parameters *range_topic_X* in which the *X* must be replaced by a number. eg: *range_topic_0*, range_topic_1*, etc. (Def: False).
	- *sensor_uncertainty*. Uncertainty or accurary of the sensory input (m). (Def: 0.05). 
	
## Publications

Besides the publications of the commands and paths according to the move_base structure. The planner publishes some RViz markers:

- */sfm/markers/goal* [visualization_msgs/Marker]. Arrow with the local goal of the controller.

- */sfm/markers/robot_forces* [visualization_msgs/MarkerArray]. Vectors with the different forces of the SFM.
	
## Dependencies

- dynamic_obstacle_detector (https://github.com/robotics-upo/dynamic_obstacle_detector).

- lightsfm (https://github.com/robotics-upo/lightsfm)


