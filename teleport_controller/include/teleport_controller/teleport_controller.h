#ifndef TELEPORT_CONTROLLER_H_
#define TELEPORT_CONTROLLER_H_

#include <ros/ros.h>
#include <costmap_2d/costmap_2d.h>
#include <costmap_2d/costmap_2d_publisher.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>
#include <tf/transform_listener.h>
#include <boost/thread.hpp>
#include <dynamic_reconfigure/server.h>
#include <move_humans/controller_interface.h>
#include <hanp_msgs/TrackedHumans.h>
#include <teleport_controller/TeleportControllerConfig.h>
#include <hanp_msgs/PathArray.h>

namespace teleport_controller {

class TeleportController : public move_humans::ControllerInterface {
public:
  TeleportController();
  TeleportController(std::string name, tf::TransformListener *tf,
                     costmap_2d::Costmap2DROS *costmap_ros);
  ~TeleportController();

  void initialize(std::string name, tf::TransformListener *tf,
                  costmap_2d::Costmap2DROS *costmap_ros);

  bool setPlans(const move_humans::map_pose_vector &plans);

  bool computeHumansStates(move_humans::map_pose &humans);

  bool areGoalsReached();

  bool isInitialized() { return initialized_; }

private:
  bool initialized_;
  bool setup_;

  void reconfigureCB(TeleportControllerConfig &config, uint32_t level);

  costmap_2d::Costmap2DROS *costmap_ros_;
  tf::TransformListener *tf_;

  ros::Subscriber controller_plans_sub_;

  ros::Publisher plans_pub_, humans_pub_, humans_poses_pub_;
  bool visualize_human_poses_;

  move_humans::map_pose_vector plans_;
  move_humans::map_pose last_human_poses_;
  move_humans::id_vector reached_goals_;
  double max_linear_vel_, sq_dist_threshold_, goal_reached_threshold_;
  std::string controller_frame_;

  boost::mutex controlling_mutex_, configuration_mutex_;

  dynamic_reconfigure::Server<TeleportControllerConfig> *dsrv_;
  teleport_controller::TeleportControllerConfig default_config_;

  bool transformPlans(const move_humans::map_pose_vector &plans,
                      const move_humans::map_pose &current_poses,
                      move_humans::map_pose_vector &transformed_plans);
  unsigned int prunePlan(const move_humans::pose_vector &plan,
                         const tf::Stamped<tf::Pose> &cp_tf);
  double dist_sq(const geometry_msgs::Pose &pose1,
                 const geometry_msgs::Pose &pose2);

  void publishPlans(move_humans::map_pose_vector &plans);
  void publishHumans(move_humans::map_pose &human_poses);

  void controllerPlansCB(const hanp_msgs::PathArray &paths);
};
}; // namespace move_humans

#endif // TELEPORT_CONTROLLER_H_
