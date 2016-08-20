#ifndef TELEPORT_CONTROLLER_H_
#define TELEPORT_CONTROLLER_H_

#include <ros/ros.h>
#include <costmap_2d/costmap_2d.h>
#include <costmap_2d/costmap_2d_publisher.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>
#include <tf/transform_listener.h>
#include <boost/thread.hpp>
#include <dynamic_reconfigure/server.h>
#include <move_humans/controller_interface.h>
#include <hanp_msgs/TrackedHumans.h>
#include <teleport_controller/TeleportControllerConfig.h>

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

  move_humans::map_pose_vector global_plans_;

  boost::mutex configuration_mutex_;

  ros::Publisher plan_pub_, cotrol_pub_;

  dynamic_reconfigure::Server<TeleportControllerConfig> *dsrv_;
  teleport_controller::TeleportControllerConfig default_config_;
};
}; // namespace move_humans

#endif // TELEPORT_CONTROLLER_H_
