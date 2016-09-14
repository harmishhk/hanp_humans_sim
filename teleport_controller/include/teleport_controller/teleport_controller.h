#ifndef TELEPORT_CONTROLLER_H_
#define TELEPORT_CONTROLLER_H_

#include <ros/ros.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <tf/transform_listener.h>
#include <dynamic_reconfigure/server.h>
#include <hanp_msgs/HumanPathArray.h>
#include <boost/thread.hpp>
#include <move_humans/controller_interface.h>

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
  bool setPlans(const move_humans::map_pose_vector &plans,
                const move_humans::map_trajectory &twists);

  bool computeHumansStates(move_humans::map_traj_point &humans);

  bool areGoalsReached(move_humans::id_vector &reached_humans);

  bool isInitialized() { return initialized_; }

private:
  bool initialized_;
  bool setup_;

  void reconfigureCB(TeleportControllerConfig &config, uint32_t level);

  costmap_2d::Costmap2DROS *costmap_ros_;
  tf::TransformListener *tf_;

  ros::Publisher plans_pub_;

  move_humans::map_pose_vector plans_;
  move_humans::map_trajectory trajs_;
  move_humans::map_traj_point last_traj_points_;
  move_humans::map_size last_traversed_indices_;
  move_humans::map_trajectory last_transformed_trajs_;
  move_humans::id_vector reached_goals_;
  double max_linear_vel_, max_angular_vel_, max_linear_acc_, max_angular_acc_,
      sq_dist_threshold_, goal_reached_threshold_;
  std::string controller_frame_;

  boost::mutex controlling_mutex_, configuration_mutex_;

  dynamic_reconfigure::Server<TeleportControllerConfig> *dsrv_;
  teleport_controller::TeleportControllerConfig default_config_;

  bool transformPlansAndTrajs(const move_humans::map_pose_vector &plans,
                              const move_humans::map_trajectory &trajs,
                              move_humans::map_trajectory &transformed_trajs);

  bool getProjectedPose(const hanp_msgs::Trajectory &traj,
                        const size_t begin_index,
                        const geometry_msgs::Vector3 &pose,
                        geometry_msgs::Vector3 &projected_pose,
                        size_t &next_pose_index);

  bool projectPoint(const geometry_msgs::Vector3 &line_point1,
                    const geometry_msgs::Vector3 &line_point2,
                    const geometry_msgs::Vector3 &point,
                    geometry_msgs::Vector3 &porjected_point);

  void publishPlansFromTrajs(const move_humans::map_trajectory &trajs);

  enum point_advancing_type { ACCUMULATIVE, DIRECT };
  point_advancing_type point_advance_method_ =
      point_advancing_type::ACCUMULATIVE;

  ros::Time last_calc_time_;
  bool reset_time_ = true;
};
}; // namespace move_humans

#endif // TELEPORT_CONTROLLER_H_
