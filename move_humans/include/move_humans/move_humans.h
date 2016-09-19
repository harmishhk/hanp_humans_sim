#ifndef MOVE_HUMANS_H_
#define MOVE_HUMANS_H_

#include <ros/ros.h>
#include <pluginlib/class_loader.h>
#include <actionlib/server/simple_action_server.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <std_srvs/Empty.h>
#include <std_srvs/SetBool.h>
#include <dynamic_reconfigure/server.h>
#include <hanp_msgs/HumanTrajectoryArray.h>

#include "move_humans/types.h"
#include "move_humans/planner_interface.h"
#include "move_humans/controller_interface.h"
#include <move_humans/MoveHumansConfig.h>
#include <move_humans/HumanPose.h>
#include <move_humans/MoveHumansAction.h>

namespace move_humans {
enum MoveHumansState { PLANNING, CONTROLLING, IDLE };

typedef actionlib::SimpleActionServer<move_humans::MoveHumansAction>
    MoveHumansActionServer;

class MoveHumans {
public:
  MoveHumans(tf::TransformListener &tf);
  virtual ~MoveHumans();

  bool executeCycle(move_humans::map_pose &goals,
                    move_humans::map_pose_vector &global_plans);

private:
  tf::TransformListener &tf_;

  MoveHumansActionServer *mhas_;
  void actionCB(const move_humans::MoveHumansGoalConstPtr &move_humans_goal);

  costmap_2d::Costmap2DROS *planner_costmap_ros_, *controller_costmap_ros_;
  ros::ServiceServer clear_costmaps_srv_;

  boost::shared_ptr<move_humans::PlannerInterface> planner_;
  boost::shared_ptr<move_humans::ControllerInterface> controller_;

  ros::Publisher current_goals_pub_, humans_pub_, humans_markers_pub_;
  bool publish_human_markers_, clear_human_markers_;

  bool use_external_trajs_, new_external_controller_trajs_;
  ros::Subscriber controller_trajs_sub_;
  void
  controllerPathsCB(const hanp_msgs::HumanTrajectoryArrayConstPtr traj_array);
  hanp_msgs::HumanTrajectoryArrayConstPtr external_controller_trajs_;

  ros::ServiceServer follow_external_path_srv_;
  std::string follow_external_path_service_name_;
  bool followExternalPaths(std_srvs::SetBool::Request &req,
                           std_srvs::SetBool::Response &res);

  dynamic_reconfigure::Server<move_humans::MoveHumansConfig> *dsrv_;
  move_humans::MoveHumansConfig last_config_;
  move_humans::MoveHumansConfig default_config_;
  void reconfigureCB(move_humans::MoveHumansConfig &config, uint32_t level);
  boost::recursive_mutex configuration_mutex_;

  pluginlib::ClassLoader<move_humans::PlannerInterface> planner_loader_;
  pluginlib::ClassLoader<move_humans::ControllerInterface> controller_loader_;

  move_humans::map_pose_vectors *planner_plans_, *controller_plans_,
      *latest_plans_;
  move_humans::map_pose_vector current_controller_plans_,
      current_planner_plans_;
  move_humans::map_size cp_indices_;
  move_humans::map_trajectory current_controller_trajectories_;

  MoveHumansState state_;
  bool setup_, shutdown_costmaps_, new_global_plans_, reset_controller_plans_,
      publish_feedback_;
  double human_radius_;

  double planner_frequency_, controller_frequency_;
  bool p_freq_change_, c_freq_change_;

  bool run_planner_;
  boost::mutex planner_mutex_, external_trajs_mutex_;
  boost::condition_variable planner_cond_;
  move_humans::map_pose planner_starts_, planner_goals_;
  move_humans::map_pose_vector planner_sub_goals_;
  boost::thread *planner_thread_;

  void planThread();
  void wakePlanner(const ros::TimerEvent &event);

  bool clearCostmapsService(std_srvs::Empty::Request &req,
                            std_srvs::Empty::Response &resp);
  void resetState();

  template <typename T>
  bool loadPlugin(const std::string plugin_name, boost::shared_ptr<T> &plugin,
                  pluginlib::ClassLoader<T> &plugin_loader,
                  costmap_2d::Costmap2DROS *plugin_costmap);

  bool isQuaternionValid(const geometry_msgs::Quaternion &q);
  move_humans::map_pose toGlobaolFrame(const move_humans::map_pose &pose_map);
  move_humans::map_pose_vector
  toGlobaolFrame(const move_humans::map_pose_vector &pose_vector_map);
  bool validateGoals(const move_humans::MoveHumansGoal &mh_goal,
                     move_humans::map_pose &starts,
                     move_humans::map_pose_vector &sub_goals,
                     move_humans::map_pose &goals);
  void publishHumans(const move_humans::map_traj_point &human_pts);
};
}; // namespace move_humans

#endif // MOVE_HUMANS_H_
