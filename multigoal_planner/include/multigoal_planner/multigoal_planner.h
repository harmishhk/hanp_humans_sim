#ifndef MULTIGOAL_PLANER_H
#define MULTIGOAL_PLANER_H

#include <ros/ros.h>
#include <nav_core/base_global_planner.h>
#include <navfn/navfn.h>
#include <costmap_2d/costmap_2d.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseArray.h>
#include <nav_msgs/Path.h>
#include <hanp_msgs/PathArray.h>
#include <tf/transform_datatypes.h>
#include <move_humans/planner_interface.h>
#include <navfn/potarr_point.h>
#include <nav_msgs/GetPlan.h>
#include <pcl_ros/publisher.h>
#include <move_humans/types.h>
#include <global_planner/potential_calculator.h>
#include <global_planner/expander.h>
#include <global_planner/traceback.h>
#include <global_planner/orientation_filter.h>

namespace multigoal_planner {
class MultiGoalPlanner : public move_humans::PlannerInterface {
public:
  MultiGoalPlanner();
  MultiGoalPlanner(std::string name, tf::TransformListener *tf,
                   costmap_2d::Costmap2DROS *costmap_ros);
  ~MultiGoalPlanner();

  void initialize(std::string name, tf::TransformListener *tf,
                  costmap_2d::Costmap2DROS *costmap_ros);

  bool makePlans(const move_humans::map_pose &starts,
                 const move_humans::map_pose &goals,
                 move_humans::map_pose_vector &plans);

  bool makePlans(const move_humans::map_pose &starts,
                 const move_humans::map_pose_vector &sub_goals,
                 const move_humans::map_pose &goals,
                 move_humans::map_pose_vector &plans);

private:
  tf::TransformListener *tf_;
  costmap_2d::Costmap2DROS *costmap_ros_;
  costmap_2d::Costmap2D *costmap_;

  ros::Publisher plans_pub_, plans_poses_pub_, potential_pub_;
  void publishPlans(move_humans::map_pose_vector &plans);

  bool initialized_, allow_unknown_, visualize_potential_,
      visualize_paths_poses_;
  double default_tolerance_, paths_poses_z_reduce_factor_;
  float convert_offset_;

  global_planner::PotentialCalculator *p_calc_;
  global_planner::Expander *planner_;
  global_planner::Traceback *path_maker_;
  global_planner::OrientationFilter *orientation_filter_;
  float *potential_array_;

  std::string tf_prefix_, global_frame_;
  boost::mutex planning_mutex_;

  bool getPlanFromPotential(double start_x, double start_y, double goal_x,
                            double goal_y,
                            std::vector<geometry_msgs::PoseStamped> &plan);

  bool worldToMap(double wx, double wy, double &mx, double &my);
  void mapToWorld(double mx, double my, double &wx, double &wy);
  void outlineMap(unsigned char *costarr, int nx, int ny, unsigned char value);
};
};

#endif // MULTIGOAL_PLANER_H
