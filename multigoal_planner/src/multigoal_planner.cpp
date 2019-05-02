#define NODE_NAME "multigoal_planner"
#define CONVERT_OFFSET 0.5
#define DEFAULT_TOLERANCE 0.0
#define SQ_DIST_PLAN_THRESHOLD 0.01
#define PLANS_PUB_TOPIC "plans"
#define PLANS_POSES_PUB_TOPIC "plans_poses"
#define POTENTIAL_PUB_TOPIC "potential"

#include <multigoal_planner/multigoal_planner.h>
#include <pluginlib/class_list_macros.h>
#include <global_planner/dijkstra.h>
#include <global_planner/gradient_path.h>
#include <global_planner/grid_path.h>
#include <global_planner/quadratic_calculator.h>
#include <boost/range/adaptor/reversed.hpp>

PLUGINLIB_EXPORT_CLASS(multigoal_planner::MultiGoalPlanner,
                       move_humans::PlannerInterface)

namespace multigoal_planner {
MultiGoalPlanner::MultiGoalPlanner()
    : tf2_(NULL), costmap_ros_(NULL), initialized_(false), allow_unknown_(true) {
}

MultiGoalPlanner::MultiGoalPlanner(std::string name, tf2_ros::Buffer *tf2,
                                   costmap_2d::Costmap2DROS *costmap_ros)
    : tf2_(NULL), costmap_ros_(NULL), initialized_(false), allow_unknown_(true) {
  initialize(name, tf2, costmap_ros);
}

MultiGoalPlanner::~MultiGoalPlanner() { delete dsrv_; }

void MultiGoalPlanner::initialize(std::string name, tf2_ros::Buffer *tf2,
                                  costmap_2d::Costmap2DROS *costmap_ros) {
  if (!initialized_) {
    tf2_ = tf2;
    tf2_ros::TransformListener tf2_lisn(*tf2_);
    costmap_ros_ = costmap_ros;

    costmap_ = costmap_ros_->getCostmap();
    planner_frame_ = costmap_ros_->getGlobalFrameID();

    auto cx = costmap_->getSizeInCellsX(), cy = costmap_->getSizeInCellsY();
    ROS_INFO_NAMED(NODE_NAME, "Costmap Xl: %f, Yl: %f", costmap_->getSizeInMetersX(),costmap_->getSizeInMetersY());

    p_calc_ = new global_planner::QuadraticCalculator(cx, cy);
    planner_ = new global_planner::DijkstraExpansion(p_calc_, cx, cy);
    path_maker_ = new global_planner::GradientPath(p_calc_);
    path_maker_fallback_ = new global_planner::GridPath(p_calc_);
    orientation_filter_ = new global_planner::OrientationFilter();
    orientation_filter_->setMode(global_planner::OrientationMode::FORWARD);

    ros::NodeHandle private_nh("~/" + name);
    private_nh.param("convert_offset", convert_offset_,
                     (float)(CONVERT_OFFSET));
    private_nh.param("allow_unknown", allow_unknown_, true);
    private_nh.param("default_tolerance", default_tolerance_,
                     DEFAULT_TOLERANCE);
    private_nh.param("sq_dist_plan_threshold", sq_dist_plan_threshold_,
                     SQ_DIST_PLAN_THRESHOLD);
    private_nh.param("publish_scale", publish_scale_, 100);

    planner_->setHasUnknown(allow_unknown_);

    ros::NodeHandle prefix_nh;
    tf_prefix_ = tf::getPrefixParam(prefix_nh);

    plans_pub_ =
        private_nh.advertise<hanp_msgs::HumanPathArray>(PLANS_PUB_TOPIC, 1);
    plans_poses_pub_ = private_nh.advertise<geometry_msgs::PoseArray>(
        PLANS_POSES_PUB_TOPIC, 1);
    potential_pub_ =
        private_nh.advertise<nav_msgs::OccupancyGrid>(POTENTIAL_PUB_TOPIC, 1);

    dsrv_ = new dynamic_reconfigure::Server<MultiGoalPlannerConfig>(private_nh);
    dynamic_reconfigure::Server<MultiGoalPlannerConfig>::CallbackType cb =
        boost::bind(&MultiGoalPlanner::reconfigureCB, this, _1, _2);
    dsrv_->setCallback(cb);

    initialized_ = true;
  } else {
    ROS_WARN_NAMED(NODE_NAME, "This planner has already been initialized, you "
                              "can't call it twice, doing nothing");
  }
}

void MultiGoalPlanner::reconfigureCB(MultiGoalPlannerConfig &config,
                                     uint32_t level) {
  boost::mutex::scoped_lock l(configuration_mutex_);
  if (setup_ && config.restore_defaults) {
    config = default_config_;
    config.restore_defaults = false;
  }
  if (!setup_) {
    default_config_ = config;
    setup_ = true;
  }

  last_config_ = config;
}

bool MultiGoalPlanner::makePlans(const move_humans::map_pose &starts,
                                 const move_humans::map_pose &goals,
                                 move_humans::map_pose_vectors &plans) {
  move_humans::map_pose_vector sub_goals;
  return makePlans(starts, sub_goals, goals, plans);
}

bool MultiGoalPlanner::makePlans(const move_humans::map_pose &starts,
                                 const move_humans::map_pose_vector &sub_goals,
                                 const move_humans::map_pose &goals,
                                 move_humans::map_pose_vectors &plans) {
  boost::mutex::scoped_lock lock(planning_mutex_);
  auto tolerance = default_tolerance_;

  if (!initialized_) {
    ROS_ERROR_NAMED(NODE_NAME, "This planner has not been initialized yet");
    return false;
  }

  if (starts.size() != goals.size()) {
    ROS_ERROR_NAMED(NODE_NAME,
                    "Size of start and goal points must be the same");
    return false;
  }
  for (auto start_kv : starts) {
    if (goals.find(start_kv.first) == goals.end()) {
      ROS_ERROR_NAMED(NODE_NAME, "Inconsistent human ids in starts and goals");
      return false;
    }
  }
  for (auto sub_goal_kv : sub_goals) {
    if (starts.find(sub_goal_kv.first) == starts.end()) {
      ROS_WARN_NAMED(NODE_NAME,
                     "Subgoal for unknown human %ld will be discarded",
                     sub_goal_kv.first);
    }
  }

  plans.clear();

  int nx = costmap_->getSizeInCellsX(), ny = costmap_->getSizeInCellsY();
  p_calc_->setSize(nx, ny);
  planner_->setSize(nx, ny);
  path_maker_->setSize(nx, ny);
  path_maker_fallback_->setSize(nx, ny);
  potential_array_ = new float[nx * ny];

  outlineMap(costmap_->getCharMap(), nx, ny, costmap_2d::LETHAL_OBSTACLE);

  move_humans::map_pose_vector combined_plans;
  for (auto start_kv : starts) {
    auto &human_id = start_kv.first;
    ROS_DEBUG_NAMED(NODE_NAME, "Planning for humans %ld", human_id);
    auto &start = start_kv.second;
    auto &goal = goals.find(human_id)->second;
    auto &sub_goal_vector = (sub_goals.find(human_id) != sub_goals.end())
                                ? sub_goals.find(human_id)->second
                                : move_humans::pose_vector();

    if (tf::resolve(tf_prefix_, start.header.frame_id) !=
        tf::resolve(tf_prefix_, planner_frame_)) {
      ROS_ERROR_NAMED(NODE_NAME, "The start pose must be in the %s frame; for "
                                 "human %ld, it is instead in the %s frame",
                      tf::resolve(tf_prefix_, planner_frame_).c_str(), human_id,
                      tf::resolve(tf_prefix_, start.header.frame_id).c_str());
      continue;
    }
    if (tf::resolve(tf_prefix_, goal.header.frame_id) !=
        tf::resolve(tf_prefix_, planner_frame_)) {
      ROS_ERROR_NAMED(NODE_NAME, "The goal pose must be in the %s frame; for "
                                 "human %ld, it is instead in the %s frame",
                      tf::resolve(tf_prefix_, planner_frame_).c_str(), human_id,
                      tf::resolve(tf_prefix_, goal.header.frame_id).c_str());
      continue;
    }
    bool valid_sub_goals = true;
    for (auto &sub_goal : sub_goal_vector) {
      if (tf::resolve(tf_prefix_, sub_goal.header.frame_id) !=
          tf::resolve(tf_prefix_, planner_frame_)) {
        ROS_ERROR_NAMED(
            NODE_NAME, "The sub-goal pose must be in the %s frame; for human "
                       "%ld, it is instead in the %s frame",
            tf::resolve(tf_prefix_, planner_frame_).c_str(), human_id,
            tf::resolve(tf_prefix_, sub_goal.header.frame_id).c_str());
        valid_sub_goals = false;
        break;
      }
    }
    if (!valid_sub_goals) {
      continue;
    }

    double valid_point_x, valid_point_y;
    std::vector<double> points_x, points_y;

    if (!worldToMap(start.pose.position.x, start.pose.position.y, valid_point_x,
                    valid_point_y)) {
      ROS_WARN_NAMED(NODE_NAME,
                     "Start position of human %ld is off the global costmap",
                     human_id);
      continue;
    }
    points_x.push_back(valid_point_x);
    points_y.push_back(valid_point_y);
    for (auto &sub_goal : sub_goal_vector) {
      if (!worldToMap(sub_goal.pose.position.x, sub_goal.pose.position.y,
                      valid_point_x, valid_point_y)) {
        ROS_WARN_NAMED(
            NODE_NAME,
            "Sub-goal position of human %ld is off the global costmap",
            human_id);
        continue;
      }
      points_x.push_back(valid_point_x);
      points_y.push_back(valid_point_y);
    }
    if (!worldToMap(goal.pose.position.x, goal.pose.position.y, valid_point_x,
                    valid_point_y)) {
      ROS_WARN_NAMED(NODE_NAME,
                     "Goal position of human %ld is off the global costmap",
                     human_id);
      continue;
    }
    points_x.push_back(valid_point_x);
    points_y.push_back(valid_point_y);

    if (points_x.size() < 2 || points_x.size() != points_y.size()) {
      continue;
    }

    move_humans::pose_vectors plan_vector;
    move_humans::pose_vector combined_plan;
    for (auto i = 0; i < (points_x.size() - 1); i++) {
      bool found_legal = planner_->calculatePotentials(
          costmap_->getCharMap(), points_x[i], points_y[i], points_x[i + 1],
          points_y[i + 1], nx * ny * 2, potential_array_);

      if (found_legal) {
        move_humans::pose_vector plan;
        if (getPlanFromPotential(points_x[i], points_y[i], points_x[i + 1],
                                 points_y[i + 1], plan)) {
          plan_vector.push_back(plan);
          combined_plan.insert(combined_plan.end(), plan.begin(), plan.end());

        } else {
          ROS_ERROR_NAMED(NODE_NAME, "Failed to get a plan from potential when "
                                     "a legal potential was found");
          combined_plan.clear();
          plan_vector.clear();
          break;
        }
      } else {
        ROS_ERROR_NAMED(NODE_NAME, "Failed to plan for human %ld", human_id);
        combined_plan.clear();
        plan_vector.clear();
        break;
      }
    }

    if (!combined_plan.empty()) {
      geometry_msgs::PoseStamped goal_copy = goal;
      goal_copy.header.stamp = ros::Time::now();
      combined_plan.push_back(goal_copy);
      orientation_filter_->processPath(combined_plan.front(), combined_plan);

      plan_vector.back().push_back(goal_copy);
      for (auto &plan : plan_vector) {
        if (!plan.empty()) {
          orientation_filter_->processPath(plan.front(), plan);
        }
      }
      plans[human_id] = plan_vector;
      combined_plans[human_id] = combined_plan;
      ROS_DEBUG_NAMED(NODE_NAME, "Added %ld plans for %ld human",
                      plan_vector.size(), human_id);
    }
  }

  delete potential_array_;
  publishPlans(combined_plans);

  return !plans.empty();
}

void MultiGoalPlanner::publishPlans(move_humans::map_pose_vector &plans) {
  if (last_config_.publish_human_plans) {
    hanp_msgs::HumanPathArray human_path_array;
    for (auto &plan_kv : plans) {
      hanp_msgs::HumanPath human_path;
      if (!plan_kv.second.empty()) {
        human_path.header.stamp = plan_kv.second.front().header.stamp;
        human_path.header.frame_id = plan_kv.second.front().header.frame_id;
        human_path.id = plan_kv.first;
        human_path.path.header.stamp = human_path.header.stamp;
        human_path.path.header.frame_id = human_path.header.frame_id;
        human_path.path.poses = plan_kv.second;
        human_path_array.paths.push_back(human_path);
      }
    }
    if (!human_path_array.paths.empty()) {
      human_path_array.header.stamp =
          human_path_array.paths.front().header.stamp;
      human_path_array.header.frame_id =
          human_path_array.paths.front().header.frame_id;
      plans_pub_.publish(human_path_array);
    }
  }

  bool header_set = false;
  if (last_config_.publish_human_poses) {
    geometry_msgs::PoseArray paths_poses;
    for (auto &plan_kv : plans) {
      for (size_t i = 0; i < plan_kv.second.size(); i++) {
        if (!header_set) {
          paths_poses.header.stamp = plan_kv.second[i].header.stamp;
          paths_poses.header.frame_id = plan_kv.second[i].header.frame_id;
        }
        auto pose_copy = plan_kv.second[i].pose;
        pose_copy.position.z = i / last_config_.poses_z_reduce_factor;
        paths_poses.poses.push_back(pose_copy);
      }
    }

    if (!paths_poses.poses.empty()) {
      plans_poses_pub_.publish(paths_poses);
    }
  }
}

bool MultiGoalPlanner::worldToMap(double wx, double wy, double &mx,
                                  double &my) {
  double origin_x = costmap_->getOriginX(), origin_y = costmap_->getOriginY();
  double resolution = costmap_->getResolution();
  if (wx < origin_x || wy < origin_y) {
    return false;
  }
  mx = (wx - origin_x) / resolution - convert_offset_;
  my = (wy - origin_y) / resolution - convert_offset_;
  if (mx < costmap_->getSizeInCellsX() && my < costmap_->getSizeInCellsY()) {
    return true;
  }
  return false;
}

void MultiGoalPlanner::outlineMap(unsigned char *costarr, int nx, int ny,
                                  unsigned char value) {
  unsigned char *pc = costarr;
  for (int i = 0; i < nx; i++) {
    *pc++ = value;
  }
  pc = costarr + (ny - 1) * nx;
  for (int i = 0; i < nx; i++) {
    *pc++ = value;
  }
  pc = costarr;
  for (int i = 0; i < ny; i++, pc += nx) {
    *pc = value;
  }
  pc = costarr + nx - 1;
  for (int i = 0; i < ny; i++, pc += nx) {
    *pc = value;
  }
}

bool MultiGoalPlanner::getPlanFromPotential(double start_x, double start_y,
                                            double goal_x, double goal_y,
                                            move_humans::pose_vector &plan) {
  std::vector<std::pair<float, float>> path;
  if (!path_maker_->getPath(potential_array_, start_x, start_y, goal_x, goal_y,
                            path)) {
    ROS_WARN_NAMED(NODE_NAME, "No path from potential using gradient");
    if (last_config_.publish_potential) {
      publishPotential(potential_array_);
    }
    path.clear();
    if (!path_maker_fallback_->getPath(potential_array_, start_x, start_y,
                                       goal_x, goal_y, path)) {
      ROS_ERROR_NAMED(NODE_NAME, "No path from potential using grid");
      return false;
    }
  }

  ros::Time plan_time = ros::Time::now();
  double world_x, world_y, last_world_x = 0.0, last_world_y = 0.0, wx_diff,
                           wy_diff, sq_dist_w;
  for (auto &point : boost::adaptors::reverse(path)) {
    mapToWorld(point.first, point.second, world_x, world_y);

    wx_diff = world_x - last_world_x;
    wy_diff = world_y - last_world_y;
    sq_dist_w = wx_diff * wx_diff + wy_diff * wy_diff;
    if (sq_dist_w < sq_dist_plan_threshold_) {
      continue;
    }
    last_world_x = world_x;
    last_world_y = world_y;

    geometry_msgs::PoseStamped pose;
    pose.header.stamp = plan_time;
    pose.header.frame_id = planner_frame_;
    pose.pose.position.x = world_x;
    pose.pose.position.y = world_y;
    pose.pose.position.z = 0.0;
    pose.pose.orientation.x = 0.0;
    pose.pose.orientation.y = 0.0;
    pose.pose.orientation.z = 0.0;
    pose.pose.orientation.w = 1.0;
    plan.push_back(pose);
  }
  return !plan.empty();
}

void MultiGoalPlanner::mapToWorld(double mx, double my, double &wx,
                                  double &wy) {
  wx = costmap_->getOriginX() +
       (mx + convert_offset_) * costmap_->getResolution();
  wy = costmap_->getOriginY() +
       (my + convert_offset_) * costmap_->getResolution();
}

void MultiGoalPlanner::publishPotential(float *potential) {
  int nx = costmap_->getSizeInCellsX(), ny = costmap_->getSizeInCellsY();
  double resolution = costmap_->getResolution();
  nav_msgs::OccupancyGrid grid;

  grid.header.frame_id = planner_frame_;
  grid.header.stamp = ros::Time::now();
  grid.info.resolution = resolution;
  grid.info.width = nx;
  grid.info.height = ny;

  double wx, wy;
  costmap_->mapToWorld(0, 0, wx, wy);
  grid.info.origin.position.x = wx - resolution / 2;
  grid.info.origin.position.y = wy - resolution / 2;
  grid.info.origin.position.z = 0.0;
  grid.info.origin.orientation.w = 1.0;

  grid.data.resize(nx * ny);

  float max = 0.0;
  for (unsigned int i = 0; i < grid.data.size(); i++) {
    float potential = potential_array_[i];
    if (potential < POT_HIGH) {
      if (potential > max) {
        max = potential;
      }
    }
  }

  for (unsigned int i = 0; i < grid.data.size(); i++) {
    if (potential_array_[i] >= POT_HIGH) {
      grid.data[i] = -1;
    } else
      grid.data[i] = potential_array_[i] * publish_scale_ / max;
  }
  potential_pub_.publish(grid);
}
} // namespace multigoal_planner
