#define NODE_NAME "teleport_controller"
#define DIST_THRESHOLD 0.2  // distance threshold for pruning the plan
#define MAX_LINEAR_VEL 0.7  // m/s
#define MAX_ANGULAR_VEL 0.4 // r/s
#define MAX_LINEAR_ACC 0.0  // m/s^2
#define MAX_ANGULAR_ACC 0.0 // r/s^2
#define HUMAN_RADIUS 0.25   // m
#define DEFAUTL_SEGMENT_TYPE hanp_msgs::TrackedSegmentType::TORSO
#define PLANS_PUB_TOPIC "plans"
#define HUMANS_PUB_TOPIC "humans"
#define HUMANS_MARKERS_PUB_TOPIC "human_markers"
#define PUBLISH_HUMAN_MARKERS true
#define HUMANS_ARROWS_ID_OFFSET 100
#define HUMANS_CYLINDERS_HEIGHT 1.5
#define HUMAN_COLOR_R 0.5
#define HUMAN_COLOR_G 0.5
#define HUMAN_COLOR_B 0.0
#define MARKER_LIFETIME 4.0
#define DEFAULT_CONTROLLER_FRAME "map"
#define M_PI_2 1.57
#define MAX_START_DIST 1.0 // meters
#define LINEAR_DIST_EPS 0.001
#define ANGULAR_DIST_EPS 0.001

#include <pluginlib/class_list_macros.h>
#include <hanp_msgs/TrackedHumans.h>
#include <hanp_msgs/TrackedSegmentType.h>
#include <visualization_msgs/MarkerArray.h>
#include <angles/angles.h>

#include "teleport_controller/teleport_controller.h"

PLUGINLIB_EXPORT_CLASS(teleport_controller::TeleportController,
                       move_humans::ControllerInterface)

namespace teleport_controller {
TeleportController::TeleportController() : initialized_(false), setup_(false) {}

TeleportController::~TeleportController() { delete dsrv_; }

void TeleportController::initialize(std::string name, tf::TransformListener *tf,
                                    costmap_2d::Costmap2DROS *costmap_ros) {
  if (!isInitialized()) {
    ros::NodeHandle private_nh("~/" + name);
    tf_ = tf;
    costmap_ros_ = costmap_ros;
    controller_frame_ = costmap_ros_->getGlobalFrameID();
    if (controller_frame_.compare("") == 0) {
      controller_frame_ = DEFAULT_CONTROLLER_FRAME;
    }

    double dist_threshold = DIST_THRESHOLD;
    private_nh.param("dist_threshold", dist_threshold);
    sq_dist_threshold_ = dist_threshold * dist_threshold;
    private_nh.param("max_linear_vel", max_linear_vel_, MAX_LINEAR_VEL);
    private_nh.param("max_angular_vel", max_angular_vel_, MAX_ANGULAR_VEL);
    private_nh.param("max_linear_acc", max_linear_acc_, MAX_LINEAR_ACC);
    private_nh.param("max_angular_acc", max_angular_acc_, MAX_ANGULAR_ACC);
    private_nh.param("human_radius", human_radius_, HUMAN_RADIUS);
    private_nh.param("publish_human_markers", publish_human_markers_,
                     PUBLISH_HUMAN_MARKERS);

    plans_pub_ = private_nh.advertise<hanp_msgs::PathArray>(PLANS_PUB_TOPIC, 1);
    humans_pub_ =
        private_nh.advertise<hanp_msgs::TrackedHumans>(HUMANS_PUB_TOPIC, 1);
    if (publish_human_markers_) {
      humans_markers_pub_ =
          private_nh.advertise<visualization_msgs::MarkerArray>(
              HUMANS_MARKERS_PUB_TOPIC, 1);
      ROS_DEBUG_NAMED(NODE_NAME, "Will %spublish human markers",
                      publish_human_markers_ ? "" : "not ");
    }

    dsrv_ =
        new dynamic_reconfigure::Server<TeleportControllerConfig>(private_nh);
    dynamic_reconfigure::Server<TeleportControllerConfig>::CallbackType cb =
        boost::bind(&TeleportController::reconfigureCB, this, _1, _2);
    dsrv_->setCallback(cb);

    initialized_ = true;
  } else {
    ROS_WARN_NAMED(NODE_NAME, "This controller has already been initialized");
  }
}

void TeleportController::reconfigureCB(TeleportControllerConfig &config,
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
}

bool TeleportController::setPlans(const move_humans::map_pose_vector &plans) {
  move_humans::map_twist_vector twist_vector_map;
  return setPlans(plans, twist_vector_map);
}

bool TeleportController::setPlans(const move_humans::map_pose_vector &plans,
                                  const move_humans::map_twist_vector &twists) {
  if (!isInitialized()) {
    ROS_ERROR_NAMED(NODE_NAME, "This controller has not been initialized");
    return false;
  }

  ROS_INFO_NAMED(NODE_NAME, "Got %ld new human-plan%s", plans.size(),
                 plans.size() > 1 ? "s" : "");

  for (auto &plan_kv : plans) {
    auto &human_id = plan_kv.first;
    auto &plan = plan_kv.second;

    reached_goals_.erase(
        std::remove(reached_goals_.begin(), reached_goals_.end(), human_id),
        reached_goals_.end());
    // last_human_pts_.erase(human_id);
    last_traversed_indices_.erase(human_id);
    last_transformed_plans_.erase(human_id);
    last_transformed_twists_.erase(human_id);
    plans_[human_id] = plan;

    auto twist_it = twists.find(human_id);
    if (twist_it != twists.end()) {
      twists_[human_id] = twist_it->second;
    } else {
      twists_.erase(human_id);
    }
  }

  return true;
}

bool TeleportController::computeHumansStates(move_humans::map_pose &humans) {
  auto now = ros::Time::now();
  if (reset_time_) {
    last_calc_time_ = now;
    reset_time_ = false;
    return true;
  }
  double time_diff_sec = (now - last_calc_time_).toSec();
  last_calc_time_ = now;

  move_humans::map_pose_vector transformed_plans;
  move_humans::map_twist_vector transformed_twists;
  if (!transformPlans(plans_, twists_, transformed_plans, transformed_twists)) {
    ROS_ERROR_NAMED(NODE_NAME, "Cannot transform plans to controller frame");
    return false;
  }

  geometry_msgs::TwistStamped nominal_twist;
  nominal_twist.twist.linear.x = max_linear_vel_;
  nominal_twist.twist.angular.z = max_angular_vel_;
  for (auto &plan_kv : transformed_plans) {
    auto &human_id = plan_kv.first;
    auto &transformed_plan = plan_kv.second;

    if (transformed_plan.empty()) {
      ROS_ERROR_NAMED(NODE_NAME, "Transformed plan is empty for human %ld",
                      human_id);
      reached_goals_.push_back(human_id);
      continue;
    }

    move_humans::twist_vector *transformed_twist_v = nullptr;
    auto transformed_twist_v_it = transformed_twists.find(human_id);
    if (transformed_twist_v_it != transformed_twists.end()) {
      if (transformed_twist_v_it->second.size() != transformed_plan.size()) {
        ROS_WARN_NAMED(NODE_NAME,
                       "Twists provided but does not have same size as poses");
      } else {
        transformed_twist_v = &transformed_twist_v_it->second;
      }
    }

    std::pair<geometry_msgs::PoseStamped, geometry_msgs::TwistStamped>
        last_human_pt;
    auto last_human_pts_it = last_human_pts_.find(human_id);
    if (last_human_pts_it != last_human_pts_.end()) {
      last_human_pt.first = last_human_pts_it->second.first;
      last_human_pt.second = last_human_pts_it->second.second;
    } else {
      last_human_pt.first = transformed_plan.front();
      // last_human_pt.first.header.stamp = now;
      // if (transformed_twist_v != nullptr) {
      //   last_human_pt.second = transformed_twist_v->front();
      // } else {
      //   last_human_pt.second = nominal_twist;
      // }
    }

    size_t begin_index;
    auto last_traversed_indices_it = last_traversed_indices_.find(human_id);
    if (last_traversed_indices_it != last_traversed_indices_.end()) {
      begin_index = last_traversed_indices_it->second;
    } else {
      begin_index = 0;
      auto &start_pose = transformed_plan.front().pose;
      double start_dist = std::hypot(
          start_pose.position.x - last_human_pt.first.pose.position.x,
          start_pose.position.y - last_human_pt.first.pose.position.y);
      if (start_dist > MAX_START_DIST) {
        ROS_INFO_NAMED(NODE_NAME, "Resetting human %ld controller", human_id);
        ROS_INFO(
            "plan start x=%.2f, y=%.2f, last pose x=%.2f, y=%.2f, dist=%.2f",
            start_pose.position.x, start_pose.position.y,
            last_human_pt.first.pose.position.x,
            last_human_pt.first.pose.position.y, start_dist);
        last_human_pt.first = transformed_plan.front();
      }
      if (transformed_twist_v != nullptr &&
          transformed_twist_v->front().twist.linear.x > -10) {
        last_human_pt.second = transformed_twist_v->front();
      } else {
        last_human_pt.second = nominal_twist;
      }
    }

    auto &last_pose = last_human_pt.first.pose;
    auto &last_twist = last_human_pt.second.twist;
    ROS_INFO("last pose x=%.2f, y=%.2f, theta=%.2f, lin=%.2f, ang=%.2f",
             last_pose.position.x, last_pose.position.y,
             tf::getYaw(last_pose.orientation), last_twist.linear.x,
             last_twist.angular.z);

    size_t new_begin_index =
        prunePlan(transformed_plan, last_human_pt.first, begin_index);
    // last_traversed_indices_[human_id] = new_begin_index;
    new_begin_index++;

    double linear_vel, linear_dist, linear_time, angular_vel, angular_dist,
        angular_time, last_total_time = 0.0, total_time = 0.0;
    ROS_INFO("new begin index before %ld of %ld", new_begin_index,
             transformed_plan.size());

    if (point_advance_method_ == point_advancing_type::ACCUMULATIVE) {
      while (new_begin_index < transformed_plan.size()) {
        auto &next_pose = transformed_plan[new_begin_index].pose;
        auto &next_twist =
            (transformed_twist_v != nullptr &&
             (*transformed_twist_v)[new_begin_index].twist.linear.x > -10.0)
                ? (*transformed_twist_v)[new_begin_index].twist
                : nominal_twist.twist;
        if (last_twist.linear.x != 0) {
          linear_vel = last_twist.linear.x < 0
                           ? std::max(last_twist.linear.x, -max_linear_vel_)
                           : std::min(last_twist.linear.x, max_linear_vel_);
          linear_dist = std::hypot(next_pose.position.x - last_pose.position.x,
                                   next_pose.position.y - last_pose.position.y);
          linear_time = std::abs(linear_dist / linear_vel);
        } else {
          linear_time = 0;
          linear_dist = 0;
        }

        if (last_twist.angular.z != 0) {
          angular_vel = last_twist.angular.z < 0
                            ? std::max(last_twist.angular.z, -max_angular_vel_)
                            : std::min(last_twist.angular.z, max_angular_vel_);
          angular_dist = std::abs(tf::getYaw(next_pose.orientation) -
                                  tf::getYaw(last_pose.orientation));
          angular_time = std::abs(angular_dist / angular_vel);
        } else {
          angular_time = 0;
          angular_dist = 0;
        }

        if (linear_time == 0 && angular_time == 0) {
          total_time += std::numeric_limits<double>::infinity();
        } else {
          total_time += std::max(linear_time, angular_time);
        }

        if (total_time > time_diff_sec) {
          break;
        }

        last_pose = next_pose;
        last_twist = next_twist;
        last_total_time = total_time;
        new_begin_index++;
      }
    } else if (point_advance_method_ == point_advancing_type::DIRECT) {
      auto begin_pose = last_pose;
      linear_vel = last_twist.linear.x < 0
                       ? std::max(last_twist.linear.x, -max_linear_vel_)
                       : std::min(last_twist.linear.x, max_linear_vel_);
      angular_vel = last_twist.angular.z < 0
                        ? std::max(last_twist.angular.z, -max_angular_vel_)
                        : std::min(last_twist.angular.z, max_angular_vel_);
      while (new_begin_index < transformed_plan.size()) {
        auto &next_pose = transformed_plan[new_begin_index].pose;
        auto &next_twist =
            (transformed_twist_v != nullptr &&
             (*transformed_twist_v)[new_begin_index].twist.linear.x > -10.0)
                ? (*transformed_twist_v)[new_begin_index].twist
                : nominal_twist.twist;
        if (linear_vel != 0) {
          linear_dist =
              std::hypot(next_pose.position.x - begin_pose.position.x,
                         next_pose.position.y - begin_pose.position.y);
          linear_time = std::abs(linear_dist / linear_vel);
        } else {
          linear_time = 0;
          linear_dist = 0;
        }

        if (angular_vel != 0) {
          angular_dist = std::abs(tf::getYaw(next_pose.orientation) -
                                  tf::getYaw(begin_pose.orientation));
          angular_time = std::abs(angular_dist / angular_vel);
        } else {
          angular_time = 0;
          angular_dist = 0;
        }

        if (linear_time == 0 && angular_time == 0) {
          total_time = std::numeric_limits<double>::infinity();
        } else {
          total_time = std::max(linear_time, angular_time);
        }
        if (total_time > time_diff_sec) {
          break;
        }

        last_pose = next_pose;
        last_twist = next_twist;
        last_total_time = total_time;
        new_begin_index++;
      }
    } else {
      ROS_ERROR_NAMED(NODE_NAME, "logical error");
      continue;
    }
    ROS_INFO("new begin index after %ld of %ld", new_begin_index,
             transformed_plan.size());
    last_traversed_indices_[human_id] = new_begin_index - 1;

    if (new_begin_index < transformed_plan.size()) {
      double ep_time = time_diff_sec - last_total_time;
      auto &next_pose = transformed_plan[new_begin_index].pose;
      auto &next_twist =
          (transformed_twist_v != nullptr &&
           (*transformed_twist_v)[new_begin_index].twist.linear.x > -10.0)
              ? (*transformed_twist_v)[new_begin_index].twist
              : nominal_twist.twist;

      ROS_INFO("pose n-1 x=%.2f, y=%.2f, theta=%.2f, lin=%.2f, ang=%.2f",
               last_pose.position.x, last_pose.position.y,
               tf::getYaw(last_pose.orientation), last_twist.linear.x,
               last_twist.angular.z);
      ROS_INFO("pose n+1 x=%.2f, y=%.2f, theta=%.2f, lin=%.2f, ang=%.2f",
               next_pose.position.x, next_pose.position.y,
               tf::getYaw(next_pose.orientation), next_twist.linear.x,
               next_twist.angular.z);

      double linear_dist_ratio;
      if (std::abs(linear_dist) > LINEAR_DIST_EPS) {
        linear_dist_ratio = (last_twist.linear.x * ep_time) / linear_dist;
      } else {
        linear_dist_ratio = 0.0;
      }

      if (linear_dist_ratio < 1.0) {
        last_pose.position.x +=
            linear_dist_ratio * (next_pose.position.x - last_pose.position.x);
        last_pose.position.y +=
            linear_dist_ratio * (next_pose.position.y - last_pose.position.y);
        last_twist.linear.x +=
            linear_dist_ratio * (next_twist.linear.x - last_twist.linear.x);
      } else {
        last_pose.position = next_pose.position;
        last_twist.linear = next_twist.linear;
      }

      double angular_dist_ratio;
      if (std::abs(angular_dist) > ANGULAR_DIST_EPS) {
        angular_dist_ratio = (last_twist.angular.z * ep_time) / angular_dist;
      } else {
        angular_dist_ratio = 0.0;
      }
      if (angular_dist_ratio < 1.0) {
        double theta = tf::getYaw(last_pose.orientation);
        theta += angular_dist_ratio * (tf::getYaw(next_pose.orientation) -
                                       tf::getYaw(last_pose.orientation));
        last_pose.orientation = tf::createQuaternionMsgFromYaw(theta);
        last_twist.angular.z +=
            angular_dist_ratio * (next_twist.angular.z - last_twist.angular.z);
      } else {
        last_pose.orientation = next_pose.orientation;
        last_twist.angular = next_twist.angular;
      }

      ROS_INFO("ep=%.2f, lin_dist=%.2f, lin_ratio= %.2f, ang_dist=%.2f, "
               "ang_ratio=%.2f",
               ep_time, linear_dist, linear_dist_ratio, angular_dist,
               angular_dist_ratio);

      transformed_plan.erase(transformed_plan.begin(),
                             transformed_plan.begin() + new_begin_index);

      // if (transformed_twist_v != nullptr) {
      //   ROS_INFO_COND(human_id == 1,
      //                 "after the loop\nlast_pose: x=%.2f, y=%.2f, theta=%.2f,
      //                 "
      //                 "lin=%.2f, ang=%.2f\nnext_pose: x=%.2f, y=%.2f, "
      //                 "theta=%.2f\ntwist: lin=%.2f, ang=%.2f\nnew begin index
      //                 "
      //                 "= %ld\nratio lin=%.2f, ang=%.2f",
      //                 last_pose.position.x, last_pose.position.y,
      //                 tf::getYaw(last_pose.orientation), last_twist.linear.x,
      //                 last_twist.angular.z, next_pose.position.x,
      //                 next_pose.position.y,
      //                 tf::getYaw(next_pose.orientation),
      //                 (*transformed_twist_v)[new_begin_index].twist.linear.x,
      //                 (*transformed_twist_v)[new_begin_index].twist.angular.z,
      //                 new_begin_index, linear_dist_ratio,
      //                 angular_dist_ratio);
      // }
    } else {
      reached_goals_.push_back(human_id);
      last_twist.linear.x = 0.0;
      last_twist.angular.z = 0.0;
    }

    ROS_INFO("new pose x=%.2f, y=%.2f, theta=%.2f, lin=%.2f, ang=%.2f\n",
             last_pose.position.x, last_pose.position.y,
             tf::getYaw(last_pose.orientation), last_twist.linear.x,
             last_twist.angular.z);

    last_human_pt.first.header.stamp = now;
    last_human_pt.second.header.stamp = now;

    last_human_pts_[human_id] = last_human_pt;
  }

  for (auto &last_pt_kv : last_human_pts_) {
    humans[last_pt_kv.first] = last_pt_kv.second.first;
  }

  for (auto &human_id : reached_goals_) {
    last_human_pts_.erase(human_id);
    transformed_plans.erase(human_id);
  }

  if (last_human_pts_.empty()) {
    reset_time_ = true;
  }

  publishPlans(transformed_plans);
  publishHumans(last_human_pts_); // TODO: move this fucntion to move_humans.cpp
  return true;
}

bool TeleportController::areGoalsReached(
    move_humans::id_vector &reached_humans) {
  if (!isInitialized()) {
    ROS_ERROR_NAMED(NODE_NAME, "This controller has not been initialized");
    return false;
  }

  reached_humans = reached_goals_;
  return true;
}

bool TeleportController::transformPlans(
    const move_humans::map_pose_vector &plans,
    const move_humans::map_twist_vector &twists,
    move_humans::map_pose_vector &transformed_plans,
    move_humans::map_twist_vector &transformed_twists) {
  transformed_plans.clear();
  transformed_twists.clear();
  for (auto &plan_kv : plans) {
    auto &human_id = plan_kv.first;
    auto &plan = plan_kv.second;

    if (plan.empty()) {
      ROS_ERROR_NAMED(NODE_NAME, "Received empty plan for human %ld", human_id);
      continue;
    }

    if (plan[0].header.frame_id == "") {
      ROS_ERROR_NAMED(NODE_NAME, "Plan frame is empty for human %ld", human_id);
      continue;
    }

    if (std::find(reached_goals_.begin(), reached_goals_.end(), human_id) !=
        reached_goals_.end()) {
      continue;
    }

    auto transformed_plan_it = last_transformed_plans_.find(human_id);
    if (transformed_plan_it != last_transformed_plans_.end()) {
      transformed_plans[human_id] = transformed_plan_it->second;

      auto transformed_twist_it = last_transformed_twists_.find(human_id);
      if (transformed_twist_it != last_transformed_twists_.end()) {
        transformed_twists[human_id] = transformed_twist_it->second;
      }
      continue;
    }

    if (plan[0].header.frame_id != controller_frame_) {
      try {
        tf::StampedTransform plan_to_controller_transform;
        tf_->waitForTransform(controller_frame_, plan[0].header.frame_id,
                              ros::Time(0), ros::Duration(0.5));
        tf_->lookupTransform(controller_frame_, plan[0].header.frame_id,
                             ros::Time(0), plan_to_controller_transform);

        tf::Stamped<tf::Pose> tf_pose;
        geometry_msgs::PoseStamped transformed_pose;
        move_humans::pose_vector transformed_plan;
        for (auto &pose : plan) {
          tf::poseStampedMsgToTF(pose, tf_pose);
          tf_pose.setData(plan_to_controller_transform * tf_pose);
          tf_pose.stamp_ = plan_to_controller_transform.stamp_;
          tf_pose.frame_id_ = controller_frame_;
          tf::poseStampedTFToMsg(tf_pose, transformed_pose);
          transformed_plan.push_back(transformed_pose);
        }
        transformed_plans[human_id] = transformed_plan;

        auto twists_it = twists.find(human_id);
        if (twists_it != twists.end()) {
          auto &twist_v = twists_it->second;
          geometry_msgs::Twist plan_twist_in_controller_frame;
          tf_->lookupTwist(controller_frame_, plan[0].header.frame_id,
                           ros::Time(0), ros::Duration(0.1),
                           plan_twist_in_controller_frame);

          geometry_msgs::TwistStamped transformed_twist;
          move_humans::twist_vector transformed_twist_v;
          for (auto &twist : twist_v) {
            transformed_twist.header.stamp =
                plan_to_controller_transform.stamp_;
            transformed_twist.header.frame_id = controller_frame_;
            transformed_twist.twist.linear.x =
                twist.twist.linear.x - plan_twist_in_controller_frame.linear.x;
            transformed_twist.twist.linear.y =
                twist.twist.linear.y - plan_twist_in_controller_frame.linear.y;
            transformed_twist.twist.angular.z =
                twist.twist.angular.z -
                plan_twist_in_controller_frame.angular.z;
            transformed_twist_v.push_back(transformed_twist);
          }
          transformed_twists[human_id] = transformed_twist_v;
        }

      } catch (tf::LookupException &ex) {
        ROS_ERROR_NAMED(NODE_NAME, "No Transform available Error: %s\n",
                        ex.what());
        continue;
      } catch (tf::ConnectivityException &ex) {
        ROS_ERROR_NAMED(NODE_NAME, "Connectivity Error: %s\n", ex.what());
        continue;
      } catch (tf::ExtrapolationException &ex) {
        ROS_ERROR_NAMED(NODE_NAME, "Extrapolation Error: %s\n", ex.what());
        continue;
      }
    } else {
      transformed_plans[human_id] = plan;

      auto twists_it = twists.find(human_id);
      if (twists_it != twists.end() && twists_it->second.size() > 0) {
        transformed_twists[human_id] = twists_it->second;
      }
    }
  }

  return !transformed_plans.empty();
}

size_t
TeleportController::prunePlan(const move_humans::pose_vector &plan,
                              const geometry_msgs::PoseStamped &current_pose,
                              size_t begin_index) {
  size_t return_index = begin_index;
  double x_diff, y_diff, sq_diff,
      smallest_sq_diff = std::numeric_limits<double>::max();
  while (begin_index < plan.size()) {
    x_diff = plan[begin_index].pose.position.x - current_pose.pose.position.x;
    y_diff = plan[begin_index].pose.position.y - current_pose.pose.position.y;
    sq_diff = x_diff * x_diff + y_diff * y_diff;
    if (sq_diff < smallest_sq_diff) {
      return_index = begin_index;
      smallest_sq_diff = sq_diff;
    }
    ++begin_index;
  }
  return return_index;
}

void TeleportController::publishPlans(move_humans::map_pose_vector &plans) {
  hanp_msgs::PathArray path_array;
  for (auto &plan_kv : plans) {
    nav_msgs::Path path;
    if (!plan_kv.second.empty()) {
      path_array.ids.push_back(plan_kv.first);
      path.header = plan_kv.second[0].header;
      path.poses = plan_kv.second;
      path_array.paths.push_back(path);
    }
  }
  if (!path_array.paths.empty()) {
    path_array.header = path_array.paths[0].header;
    plans_pub_.publish(path_array);
  }
}

void TeleportController::publishHumans(move_humans::map_pose_twist &human_pts) {
  hanp_msgs::TrackedHumans humans;
  visualization_msgs::MarkerArray humans_markers;
  for (auto &pt_kv : human_pts) {
    hanp_msgs::TrackedSegment human_segment;
    human_segment.type = DEFAUTL_SEGMENT_TYPE;
    human_segment.pose.pose = pt_kv.second.first.pose;
    human_segment.pose.covariance[0] = human_radius_;
    human_segment.pose.covariance[7] = human_radius_;
    human_segment.twist.twist = pt_kv.second.second.twist;
    hanp_msgs::TrackedHuman human;
    human.track_id = pt_kv.first;
    human.segments.push_back(human_segment);
    humans.humans.push_back(human);

    if (publish_human_markers_) {
      visualization_msgs::Marker human_arrow, human_cylinder;

      human_arrow.header = pt_kv.second.first.header;
      human_arrow.type = visualization_msgs::Marker::ARROW;
      human_arrow.action = visualization_msgs::Marker::MODIFY;
      human_arrow.id = pt_kv.first + HUMANS_ARROWS_ID_OFFSET;
      human_arrow.pose = pt_kv.second.first.pose;
      human_arrow.scale.x = human_radius_ * 2.0;
      human_arrow.scale.y = 0.1;
      human_arrow.scale.z = 0.1;
      human_arrow.color.a = 1.0;
      human_arrow.color.r = HUMAN_COLOR_R;
      human_arrow.color.g = HUMAN_COLOR_G;
      human_arrow.color.b = HUMAN_COLOR_B;
      human_arrow.lifetime = ros::Duration(MARKER_LIFETIME);

      human_cylinder.header = pt_kv.second.first.header;
      human_cylinder.type = visualization_msgs::Marker::CYLINDER;
      human_cylinder.action = visualization_msgs::Marker::MODIFY;
      human_cylinder.id = pt_kv.first;
      human_cylinder.pose = pt_kv.second.first.pose;
      human_cylinder.pose.position.z += (HUMANS_CYLINDERS_HEIGHT / 2);
      human_cylinder.scale.x = human_radius_ * 2;
      human_cylinder.scale.y = human_radius_ * 2;
      human_cylinder.scale.z = HUMANS_CYLINDERS_HEIGHT;
      human_cylinder.color.a = 1.0;
      human_cylinder.color.r = HUMAN_COLOR_R;
      human_cylinder.color.g = HUMAN_COLOR_G;
      human_cylinder.color.b = HUMAN_COLOR_B;
      human_cylinder.lifetime = ros::Duration(MARKER_LIFETIME);

      humans_markers.markers.push_back(human_arrow);
      humans_markers.markers.push_back(human_cylinder);
    }
  }
  if (!humans.humans.empty()) {
    humans.header.stamp = ros::Time::now();
    humans.header.frame_id = controller_frame_;
    humans_pub_.publish(humans);

    if (publish_human_markers_) {
      humans_markers_pub_.publish(humans_markers);
    }
  }
}
};
