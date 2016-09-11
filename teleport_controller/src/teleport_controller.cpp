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
#define EP_TIME_EPS 0.001

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
  move_humans::map_trajectory trajectory_map;
  return setPlans(plans, trajectory_map);
}

bool TeleportController::setPlans(
    const move_humans::map_pose_vector &plans,
    const move_humans::map_trajectory &trajectories) {
  if (!isInitialized()) {
    ROS_ERROR_NAMED(NODE_NAME, "This controller has not been initialized");
    return false;
  }

  ROS_INFO_NAMED(NODE_NAME, "Got %ld plan%s and %ld trajector%s", plans.size(),
                 plans.size() > 1 ? "s" : "", trajectories.size(),
                 trajectories.size() > 0 ? "y" : "ies");

  for (auto &plan_kv : plans) {
    auto &human_id = plan_kv.first;
    auto &plan = plan_kv.second;

    reached_goals_.erase(
        std::remove(reached_goals_.begin(), reached_goals_.end(), human_id),
        reached_goals_.end());
    last_traversed_indices_.erase(human_id);
    last_transformed_trajs_.erase(human_id);
    plans_[human_id] = plan;
  }

  for (auto &trajectory_kv : trajectories) {
    auto &human_id = trajectory_kv.first;
    auto &trajectory = trajectory_kv.second;

    reached_goals_.erase(
        std::remove(reached_goals_.begin(), reached_goals_.end(), human_id),
        reached_goals_.end());
    last_traversed_indices_.erase(human_id);
    last_transformed_trajs_.erase(human_id);

    // trajectories override plans
    plans_.erase(human_id);
    trajs_[human_id] = trajectory;
  }

  return true;
}

bool TeleportController::computeHumansStates(move_humans::map_pose &humans) {
  // check if are running a new control sequency, reset time in case
  auto now = ros::Time::now();
  if (reset_time_) {
    last_calc_time_ = now;
    reset_time_ = false;
    return true;
  }
  double cycle_time = (now - last_calc_time_).toSec();
  last_calc_time_ = now;

  // get a copy of transformed plans, will transform if plans are new
  move_humans::map_trajectory transformed_trajs;
  if (!transformPlansAndTrajs(plans_, trajs_, transformed_trajs)) {
    ROS_ERROR_NAMED(NODE_NAME, "Cannot transform plans to controller frame");
    return false;
  }

  for (auto &traj_kv : transformed_trajs) {
    auto &human_id = traj_kv.first;
    auto &transformed_traj = traj_kv.second;

    if (transformed_traj.points.empty()) {
      ROS_ERROR_NAMED(
          NODE_NAME, "Transformed trajectory is empty for human %ld", human_id);
      reached_goals_.push_back(human_id);
      continue;
    }

    // get the last updated traj-point of the human, if we are revisiting them
    hanp_msgs::TrajectoryPoint last_traj_point;
    auto last_traj_points_it = last_traj_points_.find(human_id);
    if (last_traj_points_it != last_traj_points_.end()) {
      last_traj_point = last_traj_points_it->second;
    } else {
      last_traj_point = transformed_traj.points.front();
      ROS_INFO("new traj start pose x=%.2f, y=%.2f, th=%.2f",
               last_traj_point.transform.translation.x,
               last_traj_point.transform.translation.y,
               tf::getYaw(last_traj_point.transform.rotation));
    }

    // get last visited point index on the plan
    auto last_traversed_indices_it = last_traversed_indices_.find(human_id);
    size_t begin_index =
        last_traversed_indices_it != last_traversed_indices_.end()
            ? last_traversed_indices_it->second
            : 0;

    // reset controller if we got a new plan with too far starting pose
    if (begin_index == 0) {
      auto &start_pose = transformed_traj.points.front().transform;
      double start_dist = std::hypot(
          start_pose.translation.x - last_traj_point.transform.translation.x,
          start_pose.translation.y - last_traj_point.transform.translation.y);
      if (start_dist > MAX_START_DIST) {
        ROS_INFO_NAMED(NODE_NAME, "Resetting human %ld controller", human_id);
        ROS_INFO(
            "traj start x=%.2f, y=%.2f, last pose x=%.2f, y=%.2f, dist = %.2f",
            start_pose.translation.x, start_pose.translation.y,
            last_traj_point.transform.translation.x,
            last_traj_point.transform.translation.y, start_dist);
        last_traj_point = transformed_traj.points.front();
      }
    }

    // find porjected last pose on the plan
    geometry_msgs::Vector3 projected_last_trans;
    size_t next_point_index;
    if (!getProjectedPose(transformed_traj, begin_index,
                          last_traj_point.transform.translation,
                          projected_last_trans, next_point_index)) {
      ROS_ERROR_NAMED(NODE_NAME, "Error in projecint current pose");
      continue;
    }
    last_traj_point.transform.translation =
        projected_last_trans; // velocity,time are same

    // get references to adjusted last pose and twist
    // ROS_INFO(
    //     "proj pose x=%.2f, y=%.2f, theta=%.2f, lin=%.2f, ang=%.2f, tfs=%.2f",
    //     last_traj_point.transform.translation.x,
    //     last_traj_point.transform.translation.y,
    //     tf::getYaw(last_traj_point.transform.rotation),
    //     last_traj_point.velocity.linear.x,
    //     last_traj_point.velocity.angular.z,
    //     last_traj_point.time_from_start.toSec());

    auto last_point = last_traj_point;
    double last_time = last_point.time_from_start.toSec();
    double linear_dist, linear_time, angular_dist, angular_time, step_time,
        total_time = 0.0, acc_time = 0.0, last_total_time = 0.0;
    double start_point_time = last_time < 0 ? 0.0 : last_time;
    while (next_point_index < transformed_traj.points.size()) {
      auto &next_point = transformed_traj.points[next_point_index];
      double point_time = next_point.time_from_start.toSec();
      if (point_time < 0.0) {
        linear_dist = std::hypot(next_point.transform.translation.x -
                                     last_point.transform.translation.x,
                                 next_point.transform.translation.y -
                                     last_point.transform.translation.y);
        linear_time = linear_dist / max_linear_vel_;
        angular_dist = std::abs(angles::shortest_angular_distance(
            tf::getYaw(last_point.transform.rotation),
            tf::getYaw(next_point.transform.rotation)));
        angular_time = angular_dist / max_angular_vel_;
        angular_time = 0.0;
        step_time = std::max(linear_time, angular_time);
        acc_time += step_time;
        total_time += step_time;
      } else {
        total_time = acc_time + (point_time - start_point_time);
      }

      if (total_time >= cycle_time) {
        break;
      }
      last_total_time = total_time;
      last_point = next_point;
      next_point_index++;
    }

    if (next_point_index == transformed_traj.points.size()) {
      reached_goals_.push_back(human_id);
      last_traj_point.transform = transformed_traj.points.back().transform;
      last_traj_point.velocity.linear.x = 0.0;
      last_traj_point.velocity.angular.z = 0.0;
      last_traj_point.time_from_start.fromSec(-1.0);
      transformed_traj.points.clear();
      continue;
    }
    if (next_point_index != 0) {
      last_traversed_indices_[human_id] = next_point_index - 1;
    } else {
      last_traversed_indices_[human_id] = next_point_index;
    }

    double ep_time = cycle_time - last_total_time;
    auto &next_point = transformed_traj.points[next_point_index];

    if (ep_time > EP_TIME_EPS) {
      double last_point_time = last_point.time_from_start.toSec();
      double next_point_time = next_point.time_from_start.toSec();
      if (last_point_time >= 0.0 && next_point_time >= 0.0) {
        // here we first get interpolated velocity

        // ROS_INFO("pose n-1 x=%.2f, y=%.2f, theta=%.2f, lin=%.2f, ang=%.2f, "
        //          "time=%.2f",
        //          last_point.transform.translation.x,
        //          last_point.transform.translation.y,
        //          tf::getYaw(last_point.transform.rotation),
        //          last_point.velocity.linear.x, last_point.velocity.angular.z,
        //          last_point_time);
        // ROS_INFO("pose n+1 x=%.2f, y=%.2f, theta=%.2f, lin=%.2f, ang=%.2f "
        //          "time=%.2f, ep=%.2f",
        //          next_point.transform.translation.x,
        //          next_point.transform.translation.y,
        //          tf::getYaw(next_point.transform.rotation),
        //          next_point.velocity.linear.x, next_point.velocity.angular.z,
        //          next_point_time, ep_time);

        linear_dist = std::hypot(next_point.transform.translation.x -
                                     last_point.transform.translation.x,
                                 next_point.transform.translation.y -
                                     last_point.transform.translation.y);
        last_point.velocity.linear.x +=
            (next_point.velocity.linear.x - last_point.velocity.linear.x) *
            ep_time;
        last_point.velocity.angular.z +=
            (next_point.velocity.angular.z - last_point.velocity.angular.z) *
            ep_time;
        double can_lin_dist = last_point.velocity.linear.x * ep_time;
        double ratio_lin_dist = can_lin_dist / linear_dist;

        last_point.transform.translation.x +=
            (next_point.transform.translation.x -
             last_point.transform.translation.x) *
            ratio_lin_dist;
        last_point.transform.translation.y +=
            (next_point.transform.translation.y -
             last_point.transform.translation.y) *
            ratio_lin_dist;

        angular_dist = angles::shortest_angular_distance(
            tf::getYaw(last_point.transform.rotation),
            tf::getYaw(next_point.transform.rotation));
        double can_ang_dist = next_point.velocity.angular.z * ep_time;
        double ratio_ang_dist = can_ang_dist / std::abs(angular_dist);
        last_point.transform.rotation = tf::createQuaternionMsgFromYaw(
            tf::getYaw(last_point.transform.rotation) +
            (angular_dist * ratio_ang_dist));
      } else {
        // assuming maximum velocities
        linear_dist = std::hypot(next_point.transform.translation.x -
                                     last_point.transform.translation.x,
                                 next_point.transform.translation.y -
                                     last_point.transform.translation.y);
        double can_lin_dist = max_linear_vel_ * ep_time;
        double ratio_lin_dist = can_lin_dist / linear_dist;
        last_point.transform.translation.x +=
            (next_point.transform.translation.x -
             last_point.transform.translation.x) *
            ratio_lin_dist;
        last_point.transform.translation.y +=
            (next_point.transform.translation.y -
             last_point.transform.translation.y) *
            ratio_lin_dist;

        angular_dist = angles::shortest_angular_distance(
            tf::getYaw(last_point.transform.rotation),
            tf::getYaw(next_point.transform.rotation));
        double can_ang_dist = max_angular_vel_ * ep_time;
        double ratio_ang_dist = can_ang_dist / std::abs(angular_dist);
        last_point.transform.rotation = tf::createQuaternionMsgFromYaw(
            tf::getYaw(last_point.transform.rotation) +
            (angular_dist * ratio_ang_dist));

        // we calculate velocities from distance to last updated point
        linear_dist = std::hypot(last_point.transform.translation.x -
                                     last_traj_point.transform.translation.x,
                                 last_point.transform.translation.y -
                                     last_traj_point.transform.translation.y);
        last_point.velocity.linear.x = linear_dist / cycle_time;
        angular_dist = std::abs(tf::getYaw(last_point.transform.rotation) -
                                tf::getYaw(last_traj_point.transform.rotation));
        last_point.velocity.angular.z = angular_dist / cycle_time;
      }
    }

    // ROS_INFO(
    //     "new pose x=%.2f, y=%.2f, theta=%.2f, lin=%.2f, ang=%.2f, ep=%.2f\n",
    //     last_point.transform.translation.x,
    //     last_point.transform.translation.y,
    //     tf::getYaw(last_point.transform.rotation),
    //     last_point.velocity.linear.x,
    //     last_point.velocity.angular.z, ep_time);

    last_traj_points_[human_id] = last_point;
    if (next_point_index != 0) {
      transformed_traj.points.erase(transformed_traj.points.begin(),
                                    transformed_traj.points.begin() +
                                        next_point_index - 1);
    }
    transformed_traj.points.insert(transformed_traj.points.begin(), last_point);
  }

  for (auto &last_traj_point_kv : last_traj_points_) {
    auto &human_id = last_traj_point_kv.first;
    auto &human_traj_point = last_traj_point_kv.second;
    geometry_msgs::PoseStamped human_pose;
    human_pose.header.stamp = now;
    human_pose.header.frame_id = controller_frame_;
    human_pose.pose.position.x = human_traj_point.transform.translation.x;
    human_pose.pose.position.y = human_traj_point.transform.translation.y;
    human_pose.pose.orientation = human_traj_point.transform.rotation;
    humans[human_id] = human_pose;
  }

  for (auto &human_id : reached_goals_) {
    last_traj_points_.erase(human_id);
    transformed_trajs.erase(human_id);
  }

  if (last_traj_points_.empty()) {
    reset_time_ = true;
  }

  publishPlansFromTrajs(transformed_trajs);
  publishHumans(
      last_traj_points_); // TODO: move this fucntion to move_humans.cpp
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

// bool TeleportController::transformPlans(
//     const move_humans::map_pose_vector &plans,
//     const move_humans::map_twist_vector &twists,
//     move_humans::map_pose_vector &transformed_plans,
//     move_humans::map_twist_vector &transformed_twists) {
//   transformed_plans.clear();
//   transformed_twists.clear();
//   for (auto &plan_kv : plans) {
//     auto &human_id = plan_kv.first;
//     auto &plan = plan_kv.second;

//     if (plan.empty()) {
//       ROS_ERROR_NAMED(NODE_NAME, "Received empty plan for human %ld",
//       human_id);
//       continue;
//     }

//     if (plan[0].header.frame_id == "") {
//       ROS_ERROR_NAMED(NODE_NAME, "Plan frame is empty for human %ld",
//       human_id);
//       continue;
//     }

//     if (std::find(reached_goals_.begin(), reached_goals_.end(), human_id) !=
//         reached_goals_.end()) {
//       continue;
//     }

//     auto transformed_plan_it = last_transformed_plans_.find(human_id);
//     if (transformed_plan_it != last_transformed_plans_.end()) {
//       transformed_plans[human_id] = transformed_plan_it->second;

//       auto transformed_twist_it = last_transformed_twists_.find(human_id);
//       if (transformed_twist_it != last_transformed_twists_.end()) {
//         transformed_twists[human_id] = transformed_twist_it->second;
//       }
//       continue;
//     }

//     if (plan[0].header.frame_id != controller_frame_) {
//       try {
//         tf::StampedTransform plan_to_controller_transform;
//         tf_->waitForTransform(controller_frame_, plan[0].header.frame_id,
//                               ros::Time(0), ros::Duration(0.5));
//         tf_->lookupTransform(controller_frame_, plan[0].header.frame_id,
//                              ros::Time(0), plan_to_controller_transform);

//         tf::Stamped<tf::Pose> tf_pose;
//         geometry_msgs::PoseStamped transformed_pose;
//         move_humans::pose_vector transformed_plan;
//         for (auto &pose : plan) {
//           tf::poseStampedMsgToTF(pose, tf_pose);
//           tf_pose.setData(plan_to_controller_transform * tf_pose);
//           tf_pose.stamp_ = plan_to_controller_transform.stamp_;
//           tf_pose.frame_id_ = controller_frame_;
//           tf::poseStampedTFToMsg(tf_pose, transformed_pose);
//           transformed_plan.push_back(transformed_pose);
//         }
//         transformed_plans[human_id] = transformed_plan;

//         auto twists_it = twists.find(human_id);
//         if (twists_it != twists.end()) {
//           auto &twist_v = twists_it->second;
//           geometry_msgs::Twist plan_twist_in_controller_frame;
//           tf_->lookupTwist(controller_frame_, plan[0].header.frame_id,
//                            ros::Time(0), ros::Duration(0.1),
//                            plan_twist_in_controller_frame);

//           geometry_msgs::TwistStamped transformed_twist;
//           move_humans::twist_vector transformed_twist_v;
//           for (auto &twist : twist_v) {
//             transformed_twist.header.stamp =
//                 plan_to_controller_transform.stamp_;
//             transformed_twist.header.frame_id = controller_frame_;
//             transformed_twist.twist.linear.x =
//                 twist.twist.linear.x -
//                 plan_twist_in_controller_frame.linear.x;
//             transformed_twist.twist.linear.y =
//                 twist.twist.linear.y -
//                 plan_twist_in_controller_frame.linear.y;
//             transformed_twist.twist.angular.z =
//                 twist.twist.angular.z -
//                 plan_twist_in_controller_frame.angular.z;
//             transformed_twist_v.push_back(transformed_twist);
//           }
//           transformed_twists[human_id] = transformed_twist_v;
//         }

//       } catch (tf::LookupException &ex) {
//         ROS_ERROR_NAMED(NODE_NAME, "No Transform available Error: %s\n",
//                         ex.what());
//         continue;
//       } catch (tf::ConnectivityException &ex) {
//         ROS_ERROR_NAMED(NODE_NAME, "Connectivity Error: %s\n", ex.what());
//         continue;
//       } catch (tf::ExtrapolationException &ex) {
//         ROS_ERROR_NAMED(NODE_NAME, "Extrapolation Error: %s\n", ex.what());
//         continue;
//       }
//     } else {
//       transformed_plans[human_id] = plan;

//       auto twists_it = twists.find(human_id);
//       if (twists_it != twists.end() && twists_it->second.size() > 0) {
//         transformed_twists[human_id] = twists_it->second;
//       }
//     }
//   }

//   return !transformed_plans.empty();
// }

bool TeleportController::transformPlansAndTrajs(
    const move_humans::map_pose_vector &plans,
    const move_humans::map_trajectory &trajs,
    move_humans::map_trajectory &transformed_trajs) {
  transformed_trajs.clear();

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

    auto transformed_traj_it = last_transformed_trajs_.find(human_id);
    if (transformed_traj_it != last_transformed_trajs_.end()) {
      transformed_trajs[human_id] = transformed_traj_it->second;
      continue;
    }

    if (plan[0].header.frame_id != controller_frame_) {
      ROS_INFO("plan %s controller %s", plan[0].header.frame_id.c_str(),
               controller_frame_.c_str());
      try {
        tf::StampedTransform plan_to_controller_transform;
        tf_->waitForTransform(controller_frame_, plan[0].header.frame_id,
                              ros::Time(0), ros::Duration(0.5));
        tf_->lookupTransform(controller_frame_, plan[0].header.frame_id,
                             ros::Time(0), plan_to_controller_transform);

        tf::Transform tf_trans;
        hanp_msgs::Trajectory transformed_traj;
        transformed_traj.header.stamp = plan_to_controller_transform.stamp_;
        transformed_traj.header.frame_id = controller_frame_;
        for (auto &pose : plan) {
          hanp_msgs::TrajectoryPoint traj_point;
          traj_point.transform.translation.x = pose.pose.position.x;
          traj_point.transform.translation.y = pose.pose.position.y;
          traj_point.transform.translation.z = pose.pose.position.z;
          traj_point.transform.rotation = pose.pose.orientation;
          tf::transformMsgToTF(traj_point.transform, tf_trans);
          tf_trans = plan_to_controller_transform * tf_trans;
          tf::transformTFToMsg(tf_trans, traj_point.transform);
          traj_point.time_from_start.fromSec(-1.0);
          transformed_traj.points.push_back(traj_point);
        }
        transformed_trajs[human_id] = transformed_traj;
        last_transformed_trajs_[human_id] = transformed_traj;
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
      hanp_msgs::Trajectory transformed_traj;
      for (auto &pose : plan) {
        hanp_msgs::TrajectoryPoint traj_point;
        traj_point.transform.translation.x = pose.pose.position.x;
        traj_point.transform.translation.y = pose.pose.position.y;
        traj_point.transform.translation.z = pose.pose.position.z;
        traj_point.transform.rotation = pose.pose.orientation;
        traj_point.time_from_start.fromSec(-1.0);
        transformed_traj.points.push_back(traj_point);
      }
      transformed_trajs[human_id] = transformed_traj;
      last_transformed_trajs_[human_id] = transformed_traj;
    }
  }

  for (auto &traj_kv : trajs) {
    auto &human_id = traj_kv.first;
    auto &traj = traj_kv.second;

    if (traj.points.empty()) {
      ROS_ERROR_NAMED(NODE_NAME, "Received empty trajectory for human %ld",
                      human_id);
      continue;
    }

    if (traj.header.frame_id == "") {
      ROS_ERROR_NAMED(NODE_NAME, "Plan frame is empty for human %ld", human_id);
      continue;
    }

    if (std::find(reached_goals_.begin(), reached_goals_.end(), human_id) !=
        reached_goals_.end()) {
      continue;
    }

    auto transformed_traj_it = last_transformed_trajs_.find(human_id);
    if (transformed_traj_it != last_transformed_trajs_.end()) {
      transformed_trajs[human_id] = transformed_traj_it->second;
      continue;
    }

    if (traj.header.frame_id != controller_frame_) {
      try {
        tf::StampedTransform traj_to_controller_transform;
        tf_->waitForTransform(controller_frame_, traj.header.frame_id,
                              ros::Time(0), ros::Duration(0.5));
        tf_->lookupTransform(controller_frame_, traj.header.frame_id,
                             ros::Time(0), traj_to_controller_transform);
        geometry_msgs::Twist traj_vel_in_controller_frame;
        tf_->lookupTwist(controller_frame_, traj.header.frame_id, ros::Time(0),
                         ros::Duration(0.1), traj_vel_in_controller_frame);

        tf::Transform tf_trans;
        hanp_msgs::Trajectory transformed_traj;
        transformed_traj.header.stamp = traj_to_controller_transform.stamp_;
        transformed_traj.header.frame_id = controller_frame_;
        for (auto &traj_point : traj.points) {
          hanp_msgs::TrajectoryPoint tr_traj_point;
          tf::transformMsgToTF(traj_point.transform, tf_trans);
          tf_trans = traj_to_controller_transform * tf_trans;
          tf::transformTFToMsg(tf_trans, tr_traj_point.transform);

          ROS_INFO_THROTTLE(
              1, "\nfrom: x=%.2f, y=%.2f, th=%.2f\nto: x=%.2f, y=%.2f, th=%.2f",
              traj_point.transform.translation.x,
              traj_point.transform.translation.y,
              tf::getYaw(traj_point.transform.rotation),
              tr_traj_point.transform.translation.x,
              tr_traj_point.transform.translation.y,
              tf::getYaw(tr_traj_point.transform.rotation));

          tr_traj_point.velocity.linear.x =
              traj_point.velocity.linear.x -
              traj_vel_in_controller_frame.linear.x;
          tr_traj_point.velocity.linear.y =
              traj_point.velocity.linear.y -
              traj_vel_in_controller_frame.linear.y;
          tr_traj_point.velocity.angular.z =
              traj_point.velocity.angular.z -
              traj_vel_in_controller_frame.angular.z;

          tr_traj_point.time_from_start = traj_point.time_from_start;
          transformed_traj.points.push_back(tr_traj_point);
        }
        transformed_trajs[human_id] = transformed_traj;
        last_transformed_trajs_[human_id] = transformed_traj;
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
      ROS_INFO("givng old traj for human %ld", human_id);
      transformed_trajs[human_id] = traj;
      last_transformed_trajs_[human_id] = traj;
    }
  }

  return !transformed_trajs.empty();
}

// size_t
// TeleportController::prunePlan(const move_humans::pose_vector &plan,
//                               const geometry_msgs::PoseStamped &current_pose,
//                               size_t begin_index) {
//   size_t return_index = begin_index;
//   double x_diff, y_diff, sq_diff,
//       smallest_sq_diff = std::numeric_limits<double>::max();
//   while (begin_index < plan.size()) {
//     x_diff = plan[begin_index].pose.position.x -
//     current_pose.pose.position.x;
//     y_diff = plan[begin_index].pose.position.y -
//     current_pose.pose.position.y;
//     sq_diff = x_diff * x_diff + y_diff * y_diff;
//     if (sq_diff < smallest_sq_diff) {
//       return_index = begin_index;
//       smallest_sq_diff = sq_diff;
//     }
//     ++begin_index;
//   }
//   return return_index;
// }

bool TeleportController::getProjectedPose(
    const hanp_msgs::Trajectory &traj, const size_t begin_index,
    const geometry_msgs::Vector3 &pose, geometry_msgs::Vector3 &projected_pose,
    size_t &next_pose_index) {
  if (begin_index >= traj.points.size()) {
    ROS_ERROR_NAMED(NODE_NAME,
                    "Out of bound index provided to getProjectedPoint");
    return false;
  }

  if (begin_index == (traj.points.size() - 1)) {
    projected_pose = pose;
    next_pose_index = begin_index;
    return true;
  }

  // get the point with the smallest distance from start index
  double x_diff, y_diff, sq_diff,
      smallest_sq_diff = std::numeric_limits<double>::max();
  auto np_index = begin_index;
  // size_t np_index = 0;
  while (np_index < traj.points.size()) {
    x_diff = traj.points[np_index].transform.translation.x - pose.x;
    y_diff = traj.points[np_index].transform.translation.y - pose.y;
    sq_diff = x_diff * x_diff + y_diff * y_diff;
    if (sq_diff > smallest_sq_diff) {
      break;
    } else {
      smallest_sq_diff = sq_diff;
      np_index++;
    }
  }
  np_index--;

  // return projected point on line between nearest point and point next ot it
  if (np_index == (traj.points.size() - 1)) {
    if (projectPoint(traj.points[np_index].transform.translation,
                     traj.points[np_index - 1].transform.translation, pose,
                     projected_pose)) {
      next_pose_index = np_index;
    } else {
      next_pose_index = np_index - 1;
    }
  } else {
    if (projectPoint(traj.points[np_index].transform.translation,
                     traj.points[np_index + 1].transform.translation, pose,
                     projected_pose)) {
      next_pose_index = np_index + 1;
    } else {
      next_pose_index = np_index;
    }
  }
  return true;
}

bool TeleportController::projectPoint(const geometry_msgs::Vector3 &line_point1,
                                      const geometry_msgs::Vector3 &line_point2,
                                      const geometry_msgs::Vector3 &point,
                                      geometry_msgs::Vector3 &porjected_point) {
  geometry_msgs::Point e1, e2;
  e1.x = line_point2.x - line_point1.x;
  e1.y = line_point2.y - line_point1.y;
  e2.x = point.x - line_point1.x;
  e2.y = point.y - line_point1.y;

  double val_dp = e1.x * e2.x + e1.y * e2.y;
  double lene1_sq = e1.x * e1.x + e1.y * e1.y;

  porjected_point.x = line_point1.x + (val_dp * e1.x) / lene1_sq;
  porjected_point.y = line_point1.y + (val_dp * e1.y) / lene1_sq;

  // ROS_INFO("\nlp1: x=%.2f, y=%.2f\nlp2: x=%.2f, y=%.2f\np: x=%.2f y=%.2f\npp:
  // "
  //          "x=%.2f, y=%.2f\nnp_index: %s\n",
  //          line_point1.x, line_point1.y, line_point2.x, line_point2.y,
  //          point.x,
  //          point.y, porjected_point.x, porjected_point.y,
  //          (val_dp > 0 && val_dp < lene1_sq) ? "in" : "out");

  return (val_dp > 0 && val_dp < lene1_sq);
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

void TeleportController::publishPlansFromTrajs(
    const move_humans::map_trajectory &trajs) {
  auto now = ros::Time::now();
  hanp_msgs::PathArray path_array;
  for (auto &traj_kv : trajs) {
    if (!traj_kv.second.points.empty()) {
      nav_msgs::Path path;
      path.header.stamp = now;
      path.header.frame_id = controller_frame_;
      for (auto &traj_point : traj_kv.second.points) {
        geometry_msgs::PoseStamped pose;
        pose.header.stamp = now;
        pose.header.frame_id = controller_frame_;
        pose.pose.position.x = traj_point.transform.translation.x;
        pose.pose.position.y = traj_point.transform.translation.y;
        pose.pose.position.z = traj_point.transform.translation.z;
        pose.pose.orientation = traj_point.transform.rotation;
        path.poses.push_back(pose);
      }
      path_array.ids.push_back(traj_kv.first);
      path_array.paths.push_back(path);
    }
  }
  if (!path_array.paths.empty()) {
    path_array.header = path_array.paths[0].header;
    plans_pub_.publish(path_array);
  }
}

// void TeleportController::publishHumans(move_humans::map_pose_twist
// &human_pts) {
//   hanp_msgs::TrackedHumans humans;
//   visualization_msgs::MarkerArray humans_markers;
//   for (auto &pt_kv : human_pts) {
//     hanp_msgs::TrackedSegment human_segment;
//     human_segment.type = DEFAUTL_SEGMENT_TYPE;
//     human_segment.pose.pose = pt_kv.second.first.pose;
//     human_segment.pose.covariance[0] = human_radius_;
//     human_segment.pose.covariance[7] = human_radius_;
//     human_segment.twist.twist = pt_kv.second.second.twist;
//     hanp_msgs::TrackedHuman human;
//     human.track_id = pt_kv.first;
//     human.segments.push_back(human_segment);
//     humans.humans.push_back(human);

//     if (publish_human_markers_) {
//       visualization_msgs::Marker human_arrow, human_cylinder;

//       human_arrow.header = pt_kv.second.first.header;
//       human_arrow.type = visualization_msgs::Marker::ARROW;
//       human_arrow.action = visualization_msgs::Marker::MODIFY;
//       human_arrow.id = pt_kv.first + HUMANS_ARROWS_ID_OFFSET;
//       human_arrow.pose = pt_kv.second.first.pose;
//       human_arrow.scale.x = human_radius_ * 2.0;
//       human_arrow.scale.y = 0.1;
//       human_arrow.scale.z = 0.1;
//       human_arrow.color.a = 1.0;
//       human_arrow.color.r = HUMAN_COLOR_R;
//       human_arrow.color.g = HUMAN_COLOR_G;
//       human_arrow.color.b = HUMAN_COLOR_B;
//       human_arrow.lifetime = ros::Duration(MARKER_LIFETIME);

//       human_cylinder.header = pt_kv.second.first.header;
//       human_cylinder.type = visualization_msgs::Marker::CYLINDER;
//       human_cylinder.action = visualization_msgs::Marker::MODIFY;
//       human_cylinder.id = pt_kv.first;
//       human_cylinder.pose = pt_kv.second.first.pose;
//       human_cylinder.pose.position.z += (HUMANS_CYLINDERS_HEIGHT / 2);
//       human_cylinder.scale.x = human_radius_ * 2;
//       human_cylinder.scale.y = human_radius_ * 2;
//       human_cylinder.scale.z = HUMANS_CYLINDERS_HEIGHT;
//       human_cylinder.color.a = 1.0;
//       human_cylinder.color.r = HUMAN_COLOR_R;
//       human_cylinder.color.g = HUMAN_COLOR_G;
//       human_cylinder.color.b = HUMAN_COLOR_B;
//       human_cylinder.lifetime = ros::Duration(MARKER_LIFETIME);

//       humans_markers.markers.push_back(human_arrow);
//       humans_markers.markers.push_back(human_cylinder);
//     }
//   }
//   if (!humans.humans.empty()) {
//     humans.header.stamp = ros::Time::now();
//     humans.header.frame_id = controller_frame_;
//     humans_pub_.publish(humans);

//     if (publish_human_markers_) {
//       humans_markers_pub_.publish(humans_markers);
//     }
//   }
// }

void TeleportController::publishHumans(
    const move_humans::map_traj_point &human_pts) {
  auto now = ros::Time::now();

  hanp_msgs::TrackedHumans humans;
  visualization_msgs::MarkerArray humans_markers;
  for (auto &human_pt_kv : human_pts) {
    hanp_msgs::TrackedSegment human_segment;
    human_segment.type = DEFAUTL_SEGMENT_TYPE;
    human_segment.pose.pose.position.x =
        human_pt_kv.second.transform.translation.x;
    human_segment.pose.pose.position.y =
        human_pt_kv.second.transform.translation.y;
    human_segment.pose.pose.orientation = human_pt_kv.second.transform.rotation;
    human_segment.pose.covariance[0] = human_radius_;
    human_segment.pose.covariance[7] = human_radius_;
    human_segment.twist.twist = human_pt_kv.second.velocity;
    hanp_msgs::TrackedHuman human;
    human.track_id = human_pt_kv.first;
    human.segments.push_back(human_segment);
    humans.humans.push_back(human);

    if (publish_human_markers_) {
      visualization_msgs::Marker human_arrow, human_cylinder;

      human_arrow.header.stamp = now;
      human_arrow.header.frame_id = controller_frame_;
      human_arrow.type = visualization_msgs::Marker::ARROW;
      human_arrow.action = visualization_msgs::Marker::MODIFY;
      human_arrow.id = human_pt_kv.first + HUMANS_ARROWS_ID_OFFSET;
      human_arrow.pose.position.x = human_pt_kv.second.transform.translation.x;
      human_arrow.pose.position.y = human_pt_kv.second.transform.translation.y;
      human_arrow.pose.position.z = human_pt_kv.second.transform.translation.z;
      human_arrow.pose.orientation = human_pt_kv.second.transform.rotation;
      human_arrow.scale.x = human_radius_ * 2.0;
      human_arrow.scale.y = 0.1;
      human_arrow.scale.z = 0.1;
      human_arrow.color.a = 1.0;
      human_arrow.color.r = HUMAN_COLOR_R;
      human_arrow.color.g = HUMAN_COLOR_G;
      human_arrow.color.b = HUMAN_COLOR_B;
      human_arrow.lifetime = ros::Duration(MARKER_LIFETIME);

      human_cylinder.header.stamp = now;
      human_cylinder.header.frame_id = controller_frame_;
      human_cylinder.type = visualization_msgs::Marker::CYLINDER;
      human_cylinder.action = visualization_msgs::Marker::MODIFY;
      human_cylinder.id = human_pt_kv.first;
      human_cylinder.pose.position.x =
          human_pt_kv.second.transform.translation.x;
      human_cylinder.pose.position.y =
          human_pt_kv.second.transform.translation.y;
      human_cylinder.pose.position.z += (HUMANS_CYLINDERS_HEIGHT / 2);
      // human_cylinder.pose.orientation =
      // human_pt_kv.second.transform.rotation;
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
    humans.header.stamp = now;
    humans.header.frame_id = controller_frame_;
    humans_pub_.publish(humans);

    if (publish_human_markers_) {
      humans_markers_pub_.publish(humans_markers);
    }
  }
}
};
