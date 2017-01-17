#define NODE_NAME "teleport_controller"
#define PLANS_PUB_TOPIC "plans"
#define DEFAULT_CONTROLLER_FRAME "map"
#define M_PI_2 1.57
#define LINEAR_DIST_EPS 0.001
#define ANGULAR_DIST_EPS 0.001
#define EP_TIME_EPS 0.001
#define POINT_JUMP_EPS 0.001
#define ANG_VEL_EPS 0.001
#define THROTTLE_TIME 1 // seconds

#include <pluginlib/class_list_macros.h>
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

    plans_pub_ =
        private_nh.advertise<hanp_msgs::HumanPathArray>(PLANS_PUB_TOPIC, 1);

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

  last_config_ = config;
}

bool TeleportController::setPlans(const move_humans::map_pose_vector &plans) {
  move_humans::map_trajectory trajectory_map;
  move_humans::map_twist twist_map;
  return setPlans(plans, trajectory_map, twist_map);
}

bool TeleportController::setPlans(
    const move_humans::map_pose_vector &plans,
    const move_humans::map_trajectory &trajectories,
    const move_humans::map_twist &vels) {
  if (!isInitialized()) {
    ROS_ERROR_NAMED(NODE_NAME, "This controller has not been initialized");
    return false;
  }

  ROS_DEBUG_NAMED(NODE_NAME, "Got %ld plan%s, %ld trajector%s and %ld twist%s",
                  plans.size(), plans.size() == 1 ? "" : "s",
                  trajectories.size(), trajectories.size() == 1 ? "y" : "ies",
                  vels.size(), vels.size() == 1 ? "" : "s");

  for (auto &plan_kv : plans) {
    auto &human_id = plan_kv.first;
    auto &plan = plan_kv.second;

    // new plans received, so remove entries from reached_goals_
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

    // new trajectories received, so remove entries from reached_goals_
    reached_goals_.erase(
        std::remove(reached_goals_.begin(), reached_goals_.end(), human_id),
        reached_goals_.end());
    last_traversed_indices_.erase(human_id);
    last_transformed_trajs_.erase(human_id);
    auto last_traj_points_it = last_traj_points_.find(human_id);
    if (last_traj_points_it != last_traj_points_.end()) {
      last_traj_points_it->second.time_from_start.fromSec(0.0);
    }

    // trajectories override plans
    plans_.erase(human_id);
    trajs_[human_id] = trajectory;
    if (trajs_[human_id].points.size() > 1) {
      trajs_[human_id].points.erase(trajs_[human_id].points.begin());
    }
  }

  vels_.clear();
  for (auto &vel_kv : vels) {
    auto &human_id = vel_kv.first;
    auto &human_twist = vel_kv.second;

    // new velocities received, so remove entries from reached_goals_
    reached_goals_.erase(
        std::remove(reached_goals_.begin(), reached_goals_.end(), human_id),
        reached_goals_.end());

    // velocity moder override plans
    plans_.erase(human_id);

    // update velocities that will be used at next iteration of controller
    if (human_twist.linear.z != -1.0) {
      vels_[human_id] = human_twist;
    }
  }

  return true;
}

bool TeleportController::computeHumansStates(
    move_humans::map_traj_point &humans) {
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
      if (start_dist > last_config_.reset_dist) {
        ROS_INFO_NAMED(NODE_NAME, "Resetting human %ld controller", human_id);
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
    last_traj_point.transform.translation = projected_last_trans;
    last_traj_point.transform.rotation =
        transformed_traj.points[next_point_index].transform.rotation;
    // not updating time and velocity of last_traj_point;

    // get references to adjusted last pose and twist
    auto last_point = last_traj_point;
    double last_time = last_point.time_from_start.toSec();
    double linear_dist, linear_time, angular_dist, angular_time, step_time,
        total_time = 0.0, acc_time = 0.0, last_total_time = 0.0;
    double start_point_time = last_time < 0.0 ? 0.0 : last_time;
    while (next_point_index < transformed_traj.points.size()) {
      auto &next_point = transformed_traj.points[next_point_index];
      double point_time = next_point.time_from_start.toSec();
      if (point_time < 0.0) {
        linear_dist = std::hypot(next_point.transform.translation.x -
                                     last_point.transform.translation.x,
                                 next_point.transform.translation.y -
                                     last_point.transform.translation.y);
        if (linear_dist < POINT_JUMP_EPS) {
          next_point_index++;
          continue;
        }
        linear_time = linear_dist / last_config_.max_linear_vel;
        angular_dist = std::abs(angles::shortest_angular_distance(
            tf::getYaw(last_point.transform.rotation),
            tf::getYaw(next_point.transform.rotation)));
        angular_time = angular_dist / last_config_.max_angular_vel;
        // angular_time = 0.0;
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

    if (next_point_index >= (transformed_traj.points.size() - 1)) {
      reached_goals_.push_back(human_id);
      last_traj_point.transform = transformed_traj.points.back().transform;
      last_traj_point.velocity.linear.x = 0.0;
      last_traj_point.velocity.angular.z = 0.0;
      last_traj_point.time_from_start.fromSec(-1.0);
      last_traj_points_[human_id] = last_traj_point;
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

    double last_point_time = last_point.time_from_start.toSec();
    double next_point_time = next_point.time_from_start.toSec();

    // ROS_INFO(
    //     "n-1 x=%.2f y=%.2f theta=%.2f lin=%.2f ang=%.2f time=%.2f",
    //     last_point.transform.translation.x, last_point.transform.translation.y,
    //     tf::getYaw(last_point.transform.rotation), last_point.velocity.linear.x,
    //     last_point.velocity.angular.z, last_point_time);
    // ROS_INFO(
    //     "n+1 x=%.2f y=%.2f theta=%.2f lin=%.2f ang=%.2f time=%.2f ep=%.2f",
    //     next_point.transform.translation.x, next_point.transform.translation.y,
    //     tf::getYaw(next_point.transform.rotation), next_point.velocity.linear.x,
    //     next_point.velocity.angular.z, next_point_time, ep_time);

    if (ep_time > EP_TIME_EPS) {
      if (last_point_time >= 0.0 && next_point_time >= 0.0) {
        // interpolate pose and velocity
        double time_ratio =
            std::min(ep_time / (next_point_time - last_point_time), 1.0);
        last_point.transform.translation.x +=
            (next_point.transform.translation.x -
             last_point.transform.translation.x) *
            time_ratio;
        last_point.transform.translation.y +=
            (next_point.transform.translation.y -
             last_point.transform.translation.y) *
            time_ratio;
        last_point.transform.rotation = tf::createQuaternionMsgFromYaw(
            tf::getYaw(last_point.transform.rotation) +
            angles::shortest_angular_distance(
                tf::getYaw(last_point.transform.rotation),
                tf::getYaw(next_point.transform.rotation)) *
                time_ratio);
        last_point.velocity.linear.x +=
            (next_point.velocity.linear.x - last_point.velocity.linear.x) *
            time_ratio;
        last_point.velocity.angular.z +=
            (next_point.velocity.angular.z - last_point.velocity.angular.z) *
            time_ratio;
        last_point.time_from_start.fromSec(
            last_point.time_from_start.toSec() +
            (next_point.time_from_start.toSec() -
             last_point.time_from_start.toSec()) *
                time_ratio);
        // ROS_INFO("tr=%.2f", time_ratio);

        // // here we first get interpolated velocity linear_dist =
        // std::hypot(next_point.transform.translation.x -
        //                last_point.transform.translation.x,
        //            next_point.transform.translation.y -
        //                last_point.transform.translation.y);
        // last_point.velocity.linear.x +=
        //     (next_point.velocity.linear.x - last_point.velocity.linear.x) *
        //     ep_time;
        // last_point.velocity.angular.z +=
        //     (next_point.velocity.angular.z - last_point.velocity.angular.z) *
        //     ep_time;
        // double can_lin_dist = last_point.velocity.linear.x * ep_time;
        // double ratio_lin_dist = std::min(can_lin_dist / linear_dist, 1.0);

        // last_point.transform.translation.x +=
        //     (next_point.transform.translation.x -
        //      last_point.transform.translation.x) *
        //     ratio_lin_dist;
        // last_point.transform.translation.y +=
        //     (next_point.transform.translation.y -
        //      last_point.transform.translation.y) *
        //     ratio_lin_dist;

        // angular_dist = angles::shortest_angular_distance(
        //     tf::getYaw(last_point.transform.rotation),
        //     tf::getYaw(next_point.transform.rotation));
        // double can_ang_dist = next_point.velocity.angular.z * ep_time;
        // double ratio_ang_dist =
        //     std::min(can_ang_dist / std::abs(angular_dist), 1.0);
        // last_point.transform.rotation = tf::createQuaternionMsgFromYaw(
        //     tf::getYaw(last_point.transform.rotation) +
        //     (angular_dist * ratio_ang_dist));
      } else {
        // assuming maximum velocities
        linear_dist = std::hypot(next_point.transform.translation.x -
                                     last_point.transform.translation.x,
                                 next_point.transform.translation.y -
                                     last_point.transform.translation.y);
        double can_lin_dist = last_config_.max_linear_vel * ep_time;
        double ratio_lin_dist = std::min(can_lin_dist / linear_dist, 1.0);
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
        double can_ang_dist = last_config_.max_angular_vel * ep_time;
        double ratio_ang_dist =
            std::min(can_ang_dist / std::abs(angular_dist), 1.0);
        last_point.transform.rotation = tf::createQuaternionMsgFromYaw(
            tf::getYaw(last_point.transform.rotation) +
            (angular_dist * ratio_ang_dist));
        // ROS_INFO("ad=%.2f, cad=%.2f, rad=%.2f", angular_dist, can_ang_dist,
        //          ratio_ang_dist);

        // we calculate velocities from distance to last updated point
        linear_dist = std::hypot(last_point.transform.translation.x -
                                     last_traj_point.transform.translation.x,
                                 last_point.transform.translation.y -
                                     last_traj_point.transform.translation.y);
        last_point.velocity.linear.x = linear_dist / cycle_time;
        angular_dist = tf::getYaw(last_point.transform.rotation) -
                       tf::getYaw(last_traj_point.transform.rotation);
        last_point.velocity.angular.z = angular_dist / cycle_time;

        last_point.time_from_start.fromSec(-1.0);
      }
    }

    // ROS_INFO(
    //     "new x=%.2f y=%.2f theta=%.2f lin=%.2f ang=%.2f time=%.2f\n",
    //     last_point.transform.translation.x, last_point.transform.translation.y,
    //     tf::getYaw(last_point.transform.rotation), last_point.velocity.linear.x,
    //     last_point.velocity.angular.z, last_point.time_from_start.toSec());

    last_traj_points_[human_id] = last_point;
    if (next_point_index != 0) {
      transformed_traj.points.erase(transformed_traj.points.begin(),
                                    transformed_traj.points.begin() +
                                        next_point_index - 1);
    }
    transformed_traj.points.insert(transformed_traj.points.begin(), last_point);
  }

  // velocity control mode
  for (auto &vel_kv : vels_) {
    auto last_traj_points_it = last_traj_points_.find(vel_kv.first);
    auto &vel = vel_kv.second;
    if (last_traj_points_it != last_traj_points_.end()) {
      auto &last_traj_point = last_traj_points_it->second;
      if (std::abs(vel.angular.z) > ANG_VEL_EPS) {
        double r = std::hypot(vel.linear.x, vel.linear.y) / vel.angular.z;
        double theta = vel.angular.z * cycle_time;
        double crd = r * 2 * std::sin(theta / 2);
        double alpha = std::atan2(vel.linear.y, vel.linear.x) + (theta / 2);
        last_traj_point.transform.translation.x += (crd * std::cos(alpha));
        last_traj_point.transform.translation.y += (crd * std::sin(alpha));
        last_traj_point.transform.rotation = tf::createQuaternionMsgFromYaw(
            tf::getYaw(last_traj_point.transform.rotation) + theta);
      } else {
        last_traj_point.transform.translation.x += (vel.linear.x * cycle_time);
        last_traj_point.transform.translation.y += (vel.linear.y * cycle_time);
      }

      // ROS_INFO("new x=%.2f y=%.2f theta=%.2f lin=%.2f ang=%.2f time=%.2f",
      //          last_traj_point.transform.translation.x,
      //          last_traj_point.transform.translation.y,
      //          tf::getYaw(last_traj_point.transform.rotation),
      //          last_traj_point.velocity.linear.x,
      //          last_traj_point.velocity.angular.z,
      //          last_traj_point.time_from_start.toSec());
    } else {
      ROS_WARN_THROTTLE_NAMED(THROTTLE_TIME, NODE_NAME,
                              "trying to twist human without plans");
    }
  }

  humans = last_traj_points_;

  for (auto &human_id : reached_goals_) {
    last_traj_points_.erase(human_id);
    transformed_trajs.erase(human_id);
    plans_.erase(human_id);
    trajs_.erase(human_id);
  }

  if (last_traj_points_.empty()) {
    reset_time_ = true;
  }

  publishPlansFromTrajs(transformed_trajs);
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

bool TeleportController::transformPlansAndTrajs(
    const move_humans::map_pose_vector &plans,
    const move_humans::map_trajectory &trajs,
    move_humans::map_trajectory &transformed_trajs) {
  transformed_trajs.clear();

  int skipped = 0;

  for (auto &plan_kv : plans) {
    auto &human_id = plan_kv.first;
    auto &plan = plan_kv.second;

    // skip if velocity mode is activated for this human
    if (vels_.find(human_id) != vels_.end()) {
      skipped++;
      continue;
    }

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
      ROS_DEBUG_NAMED(
          NODE_NAME,
          "Giving pre-transformed trajectory (from plan) for human %ld",
          human_id);
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
        ROS_DEBUG_NAMED(
            NODE_NAME,
            "Giving new transformed trajectory (from plan) for human %ld",
            human_id);
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
      ROS_DEBUG_NAMED(NODE_NAME,
                     "Giving converted trajectory (from plan) for human %ld",
                     human_id);
    }
  }

  for (auto &traj_kv : trajs) {
    auto &human_id = traj_kv.first;
    auto &traj = traj_kv.second;

    // skip if velocity mode is activated for this human
    if (vels_.find(human_id) != vels_.end()) {
      skipped++;
      continue;
    }

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
      ROS_DEBUG_NAMED(
          NODE_NAME,
          "Giving pre-transformed trajectory (from traj) for human %ld",
          human_id);
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
        ROS_DEBUG_NAMED(
            NODE_NAME,
            "Giving transformed trajectory (from traj) for human %ld",
            human_id);
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
      transformed_trajs[human_id] = traj;
      last_transformed_trajs_[human_id] = traj;
      ROS_DEBUG_NAMED(NODE_NAME,
                     "Giving converted trajectory (from traj) for human %ld",
                     human_id);
    }
  }

  return (plans.size() + trajs.size() == transformed_trajs.size() + skipped);
}

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
    projectPoint(traj.points[np_index].transform.translation,
                 traj.points[np_index - 1].transform.translation, pose,
                 projected_pose);
    next_pose_index = np_index;
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

  return (val_dp > 0 && val_dp < lene1_sq);
}

void TeleportController::publishPlansFromTrajs(
    const move_humans::map_trajectory &trajs) {
  if (!last_config_.publish_plans) {
    return;
  }

  auto now = ros::Time::now();
  hanp_msgs::HumanPathArray human_path_array;
  for (auto &traj_kv : trajs) {
    if (!traj_kv.second.points.empty()) {
      hanp_msgs::HumanPath human_path;
      human_path.header.stamp = now;
      human_path.header.frame_id = controller_frame_;
      human_path.id = traj_kv.first;
      human_path.path.header.stamp = now;
      human_path.path.header.frame_id = controller_frame_;
      for (auto &traj_point : traj_kv.second.points) {
        geometry_msgs::PoseStamped pose;
        pose.header.stamp = now;
        pose.header.frame_id = controller_frame_;
        pose.pose.position.x = traj_point.transform.translation.x;
        pose.pose.position.y = traj_point.transform.translation.y;
        pose.pose.position.z = traj_point.transform.translation.z;
        pose.pose.orientation = traj_point.transform.rotation;
        human_path.path.poses.push_back(pose);
      }
      human_path_array.paths.push_back(human_path);
    }
  }
  if (!human_path_array.paths.empty()) {
    human_path_array.header.stamp = now;
    human_path_array.header.frame_id = controller_frame_;
    plans_pub_.publish(human_path_array);
  }
}
};
