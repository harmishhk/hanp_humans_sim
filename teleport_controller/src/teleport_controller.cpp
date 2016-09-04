#define NODE_NAME "teleport_controller"
#define DIST_THRESHOLD 0.2 // distance threshold for pruning the plan
#define MAX_LINEAR_VEL 0.7 // m/s
#define HUMAN_RADIUS 0.25  // m
#define DEFAUTL_SEGMENT_TYPE hanp_msgs::TrackedSegmentType::TORSO
#define PLANS_PUB_TOPIC "plans"
#define HUMANS_PUB_TOPIC "humans"
#define HUMANS_MARKERS_PUB_TOPIC "human_markers"
#define CONTROLLER_PLANS_SUB_TOPIC "human_plans"
#define PUBLISH_HUMAN_MARKERS true
#define HUMANS_ARROWS_ID_OFFSET 100
#define HUMANS_CYLINDERS_HEIGHT 1.5
#define HUMAN_COLOR_R 0.5
#define HUMAN_COLOR_G 0.5
#define HUMAN_COLOR_B 0.0
#define MARKER_LIFETIME 4.0

#include <pluginlib/class_list_macros.h>
#include <hanp_msgs/TrackedHumans.h>
#include <hanp_msgs/TrackedSegmentType.h>
#include <visualization_msgs/MarkerArray.h>

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

    double dist_threshold = DIST_THRESHOLD;
    private_nh.param("dist_threshold", dist_threshold);
    sq_dist_threshold_ = dist_threshold * dist_threshold;
    private_nh.param("max_linear_vel", max_linear_vel_, MAX_LINEAR_VEL);
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

    controller_plans_sub_ =
        private_nh.subscribe(CONTROLLER_PLANS_SUB_TOPIC, 1,
                             &TeleportController::controllerPlansCB, this);

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
  if (!isInitialized()) {
    ROS_ERROR_NAMED(NODE_NAME, "This controller has not been initialized");
    return false;
  }

  ROS_INFO_NAMED(NODE_NAME, "Got new plan for %ld human%s", plans.size(),
                 plans.size() > 1 ? "s" : "");

  for (auto &plan_kv : plans) {
    auto &human_id = plan_kv.first;
    auto &plan = plan_kv.second;

    // TODO: just remove ids for new plans from reached_goals_
    reached_goals_.clear();
    last_human_poses_.erase(human_id);
    plans_[human_id] = plan;
  }

  return true;
}

bool TeleportController::computeHumansStates(move_humans::map_pose &humans) {
  move_humans::map_pose_vector transformed_plans;
  if (!transformPlans(plans_, last_human_poses_, transformed_plans)) {
    ROS_ERROR_NAMED(NODE_NAME, "Cannot transform plans to controller frame");
    return false;
  }

  auto now = ros::Time::now();
  for (auto &plan_kv : transformed_plans) {
    auto &human_id = plan_kv.first;
    auto &transformed_plan = plan_kv.second;
    if (transformed_plan.empty()) {
      ROS_ERROR_NAMED(NODE_NAME, "Transformed plan is empty for human %ld",
                      human_id);
      reached_goals_[human_id] = true;
      continue;
    }
    auto now = ros::Time::now();
    if (last_human_poses_.find(human_id) == last_human_poses_.end()) {
      auto human_pose = transformed_plan.front();
      human_pose.header.stamp = now;
      last_human_poses_[human_id] = human_pose;
      continue;
    }
    auto &last_pose = last_human_poses_.at(human_id);

    auto time_diff = now - last_pose.header.stamp;
    double traveled_dist = max_linear_vel_ * time_diff.toSec();
    // double sq_traveled_dist = traveled_dist * traveled_dist;

    unsigned int i = (unsigned int)transformed_plan.size() - 1;
    while (i > 0) {
      // auto sq_dist = dist_sq(transformed_plan[i].pose, last_pose.pose);
      double point_dist = std::hypot(
          (transformed_plan[i].pose.position.x - last_pose.pose.position.x),
          (transformed_plan[i].pose.position.y - last_pose.pose.position.y));
      // if (sq_dist < sq_traveled_dist) {
      if (point_dist < traveled_dist) { // or the point is bbehind
        break;
      } else {
        --i;
      }
    }

    if (i == ((unsigned int)transformed_plan.size() - 1)) {
      reached_goals_.push_back(human_id);
      plans_.erase(human_id);
      auto current_pose = transformed_plan.back();
      current_pose.header.stamp = now;
      last_human_poses_[human_id] = current_pose;
      ROS_INFO_NAMED(NODE_NAME, "Human %ld reached its goal", human_id);
    } else {
      // auto &near_pose = transformed_plan[i];
      // auto &far_pose = transformed_plan[i + 1];1
      auto &far_pose = transformed_plan[i + 1];
      // double np_dist =
      //     std::hypot((near_pose.pose.position.x - last_pose.pose.position.x),
      //                (near_pose.pose.position.y -
      //                last_pose.pose.position.y));
      double fp_dist =
          std::hypot((far_pose.pose.position.x - last_pose.pose.position.x),
                     (far_pose.pose.position.y - last_pose.pose.position.y));
      // double ratio = (traveled_dist - np_dist) / (fp_dist - np_dist);
      double ratio = traveled_dist / fp_dist;
      // ROS_INFO_COND(human_id == 1,
      //               "td=%.4f, fpd=%.4f, ratio=%.4f, timediff= %.4f",
      //               traveled_dist, fp_dist, ratio, time_diff.toSec());
      // ROS_INFO_COND(human_id == 1, "lp=(%.3f,%.3f), fp=(%.3f,%.3f)",
      //               last_pose.pose.position.x, last_pose.pose.position.y,
      //               far_pose.pose.position.x, far_pose.pose.position.y);

      geometry_msgs::PoseStamped current_pose;
      // current_pose.pose.position.x =
      //     near_pose.pose.position.x +
      //     ratio * (far_pose.pose.position.x - near_pose.pose.position.x);
      // current_pose.pose.position.y =
      //     near_pose.pose.position.y +
      //     ratio * (far_pose.pose.position.y - near_pose.pose.position.y);
      current_pose.pose.position.x =
          last_pose.pose.position.x +
          ratio * (far_pose.pose.position.x - last_pose.pose.position.x);
      current_pose.pose.position.y =
          last_pose.pose.position.y +
          ratio * (far_pose.pose.position.y - last_pose.pose.position.y);
      current_pose.pose.position.z = 0;
      auto yaw =
          std::atan2(current_pose.pose.position.y - last_pose.pose.position.y,
                     current_pose.pose.position.x - last_pose.pose.position.x);
      current_pose.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);
      current_pose.header.stamp = now;
      current_pose.header.frame_id = controller_frame_;
      last_human_poses_[human_id] = current_pose;

      // ROS_INFO_COND(human_id == 1, "cp=(%.3f,%.3f)",
      //               current_pose.pose.position.x,
      //               current_pose.pose.position.y);
    }
  }

  for (auto &last_pose_kv : last_human_poses_) {
    humans[last_pose_kv.first] = last_pose_kv.second;
  }

  publishPlans(transformed_plans);
  publishHumans(last_human_poses_);
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
    const move_humans::map_pose &current_poses,
    move_humans::map_pose_vector &transformed_plans) {
  transformed_plans.clear();
  for (auto &plan_kv : plans) {
    auto &human_id = plan_kv.first;
    auto &plan = plan_kv.second;
    if (plan.empty()) {
      ROS_ERROR_NAMED(NODE_NAME, "Received empty plan for human %ld", human_id);
      continue;
    }

    auto &current_pose = (current_poses.find(human_id) == current_poses.end())
                             ? plan.front()
                             : current_poses.at(human_id);

    tf::Stamped<tf::Pose> cp_tf;
    tf::poseStampedMsgToTF(current_pose, cp_tf);

    if (plan[0].header.frame_id != controller_frame_ ||
        cp_tf.frame_id_ != controller_frame_) {
      try {
        tf::StampedTransform plan_to_controller_transform;
        tf_->waitForTransform(controller_frame_, plan[0].header.frame_id,
                              ros::Time(0), ros::Duration(0.5));
        tf_->lookupTransform(controller_frame_, plan[0].header.frame_id,
                             ros::Time(0), plan_to_controller_transform);

        cp_tf.setData(plan_to_controller_transform.inverse() * cp_tf);
        auto i = prunePlan(plan, cp_tf);

        tf::Stamped<tf::Pose> tf_pose;
        geometry_msgs::PoseStamped transformed_pose;
        move_humans::pose_vector transformed_plan;
        while (i < (unsigned int)plan.size()) {
          tf::poseStampedMsgToTF(plan[i], tf_pose);
          tf_pose.setData(plan_to_controller_transform * tf_pose);
          tf_pose.stamp_ = plan_to_controller_transform.stamp_;
          tf_pose.frame_id_ = controller_frame_;
          tf::poseStampedTFToMsg(tf_pose, transformed_pose);
          transformed_plan.push_back(transformed_pose);
        }
        transformed_plans[human_id] = transformed_plan;
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
      auto i = prunePlan(plan, cp_tf);
      move_humans::pose_vector transformed_plan(plan.begin() + i, plan.end());
      transformed_plans[human_id] = transformed_plan;
    }
  }

  return !transformed_plans.empty();
}

unsigned int TeleportController::prunePlan(const move_humans::pose_vector &plan,
                                           const tf::Stamped<tf::Pose> &cp_tf) {
  unsigned int i = plan.size() - 1;
  double x_diff, y_diff, sq_diff, sq_dist = std::numeric_limits<double>::max();
  while (i > 0) {
    x_diff = plan[i].pose.position.x - cp_tf.getOrigin().x();
    y_diff = plan[i].pose.position.y - cp_tf.getOrigin().y();
    sq_diff = x_diff * x_diff + y_diff * y_diff;
    if (sq_diff <= sq_dist_threshold_) {
      if (sq_diff > sq_dist) {
        break;
      }
      sq_dist = sq_diff;
    }
    --i;
  }
  return ((i + 2) > ((unsigned int)plan.size() - 1))
             ? ((unsigned int)plan.size() - 1)
             : (i + 2);
}

double TeleportController::dist_sq(const geometry_msgs::Pose &pose1,
                                   const geometry_msgs::Pose &pose2) {
  auto x_diff = pose1.position.x - pose2.position.x;
  auto y_diff = pose1.position.y - pose2.position.y;
  return (x_diff * x_diff + y_diff * y_diff);
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

void TeleportController::publishHumans(move_humans::map_pose &human_poses) {
  hanp_msgs::TrackedHumans humans;
  visualization_msgs::MarkerArray humans_markers;
  for (auto &pose_kv : human_poses) {
    hanp_msgs::TrackedSegment human_segment;
    human_segment.type = DEFAUTL_SEGMENT_TYPE;
    human_segment.pose.pose = pose_kv.second.pose;
    human_segment.pose.covariance[0] = human_radius_;
    human_segment.pose.covariance[7] = human_radius_;
    hanp_msgs::TrackedHuman human;
    human.track_id = pose_kv.first;
    human.segments.push_back(human_segment);

    humans.humans.push_back(human);

    if (publish_human_markers_) {
      visualization_msgs::Marker human_arrow, human_cylinder;

      human_arrow.header = pose_kv.second.header;
      human_arrow.type = visualization_msgs::Marker::ARROW;
      human_arrow.action = visualization_msgs::Marker::MODIFY;
      human_arrow.id = pose_kv.first + HUMANS_ARROWS_ID_OFFSET;
      human_arrow.pose = pose_kv.second.pose;
      human_arrow.scale.x = human_radius_ * 2.0;
      human_arrow.scale.y = 0.1;
      human_arrow.scale.z = 0.1;
      human_arrow.color.a = 1.0;
      human_arrow.color.r = HUMAN_COLOR_R;
      human_arrow.color.g = HUMAN_COLOR_G;
      human_arrow.color.b = HUMAN_COLOR_B;
      human_arrow.lifetime = ros::Duration(MARKER_LIFETIME);

      human_cylinder.header = pose_kv.second.header;
      human_cylinder.type = visualization_msgs::Marker::CYLINDER;
      human_cylinder.action = visualization_msgs::Marker::MODIFY;
      human_cylinder.id = pose_kv.first;
      human_cylinder.pose = pose_kv.second.pose;
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

void TeleportController::controllerPlansCB(
    const hanp_msgs::PathArray &path_array) {
  if (path_array.ids.size() != path_array.paths.size()) {
    ROS_ERROR_NAMED(NODE_NAME, "Erroneous path array message");
    return;
  }
  boost::mutex::scoped_lock l(controlling_mutex_);
  for (int i = 0; i < path_array.ids.size(); i++) {
    plans_[path_array.ids[i]] = path_array.paths[i].poses;
  }
}
};
