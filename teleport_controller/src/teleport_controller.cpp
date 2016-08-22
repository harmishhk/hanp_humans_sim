#define NODE_NAME "teleport_controller"
#define DIST_THRESHOLD 0.5
#define MAX_LINEAR_VEL 0.01 // m/s
#define DEFAUTL_SEGMENT_TYPE hanp_msgs::TrackedSegmentType::TORSO
#define PLANS_PUB_TOPIC "plans"
#define HUMANS_PUB_TOPIC "humans"
#define HUMANS_POSES_PUB_TOPIC "humans_poses"
#define CONTROLLER_PLANS_SUB_TOPIC "human_plans"
#define VISUALIZE_HUMAN_POSES true

#include <teleport_controller/teleport_controller.h>
#include <pluginlib/class_list_macros.h>
#include <nav_msgs/Path.h>
#include <hanp_msgs/TrackedHumans.h>
#include <hanp_msgs/TrackedSegmentType.h>

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
    private_nh.param("visualize_human_poses", visualize_human_poses_,
                     VISUALIZE_HUMAN_POSES);

    plans_pub_ = private_nh.advertise<hanp_msgs::PathArray>(PLANS_PUB_TOPIC, 1);
    humans_pub_ =
        private_nh.advertise<hanp_msgs::TrackedHumans>(HUMANS_PUB_TOPIC, 1);
    if (visualize_human_poses_) {
      humans_poses_pub_ = private_nh.advertise<geometry_msgs::PoseArray>(
          HUMANS_POSES_PUB_TOPIC, 1);
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

  reached_goals_.clear();
  plans_.clear();
  plans_ = plans;

  last_human_poses_.clear();
  auto now = ros::Time::now();
  for (auto &plan_kv : plans_) {
    if (!plan_kv.second.empty()) {
      auto human_pose = plan_kv.second.front();
      human_pose.header.stamp = now;
      last_human_poses_[plan_kv.first] = human_pose;
    }
  }

  ROS_INFO_NAMED(NODE_NAME, "Got new plans");
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
    if (last_human_poses_.find(human_id) == last_human_poses_.end()) {
      ROS_ERROR_NAMED(NODE_NAME, "Last pose not found for human %ld", human_id);
      reached_goals_[human_id] = true;
      continue;
    }
    auto &last_pose = last_human_poses_.at(human_id);

    auto time_diff = ros::Time::now() - last_pose.header.stamp;
    auto traveled_dist = max_linear_vel_ * time_diff.toSec();
    auto sq_traveled_dist = traveled_dist * traveled_dist;

    unsigned int i = (unsigned int)transformed_plan.size() - 1;
    while (i > 0) {
      auto sq_dist = dist_sq(transformed_plan[i].pose, last_pose.pose);
      if (sq_dist < sq_traveled_dist) {
        break;
      } else {
        --i;
      }
    }

    if (i == ((unsigned int)transformed_plan.size() - 1)) {
      reached_goals_.push_back(human_id);
      plans_.erase(human_id);
      last_human_poses_[human_id] = transformed_plan.back();
      ROS_INFO_NAMED(NODE_NAME, "Human %ld reached its goal", human_id);
    } else {
      auto &near_pose = transformed_plan[i];
      auto &far_pose = transformed_plan[i + 1];
      auto sq_dist_far = dist_sq(far_pose.pose, last_pose.pose);
      auto sq_dist_near = dist_sq(near_pose.pose, last_pose.pose);
      auto ip_ratio =
          (sq_traveled_dist - sq_dist_near) / (sq_dist_far - sq_dist_near);

      auto current_pose = last_pose;
      current_pose.pose.position.x =
          near_pose.pose.position.x +
          ip_ratio * (far_pose.pose.position.x - near_pose.pose.position.x);
      current_pose.pose.position.y =
          near_pose.pose.position.y +
          ip_ratio * (far_pose.pose.position.y - near_pose.pose.position.y);
      auto yaw =
          std::atan2(current_pose.pose.position.y - last_pose.pose.position.y,
                     current_pose.pose.position.x - last_pose.pose.position.x);
      current_pose.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);
      last_human_poses_[human_id] = current_pose;
    }
  }

  for (auto &last_pose_kv : last_human_poses_) {
    humans[last_pose_kv.first] = last_pose_kv.second;
  }

  publishPlans(transformed_plans);
  publishHumans(last_human_poses_);
  return true;
}

bool TeleportController::areGoalsReached() {
  if (!isInitialized()) {
    ROS_ERROR_NAMED(NODE_NAME, "This controller has not been initialized");
    return false;
  }

  for (auto &plan_kv : plans_) {
    if (std::find(reached_goals_.begin(), reached_goals_.end(),
                  plan_kv.first) == reached_goals_.end()) {
      return false;
    }
  }
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
    if (current_poses.find(human_id) == current_poses.end()) {
      ROS_ERROR_NAMED(NODE_NAME, "No current pose found for human %ld",
                      human_id);
      continue;
    }
    tf::Stamped<tf::Pose> cp_tf, cp_side_tf;
    tf::poseStampedMsgToTF(current_poses.at(human_id), cp_tf);
    cp_side_tf.setData(cp_tf * tf::Transform(tf::Quaternion(0, 0, 0, 1),
                                             tf::Vector3(0, 0, 0)));

    if (plan[0].header.frame_id != controller_frame_ ||
        cp_tf.frame_id_ != controller_frame_) {
      try {
        tf::StampedTransform plan_to_controller_transform;
        tf_->waitForTransform(controller_frame_, plan[0].header.frame_id,
                              ros::Time(0), ros::Duration(0.5));
        tf_->lookupTransform(controller_frame_, plan[0].header.frame_id,
                             ros::Time(0), plan_to_controller_transform);

        cp_tf.setData(plan_to_controller_transform.inverse() * cp_tf);
        cp_side_tf.setData(plan_to_controller_transform.inverse() * cp_side_tf);
        auto i = prunePlan(plan, cp_tf, cp_side_tf);

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
      auto i = prunePlan(plan, cp_tf, cp_side_tf);
      move_humans::pose_vector transformed_plan(plan.begin() + i, plan.end());
      transformed_plans[human_id] = transformed_plan;
    }
  }

  return !transformed_plans.empty();
}

unsigned int
TeleportController::prunePlan(const move_humans::pose_vector &plan,
                              const tf::Stamped<tf::Pose> &cp_tf,
                              const tf::Stamped<tf::Pose> &cp_side_tf) {
  unsigned int i = 0;
  double sq_dist = 0.0, x_diff, y_diff;
  while (i < (unsigned int)plan.size()) {
    x_diff = cp_tf.getOrigin().x() - plan[i].pose.position.x;
    y_diff = cp_tf.getOrigin().y() - plan[i].pose.position.y;
    sq_dist = x_diff * x_diff + y_diff * y_diff;

    auto &ax = cp_tf.getOrigin().x();
    auto &ay = cp_tf.getOrigin().y();
    auto &bx = cp_tf.getOrigin().x();
    auto &by = cp_tf.getOrigin().y();
    auto &cx = plan[i].pose.position.x;
    auto &cy = plan[i].pose.position.y;
    auto is_behind = ((bx - ax) * (cy - ay) - (by - ay) * (cx - ax)) < 0;

    if (sq_dist <= sq_dist_threshold_ && !is_behind) {
      break;
    }
    ++i;
  }
  return i;
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
  for (auto &pose_kv : human_poses) {
    hanp_msgs::TrackedSegment human_segment;
    human_segment.type = DEFAUTL_SEGMENT_TYPE;
    human_segment.pose.pose = pose_kv.second.pose;

    hanp_msgs::TrackedHuman human;
    human.track_id = pose_kv.first;
    human.segments.push_back(human_segment);

    humans.humans.push_back(human);
  }
  if (!humans.humans.empty()) {
    humans.header.stamp = ros::Time::now();
    humans.header.frame_id = controller_frame_;
    humans_pub_.publish(humans);

    if (visualize_human_poses_) {
      geometry_msgs::PoseArray human_pose_array;
      human_pose_array.header = humans.header;
      for (auto &pose_kv : human_poses) {
        human_pose_array.poses.push_back(pose_kv.second.pose);
      }
      humans_poses_pub_.publish(human_pose_array);
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
