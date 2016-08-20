#define NODE_NAME "teleport_controller"

#include <teleport_controller/teleport_controller.h>
#include <pluginlib/class_list_macros.h>
#include <nav_msgs/Path.h>

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

    plan_pub_ = private_nh.advertise<nav_msgs::Path>("gloabl_plan", 1);
    cotrol_pub_ = private_nh.advertise<nav_msgs::Path>("local_plan", 1);

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

  ROS_INFO_NAMED(NODE_NAME, "Got new plans");
  return true;
}

bool TeleportController::computeHumansStates(move_humans::map_pose &humans) {
  return true;
}

bool TeleportController::areGoalsReached() {
  if (!isInitialized()) {
    ROS_ERROR_NAMED(NODE_NAME, "This controller has not been initialized");
    return false;
  }

  return true;
}
};
