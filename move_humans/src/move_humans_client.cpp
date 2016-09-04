#define NODE_NAME "move_humans_client"
#define TIMEOUT 30.0
#define FRAME_ID "map"
#define RESET_SIMULATION_SERVICE_NAME "reset_simulation"
#define ADD_HUMAN_SERVICE_NAME "add_human"
#define DELETE_HUMAN_SERVICE_NAME "delete_human"
#define ADD_SUBGOAL_SERVICE_NAME "add_sub_goal"
#define UPDATE_GOAL_SERVICE_NAME "update_goal"
#define SUBGOAL_REACHING_THRESHOLD 0.1 // m

#include "move_humans/move_humans_client.h"

namespace move_humans {
MoveHumansClient::MoveHumansClient(tf::TransformListener &tf) : tf_(tf) {
  ros::NodeHandle private_nh("~");

  mhac_ = new MoveHumansActionClient("/move_humans_node/action_server", true);

  private_nh.param("frame_id", frame_id_, std::string(FRAME_ID));

  private_nh.param("reset_simulation_service_name",
                   reset_simulation_service_name_,
                   std::string(RESET_SIMULATION_SERVICE_NAME));
  private_nh.param("add_human_service_name", add_human_service_name_,
                   std::string(ADD_HUMAN_SERVICE_NAME));
  private_nh.param("delete_human_service_name", delete_human_service_name_,
                   std::string(DELETE_HUMAN_SERVICE_NAME));
  private_nh.param("add_subgoal_service_name", add_subgoal_service_name_,
                   std::string(ADD_SUBGOAL_SERVICE_NAME));
  private_nh.param("update_goal_service_name", update_goal_service_name_,
                   std::string(UPDATE_GOAL_SERVICE_NAME));

  if (!getHumansGoals(private_nh, starts_, goals_)) {
    ROS_ERROR_NAMED(
        NODE_NAME,
        "Failed to read human start-goal positions from the parameter server");
    starts_.clear();
    goals_.clear();
  }

  reset_simulation_server_ = private_nh.advertiseService(
      reset_simulation_service_name_, &MoveHumansClient::resetSimulation, this);
  add_human_server_ = private_nh.advertiseService(
      add_human_service_name_, &MoveHumansClient::addHuman, this);
  delete_human_server_ = private_nh.advertiseService(
      delete_human_service_name_, &MoveHumansClient::deleteHuman, this);
  add_subgoal_server_ = private_nh.advertiseService(
      add_subgoal_service_name_, &MoveHumansClient::addSubgoal, this);
  update_goal_server_ = private_nh.advertiseService(
      update_goal_service_name_, &MoveHumansClient::updateGoal, this);

  client_thread_ =
      new boost::thread(boost::bind(&MoveHumansClient::clientThread, this));
}

MoveHumansClient::~MoveHumansClient() {
  delete mhac_;

  client_thread_->interrupt();
  client_thread_->join();
  delete client_thread_;
}

void MoveHumansClient::clientThread() {
  ROS_INFO_NAMED(NODE_NAME, "Waiting for move_humans server to start");
  mhac_->waitForServer();
  ROS_INFO_NAMED(NODE_NAME, "Connected to move_humans server");

  boost::unique_lock<boost::mutex> lock(client_mutex_);
  move_humans::MoveHumansGoal goal;
  for (auto &start_kv : starts_) {
    move_humans::HumanPose start_human_pose;
    start_human_pose.human_id = start_kv.first;
    start_human_pose.pose = start_kv.second;
    goal.start_poses.push_back(start_human_pose);
  }
  for (auto &goal_kv : goals_) {
    move_humans::HumanPose goal_human_pose;
    goal_human_pose.human_id = goal_kv.first;
    goal_human_pose.pose = goal_kv.second;
    goal.goal_poses.push_back(goal_human_pose);
  }
  lock.unlock();

  ROS_INFO_NAMED(NODE_NAME, "Seding initial goals");
  mhac_->sendGoal(goal, MoveHumansActionClient::SimpleDoneCallback(),
                  MoveHumansActionClient::SimpleActiveCallback(),
                  boost::bind(&MoveHumansClient::feedbackCB, this, _1));

  ros::NodeHandle nh;
  lock.lock();
  while (nh.ok()) {
    client_cond_.wait(lock);

    move_humans::MoveHumansGoal goal;
    for (auto &start_kv : starts_) {
      move_humans::HumanPose start_human_pose;
      start_human_pose.human_id = start_kv.first;
      start_human_pose.pose = start_kv.second;
      goal.start_poses.push_back(start_human_pose);
    }
    for (auto &goal_kv : goals_) {
      move_humans::HumanPose goal_human_pose;
      goal_human_pose.human_id = goal_kv.first;
      goal_human_pose.pose = goal_kv.second;
      goal.goal_poses.push_back(goal_human_pose);
    }
    for (auto &sub_goals_kv : sub_goals_) {
      move_humans::HumanPoseArray sub_goals_human_pose_array;
      sub_goals_human_pose_array.human_id = sub_goals_kv.first;
      sub_goals_human_pose_array.poses = sub_goals_kv.second;
      goal.sub_goal_poses.push_back(sub_goals_human_pose_array);
    }
    lock.unlock();

    ROS_INFO_NAMED(NODE_NAME, "Seding new goals");
    mhac_->sendGoal(goal, MoveHumansActionClient::SimpleDoneCallback(),
                    MoveHumansActionClient::SimpleActiveCallback(),
                    boost::bind(&MoveHumansClient::feedbackCB, this, _1));
    lock.lock();
  }

  // bool finished_before_timeout =
  // mhac_->waitForResult(ros::Duration(TIMEOUT));
  // if (finished_before_timeout) {
  //   auto state = mhac_->getState();
  //   ROS_INFO_NAMED(NODE_NAME, "move_humans goals reached: %s",
  //                  state.toString().c_str());
  // } else {
  //   ROS_INFO_NAMED(NODE_NAME, "move_humans did not finish before timeout");
  // }
}

bool MoveHumansClient::getHumansGoals(ros::NodeHandle &nh,
                                      move_humans::map_pose &starts,
                                      move_humans::map_pose &goals) {
  XmlRpc::XmlRpcValue humans;
  if (!nh.getParam("humans", humans) &&
      humans.getType() != XmlRpc::XmlRpcValue::TypeArray) {
    return false;
  }

  for (auto i = 0; i < humans.size(); i++) {
    // ROS_ASSERT(humans[i].getType() == XmlRpc::XmlRpcValue::TypeStruct);
    if (humans[i].getType() != XmlRpc::XmlRpcValue::TypeStruct) {
      ROS_ERROR_NAMED(NODE_NAME, "Wrong human position data");
      continue;
    }

    bool got_id = false, got_start = false, got_goal = false;
    move_humans::HumanPose start_pose, goal_pose;

    for (auto it = humans[i].begin(); it != humans[i].end(); ++it) {
      if (!it->first.compare("id")) {
        // ROS_ASSERT(it->second.getType() == XmlRpc::XmlRpcValue::TypeInt);
        if (it->second.getType() == XmlRpc::XmlRpcValue::TypeInt) {
          auto human_id = (int)(it->second);
          if (human_id >= 0) {
            start_pose.human_id = human_id;
            goal_pose.human_id = human_id;
          }
          got_id = true;
        }
      } else if (!it->first.compare("start")) {
        // ROS_ASSERT(it->second.getType() == XmlRpc::XmlRpcValue::TypeArray);
        // ROS_ASSERT(it->second.size() == 3);
        if (it->second.getType() == XmlRpc::XmlRpcValue::TypeArray &&
            it->second.size() == 3) {
          bool are_doubles = true;
          for (auto j = 0; j < it->second.size(); ++j) {
            // ROS_ASSERT(it->second[j].getType() ==
            // XmlRpc::XmlRpcValue::TypeDouble);
            if (it->second[j].getType() != XmlRpc::XmlRpcValue::TypeDouble) {
              are_doubles = false;
            }
          }
          if (are_doubles) {
            start_pose.pose.header.frame_id = frame_id_;
            start_pose.pose.pose.position.x = (double)(it->second[0]);
            start_pose.pose.pose.position.y = (double)(it->second[1]);
            start_pose.pose.pose.position.z = 0;
            start_pose.pose.pose.orientation =
                tf::createQuaternionMsgFromYaw((double)(it->second[2]));
            got_start = true;
          }
        }
      } else if (!it->first.compare("end")) {
        // ROS_ASSERT(it->second.getType() == XmlRpc::XmlRpcValue::TypeArray);
        // ROS_ASSERT(it->second.size() == 3);
        if (it->second.getType() == XmlRpc::XmlRpcValue::TypeArray &&
            it->second.size() == 3) {
          bool are_doubles = true;
          for (auto j = 0; j < it->second.size(); ++j) {
            // ROS_ASSERT(it->second[j].getType() ==
            // XmlRpc::XmlRpcValue::TypeDouble);
            if (it->second[j].getType() != XmlRpc::XmlRpcValue::TypeDouble) {
              are_doubles = false;
            }
          }
          if (are_doubles) {
            goal_pose.pose.header.frame_id = frame_id_;
            goal_pose.pose.pose.position.x = (double)(it->second[0]);
            goal_pose.pose.pose.position.y = (double)(it->second[1]);
            goal_pose.pose.pose.position.z = 0;
            goal_pose.pose.pose.orientation =
                tf::createQuaternionMsgFromYaw((double)(it->second[2]));
            got_goal = true;
          }
        }
      } else {
        ROS_ERROR_NAMED(NODE_NAME,
                        "Could not process %s identifier for human positions",
                        ((std::string)(it->first)).c_str());
        continue;
      }
    }

    if (!got_id || !got_start || !got_goal) {
      ROS_ERROR_NAMED(NODE_NAME, "Incomplete or wrong human position data");
      continue;
    }

    starts[start_pose.human_id] = start_pose.pose;
    goals[goal_pose.human_id] = goal_pose.pose;
  }

  return true;
}

bool MoveHumansClient::resetSimulation(std_srvs::Trigger::Request &req,
                                       std_srvs::Trigger::Response &res) {
  ros::NodeHandle private_nh("~");
  boost::unique_lock<boost::mutex> lock(client_mutex_);
  std::stringstream message;
  sub_goals_.clear();
  starts_.clear();
  goals_.clear();
  if (!getHumansGoals(private_nh, starts_, goals_)) {
    message << "Failed to read human start-goal positions from the parameter "
               "server";
    ROS_ERROR_STREAM_NAMED(NODE_NAME, message);
    starts_.clear();
    goals_.clear();
    res.message = message.str();
    res.success = false;
  } else {
    res.success = true;
    client_cond_.notify_one();
  }
  return true;
}

bool MoveHumansClient::addHuman(move_humans::HumanUpdate::Request &req,
                                move_humans::HumanUpdate::Response &res) {
  boost::unique_lock<boost::mutex> lock(client_mutex_);
  std::stringstream message;
  if (starts_.find(req.human_pose.human_id) != starts_.end()) {
    message << "Human " << req.human_pose.human_id << " already exists";
    ROS_WARN_STREAM_NAMED(NODE_NAME, message);
    res.message = message.str();
    res.success = false;
  } else {
    starts_[req.human_pose.human_id] = req.human_pose.pose;
    message << "Added human " << req.human_pose.human_id;
    ROS_INFO_STREAM_NAMED(NODE_NAME, message);
    res.message = message.str();
    res.success = true;
  }
  return true;
}

bool MoveHumansClient::deleteHuman(move_humans::HumanUpdate::Request &req,
                                   move_humans::HumanUpdate::Response &res) {
  boost::unique_lock<boost::mutex> lock(client_mutex_);
  std::stringstream message;
  starts_.erase(req.human_pose.human_id);
  goals_.erase(req.human_pose.human_id);
  sub_goals_.erase(req.human_pose.human_id);
  client_cond_.notify_one();
  message << "Deleted human " << req.human_pose.human_id;
  ROS_INFO_STREAM_NAMED(NODE_NAME, message);
  res.message = message.str();
  res.success = true;
  return true;
}

bool MoveHumansClient::addSubgoal(move_humans::HumanUpdate::Request &req,
                                  move_humans::HumanUpdate::Response &res) {
  boost::unique_lock<boost::mutex> lock(client_mutex_);
  std::stringstream message;
  if (starts_.find(req.human_pose.human_id) == starts_.end() ||
      goals_.find(req.human_pose.human_id) == goals_.end()) {
    message << "Human " << req.human_pose.human_id
            << "does not exists in the database, please first add human "
            << req.human_pose.human_id << " with a start pose";
    ROS_ERROR_STREAM_NAMED(NODE_NAME, message);
    res.message = message.str();
    res.success = false;
  } else if (sub_goals_.find(req.human_pose.human_id) != sub_goals_.end()) {
    sub_goals_[req.human_pose.human_id].push_back(req.human_pose.pose);
    message << "Added another sub-goal pose for human"
            << req.human_pose.human_id;
    ROS_INFO_STREAM_NAMED(NODE_NAME, message);
    res.message = message.str();
    res.success = true;
  } else {
    move_humans::pose_vector sub_goal_vector;
    sub_goal_vector.push_back(req.human_pose.pose);
    sub_goals_[req.human_pose.human_id] = sub_goal_vector;
    message << "Added sub-goal pose for human" << req.human_pose.human_id;
    ROS_INFO_STREAM_NAMED(NODE_NAME, message);
    res.message = message.str();
    res.success = true;
  }
  client_cond_.notify_one();
  return true;
}

bool MoveHumansClient::updateGoal(move_humans::HumanUpdate::Request &req,
                                  move_humans::HumanUpdate::Response &res) {
  boost::unique_lock<boost::mutex> lock(client_mutex_);
  std::stringstream message;
  if (starts_.find(req.human_pose.human_id) == starts_.end()) {
    message << "Human " << req.human_pose.human_id
            << " does not exists in the database, please first add human "
            << req.human_pose.human_id << " with a start pose";
    ROS_ERROR_STREAM_NAMED(NODE_NAME, message);
    res.message = message.str();
    res.success = false;
  } else if (goals_.find(req.human_pose.human_id) != goals_.end()) {
    goals_[req.human_pose.human_id] = req.human_pose.pose;
    message << "Updated goal pose for human " << req.human_pose.human_id;
    ROS_INFO_STREAM_NAMED(NODE_NAME, message);
    res.message = message.str();
    res.success = true;
  } else {
    goals_[req.human_pose.human_id] = req.human_pose.pose;
    message << "Added goal pose for human " << req.human_pose.human_id;
    ROS_INFO_STREAM_NAMED(NODE_NAME, message);
    res.message = message.str();
    res.success = true;
  }
  client_cond_.notify_one();
  return true;
}

void MoveHumansClient::feedbackCB(const MoveHumansFeedbackConstPtr &feedback) {
  boost::unique_lock<boost::mutex> lock(client_mutex_);
  for (auto &current_pose : feedback->current_poses) {
    starts_[current_pose.human_id] = current_pose.pose;

    auto sub_goals_it = sub_goals_.find(current_pose.human_id);
    if (sub_goals_it != sub_goals_.end()) {
      auto sub_goals = sub_goals_it->second;
      if (sub_goals.size() > 0) {
        auto &sub_goal_pose = sub_goals.front();
        double dist = hypot(
            sub_goal_pose.pose.position.x - current_pose.pose.pose.position.x,
            sub_goal_pose.pose.position.y - current_pose.pose.pose.position.y);
        if (dist < SUBGOAL_REACHING_THRESHOLD) {
          sub_goals_.erase(sub_goals_it);
          ROS_DEBUG_NAMED(NODE_NAME, "Erased a sub goal for %ld",
                          current_pose.human_id);
        }
      }
    }
  }
}

} // namespace move_humans
