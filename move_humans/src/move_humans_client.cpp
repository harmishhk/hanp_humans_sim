#define NODE_NAME "move_humans_client"
#define TIMEOUT 30.0
#define FRAME_ID "map"
#define NEW_HUMAN_SUB_TOPIC "add_human"
#define GOAL_SUB_TOPIC "add_human_goal"
#define SUB_GOAL_SUB_TOPIC "add_sub_goal"
#define REMOVE_HUMAN_SUB_TOPIC "remove_human"
#include <move_humans/move_humans_client.h>

#include <boost/thread.hpp>

namespace move_humans {
MoveHumansClient::MoveHumansClient(tf::TransformListener &tf) : tf_(tf) {
  ros::NodeHandle private_nh("~");

  mhac_ = new MoveHumansActionClient("/move_humans_node/action_server", true);

  private_nh.param("new_human_sub_topic", new_human_sub_topic_,
                   std::string(NEW_HUMAN_SUB_TOPIC));
  private_nh.param("goal_sub_topic", goal_sub_topic_,
                   std::string(GOAL_SUB_TOPIC));
  private_nh.param("sub_goal_sub_topic", sub_goal_sub_topic_,
                   std::string(SUB_GOAL_SUB_TOPIC));
  private_nh.param("remove_human_sub_topic", remove_human_sub_topic_,
                   std::string(REMOVE_HUMAN_SUB_TOPIC));
  private_nh.param("frame_id", frame_id_, std::string(FRAME_ID));

  if (!getHumansGoals(private_nh, starts_, goals_)) {
    ROS_ERROR_NAMED(
        NODE_NAME,
        "Failed to read human start-goal positions from the parameter server");
    starts_.clear();
    goals_.clear();
  }

  new_human_sub_ = private_nh.subscribe(new_human_sub_topic_, 1,
                                        &MoveHumansClient::newHumanCB, this);
  goal_sub_ =
      private_nh.subscribe(goal_sub_topic_, 1, &MoveHumansClient::goalCB, this);
  sub_goal_sub_ = private_nh.subscribe(sub_goal_sub_topic_, 1,
                                       &MoveHumansClient::subGoalCB, this);
  remove_human_sub_ = private_nh.subscribe(
      remove_human_sub_topic_, 1, &MoveHumansClient::removeHumanCB, this);

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
  mhac_->sendGoal(goal);

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
    mhac_->sendGoal(goal);
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

void MoveHumansClient::newHumanCB(const move_humans::HumanPose &new_human) {
  boost::unique_lock<boost::mutex> lock(client_mutex_);
  if (starts_.find(new_human.human_id) != starts_.end()) {
    ROS_WARN_NAMED(NODE_NAME, "Human %ld already exists", new_human.human_id);
    return;
  }
  starts_[new_human.human_id] = new_human.pose;
  ROS_INFO_NAMED(NODE_NAME, "Added human %ld", new_human.human_id);
}

void MoveHumansClient::goalCB(const move_humans::HumanPose &goal) {
  boost::unique_lock<boost::mutex> lock(client_mutex_);
  if (starts_.find(goal.human_id) == starts_.end()) {
    ROS_ERROR_NAMED(NODE_NAME, "Human %ld does not exists in the database, "
                               "please first add human %ld with a start pose",
                    goal.human_id, goal.human_id);
    return;
  }
  if (goals_.find(goal.human_id) != goals_.end()) {
    ROS_INFO_NAMED(NODE_NAME, "Redefining goal pose for human %ld",
                   goal.human_id);
  } else {
    ROS_INFO_NAMED(NODE_NAME, "Adding goal pose for human %ld", goal.human_id);
  }
  goals_[goal.human_id] = goal.pose;
  client_cond_.notify_one();
}

void MoveHumansClient::subGoalCB(const move_humans::HumanPose &sub_goal) {
  boost::unique_lock<boost::mutex> lock(client_mutex_);
  if (starts_.find(sub_goal.human_id) == starts_.end() ||
      goals_.find(sub_goal.human_id) == goals_.end()) {
    ROS_ERROR_NAMED(NODE_NAME, "Human %ld does not exists in the database, "
                               "please first add human %ld with a start pose",
                    sub_goal.human_id, sub_goal.human_id);
    return;
  }
  if (sub_goals_.find(sub_goal.human_id) != sub_goals_.end()) {
    sub_goals_[sub_goal.human_id].push_back(sub_goal.pose);
    ROS_INFO_NAMED(NODE_NAME, "Added additional sub-goal pose for human %ld",
                   sub_goal.human_id);
  } else {
    std::vector<geometry_msgs::PoseStamped> sub_goal_vector;
    sub_goal_vector.push_back(sub_goal.pose);
    sub_goals_[sub_goal.human_id] = sub_goal_vector;
    ROS_INFO_NAMED(NODE_NAME, "Added sub-goal pose for human %ld",
                   sub_goal.human_id);
  }
  client_cond_.notify_one();
}

void MoveHumansClient::removeHumanCB(const std_msgs::UInt64 &human_id) {
  boost::unique_lock<boost::mutex> lock(client_mutex_);
  starts_.erase(human_id.data);
  goals_.erase(human_id.data);
  sub_goals_.erase(human_id.data);
  client_cond_.notify_one();
}

} // namespace move_humans
