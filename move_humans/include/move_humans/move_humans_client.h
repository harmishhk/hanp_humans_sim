#ifndef MOVE_HUMANS_CLIENT_H_
#define MOVE_HUMANS_CLIENT_H_

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <tf/transform_listener.h>
#include <std_srvs/Trigger.h>
#include <boost/thread.hpp>

#include "move_humans/types.h"
#include <move_humans/HumanUpdate.h>
#include <move_humans/MoveHumansAction.h>

namespace move_humans {
typedef actionlib::SimpleActionClient<move_humans::MoveHumansAction>
    MoveHumansActionClient;

class MoveHumansClient {
public:
  MoveHumansClient(tf::TransformListener &tf);
  virtual ~MoveHumansClient();

private:
  tf::TransformListener &tf_;

  MoveHumansActionClient *mhac_;

  std::string frame_id_;

  std::string reset_simulation_service_name_, add_human_service_name_,
      delete_human_service_name_, add_subgoal_service_name_,
      update_goal_service_name_;
  ros::ServiceServer reset_simulation_server_, add_human_server_,
      delete_human_server_, add_subgoal_server_, update_goal_server_;
  bool resetSimulation(std_srvs::Trigger::Request &req,
                       std_srvs::Trigger::Response &res);
  bool addHuman(move_humans::HumanUpdate::Request &req,
                move_humans::HumanUpdate::Response &res);
  bool deleteHuman(move_humans::HumanUpdate::Request &req,
                   move_humans::HumanUpdate::Response &res);
  bool addSubgoal(move_humans::HumanUpdate::Request &req,
                  move_humans::HumanUpdate::Response &res);
  bool updateGoal(move_humans::HumanUpdate::Request &req,
                  move_humans::HumanUpdate::Response &res);

  void feedbackCB(const MoveHumansFeedbackConstPtr &feedback);

  boost::thread *client_thread_;
  boost::mutex client_mutex_;
  boost::condition_variable client_cond_;
  void clientThread();

  move_humans::map_pose starts_, goals_;
  move_humans::map_pose_vector sub_goals_;

  bool getHumansGoals(ros::NodeHandle &nh, move_humans::map_pose &starts,
                      move_humans::map_pose &goals);
};
}; // namespace move_humans

#endif // MOVE_HUMANS_CLIENT_H_
