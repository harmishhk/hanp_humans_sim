#ifndef MOVE_HUMANS_CLIENT_H_
#define MOVE_HUMANS_CLIENT_H_

#include <ros/ros.h>
#include <move_humans/MoveHumansAction.h>
#include <move_humans/types.h>
#include <actionlib/client/simple_action_client.h>
#include <tf/transform_listener.h>
#include <std_msgs/UInt64.h>
#include <boost/thread.hpp>

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

  std::string new_human_sub_topic_, goal_sub_topic_, sub_goal_sub_topic_,
      remove_human_sub_topic_, frame_id_;
  ros::Subscriber new_human_sub_, goal_sub_, sub_goal_sub_, remove_human_sub_;
  void newHumanCB(const move_humans::HumanPose &new_human);
  void goalCB(const move_humans::HumanPose &goal);
  void subGoalCB(const move_humans::HumanPose &sub_goal);
  void removeHumanCB(const std_msgs::UInt64 &human_id);

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
