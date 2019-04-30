#include "move_humans/move_humans.h"
#include "move_humans/move_humans_client.h"

// the main method starts a rosnode and initializes the MoveHumans class
int main(int argc, char **argv) {

  // starting the move_humans node
  ros::init(argc, argv, "move_humans_node");

  // tf::TransformListener tf(ros::Duration(10));
  tf2_ros::Buffer tf2(ros::Duration(10));

  // starting the move_humans server
  move_humans::MoveHumans move_humans(tf2);

  // starting the move_humans client
  move_humans::MoveHumansClient move_humans_client(tf2);

  tf2.setUsingDedicatedThread(true);
  // start spinning
  ros::spin();

  return 0;
}
