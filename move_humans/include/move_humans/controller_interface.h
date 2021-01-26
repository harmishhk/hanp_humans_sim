#ifndef MOVE_HUMANS_CONTROLLER_INTERFACE_
#define MOVE_HUMANS_CONTROLLER_INTERFACE_

#include <costmap_2d/costmap_2d_ros.h>
#include <tf/transform_listener.h>
#include <tf2_ros/transform_listener.h>

#include "move_humans/types.h"

namespace move_humans {
class ControllerInterface {
public:
  virtual ~ControllerInterface() {}

  virtual void initialize(std::string name, tf2_ros::Buffer *tf2,
                          costmap_2d::Costmap2DROS *costmap_ros) = 0;

  virtual bool setPlans(const move_humans::map_pose_vector &plans) = 0;
  virtual bool setPlans(const move_humans::map_pose_vector &plans,
                        const move_humans::map_trajectory &trajectories,
                        const move_humans::map_twist &vels) = 0;

  virtual bool computeHumansStates(move_humans::map_traj_point &humans) = 0;

  virtual bool areGoalsReached(move_humans::id_vector &reached_humans) = 0;

protected:
  ControllerInterface() {}
};
}; // namespace move_humans

#endif // MOVE_HUMANS_CONTROLLER_INTERFACE_
