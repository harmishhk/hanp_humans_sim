#ifndef MOVE_HUMANS_PLANNER_INTERFACE_
#define MOVE_HUMANS_PLANNER_INTERFACE_

#include <costmap_2d/costmap_2d_ros.h>
#include <tf/transform_listener.h>

#include "move_humans/types.h"

namespace move_humans {

class PlannerInterface {
public:
  virtual ~PlannerInterface() {}

  virtual void initialize(std::string name, tf::TransformListener *tf,
                          costmap_2d::Costmap2DROS *costmap_ros) = 0;

  virtual bool makePlans(const move_humans::map_pose &starts,
                         const move_humans::map_pose &goals,
                         move_humans::map_pose_vectors &plans) = 0;

  virtual bool makePlans(const move_humans::map_pose &starts,
                         const move_humans::map_pose_vector &sub_goals,
                         const move_humans::map_pose &goals,
                         move_humans::map_pose_vectors &plans) = 0;

protected:
  PlannerInterface() {}
};
}; // namespace move_humans

#endif // MOVE_HUMANS_PLANNER_INTERFACE_
