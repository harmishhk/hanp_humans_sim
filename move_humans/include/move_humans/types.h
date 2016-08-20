#ifndef MOVE_HUMANS_TYPES_
#define MOVE_HUMANS_TYPES_

#include <geometry_msgs/PoseStamped.h>

namespace move_humans {
template <typename T1, typename T2>
using map_of_vector = std::map<T1, std::vector<T2>>;
using map_pose = std::map<uint64_t, geometry_msgs::PoseStamped>;
using map_pose_vector = map_of_vector<uint64_t, geometry_msgs::PoseStamped>;
}; // namespace move_humans

#endif // MOVE_HUMANS_TYPES_
