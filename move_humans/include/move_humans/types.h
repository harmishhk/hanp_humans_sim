#ifndef MOVE_HUMANS_TYPES_
#define MOVE_HUMANS_TYPES_

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <hanp_msgs/Trajectory.h>

namespace move_humans {
template <typename T1, typename T2>
using map_of_vector = std::map<T1, std::vector<T2>>;
template <typename T1, typename T2>
using map_of_vectors = std::map<T1, std::vector<std::vector<T2>>>;
using map_pose = std::map<uint64_t, geometry_msgs::PoseStamped>;
using map_twist = std::map<uint64_t, geometry_msgs::Twist>;
using map_trajectory = std::map<uint64_t, hanp_msgs::Trajectory>;
using map_pose_twist =
    std::map<uint64_t, std::pair<geometry_msgs::PoseStamped,
                                 geometry_msgs::TwistStamped>>;
using map_traj_point = std::map<uint64_t, hanp_msgs::TrajectoryPoint>;
using map_pose_vector = map_of_vector<uint64_t, geometry_msgs::PoseStamped>;
using map_twist_vector = map_of_vector<uint64_t, geometry_msgs::Twist>;
using map_pose_vectors = map_of_vectors<uint64_t, geometry_msgs::PoseStamped>;
using map_size = std::map<uint64_t, size_t>;
using pose_vector = std::vector<geometry_msgs::PoseStamped>;
using twist_vector = std::vector<geometry_msgs::TwistStamped>;
using pose_vectors = std::vector<std::vector<geometry_msgs::PoseStamped>>;
using id_vector = std::vector<uint64_t>;
struct TwistTime {
    geometry_msgs::Twist vel;
    ros::Time update_time;
  };
using map_twist_time = std::map<uint64_t, TwistTime>;
}; // namespace move_humans

#endif // MOVE_HUMANS_TYPES_
