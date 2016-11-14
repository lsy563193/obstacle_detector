#include "../include/obstacle_tracker.h"

using namespace obstacle_detector;
using namespace arma;
using namespace std;

int main(int argc, char** argv) {
  ros::init(argc, argv, "obstacle_tracker");
  ObstacleTracker ot;
  return 0;
}

ObstacleTracker::ObstacleTracker() : nh_(""), nh_local_("~"), rate_(5.0), p_active_(false) {
  std_srvs::Empty empty;
  updateParams(empty.request, empty.response);
  params_srv_ = nh_local_.advertiseService("params", &ObstacleTracker::updateParams, this);

  {
    ROS_INFO("Obstacle Tracker [Waiting for first message]");

    boost::shared_ptr<Obstacles const> msg_ptr;
    msg_ptr = ros::topic::waitForMessage<Obstacles>("raw_obstacles", ros::Duration(10));

    if (msg_ptr == nullptr) {
      ROS_INFO("Obstacle Tracker [ERROR: Didn't receive any message. Falling into sleep mode.]");
      nh_local_.setParam("active", false);
      std_srvs::Empty empty;
      updateParams(empty.request, empty.response);
    }
    else
      ROS_INFO("Obstacle Tracker [First message received]");
  }

  while (ros::ok()) {
    ros::spinOnce();

    if (p_active_) {
      updateObstacles();
      publishObstacles();
    }

    rate_.sleep();
  }
}

bool ObstacleTracker::updateParams(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res) {
  bool prev_active = p_active_;

  nh_local_.param<bool>("active", p_active_, true);
  nh_local_.param<bool>("copy_segments", p_copy_segments_, true);

  nh_local_.param<double>("loop_rate", p_loop_rate_, 100.0);
  nh_local_.param<double>("tracking_duration", p_tracking_duration_, 2.0);
  nh_local_.param<double>("min_correspondence_cost", p_min_correspondence_cost_, 0.3);
  nh_local_.param<double>("std_correspondence_dev", p_std_correspondence_dev_, 0.15);
  nh_local_.param<double>("process_variance", p_process_variance_, 0.01);
  nh_local_.param<double>("process_rate_variance", p_process_rate_variance_, 0.1);
  nh_local_.param<double>("measurement_variance", p_measurement_variance_, 1.0);

  TrackedObstacle::setSamplingTime(1.0 / p_loop_rate_);
  TrackedObstacle::setCounterSize(static_cast<int>(p_loop_rate_ * p_tracking_duration_));
  TrackedObstacle::setCovariances(p_process_variance_, p_process_rate_variance_, p_measurement_variance_);

  if (p_active_ != prev_active) {
    if (p_active_) {
      obstacles_sub_ = nh_.subscribe("raw_obstacles", 10, &ObstacleTracker::obstaclesCallback, this);
      obstacles_pub_ = nh_.advertise<Obstacles>("tracked_obstacles", 10);
      rate_ = ros::Rate(p_loop_rate_);

      ROS_INFO("Obstacle Tracker [ACTIVE]");
    }
    else {
      obstacles_sub_.shutdown();
      obstacles_pub_.shutdown();
      rate_ = ros::Rate(5.0);

      ROS_INFO("Obstacle Tracker [OFF]");
    }
  }

  return true;
}

void ObstacleTracker::obstaclesCallback(const Obstacles::ConstPtr& new_obstacles) {
  obstacles_msg_.header.frame_id = new_obstacles->header.frame_id;

  if (p_copy_segments_) {
    obstacles_msg_.segments.clear();
    obstacles_msg_.segments.assign(new_obstacles->segments.begin(), new_obstacles->segments.end());
  }

  int N = new_obstacles->circles.size();
  int T = tracked_obstacles_.size();
  int U = untracked_obstacles_.size();

  if (T + U == 0) {
    untracked_obstacles_.assign(new_obstacles->circles.begin(), new_obstacles->circles.end());
    return;
  }

  mat cost_matrix;
  calculateCostMatrix(new_obstacles->circles, cost_matrix);

  vector<int> row_min_indices;
  calculateRowMinIndices(cost_matrix, row_min_indices);

  vector<int> col_min_indices;
  calculateColMinIndices(cost_matrix, col_min_indices);

  vector<int> used_old_obstacles;
  vector<int> used_new_obstacles;

  vector<TrackedObstacle> new_tracked_obstacles;
  vector<CircleObstacle> new_untracked_obstacles;

  // Check for fusion (only tracked obstacles)
  for (int i = 0; i < T-1; ++i) {
    if (fusionObstacleUsed(i, col_min_indices, used_new_obstacles, used_old_obstacles))
      continue;

    vector<int> fusion_indices;
    fusion_indices.push_back(i);

    for (int j = i+1; j < T; ++j) {
      if (fusionObstaclesCorrespond(i, j, col_min_indices, used_old_obstacles))
        fusion_indices.push_back(j);
    }

    if (fusion_indices.size() > 1) {
      fuseObstacles(fusion_indices, col_min_indices, new_tracked_obstacles, new_obstacles);

      // Mark used old and new obstacles
      used_old_obstacles.insert(used_old_obstacles.end(), fusion_indices.begin(), fusion_indices.end());
      used_new_obstacles.push_back(col_min_indices[i]);
    }
  }

  // Check for fission (only tracked obstacles)
  for (int i = 0; i < N-1; ++i) {
    if (fissionObstacleUsed(i, T, row_min_indices, used_new_obstacles, used_old_obstacles))
      continue;

    vector<int> fission_indices;
    fission_indices.push_back(i);

    for (int j = i+1; j < N; ++j) {
      if (fissionObstaclesCorrespond(i, j, row_min_indices, used_new_obstacles))
        fission_indices.push_back(j);
    }

    if (fission_indices.size() > 1) {
      fissureObstacle(fission_indices, row_min_indices, new_tracked_obstacles, new_obstacles);

      // Mark used old and new obstacles
      used_old_obstacles.push_back(row_min_indices[i]);
      used_new_obstacles.insert(used_new_obstacles.end(), fission_indices.begin(), fission_indices.end());
    }
  }

  // Check for other possibilities
  for (int n = 0; n < N; ++n) {
    if (find(used_new_obstacles.begin(), used_new_obstacles.end(), n) != used_new_obstacles.end())
      continue;

    if (row_min_indices[n] == -1) {
      new_untracked_obstacles.push_back(new_obstacles->circles[n]);
    }
    else if (find(used_old_obstacles.begin(), used_old_obstacles.end(), row_min_indices[n]) == used_old_obstacles.end()) {
      if (row_min_indices[n] >= 0 && row_min_indices[n] < T) {
        tracked_obstacles_[row_min_indices[n]].correctState(new_obstacles->circles[n]);
      }
      else if (row_min_indices[n] >= T) {
        TrackedObstacle to(untracked_obstacles_[row_min_indices[n] - T]);
        to.assignId();
        to.correctState(new_obstacles->circles[n]);
        for (int i = 0; i < 10; ++i)
          to.updateState();

        new_tracked_obstacles.push_back(to);
      }

      used_new_obstacles.push_back(n);
    }
  }

  // Remove tracked obstacles that are no longer existent due to fusion or fission and insert new ones
  // Sort in descending order to remove from back of the list
  sort(used_old_obstacles.rbegin(), used_old_obstacles.rend());
  for (int idx : used_old_obstacles)
    tracked_obstacles_.erase(tracked_obstacles_.begin() + idx);

  tracked_obstacles_.insert(tracked_obstacles_.end(), new_tracked_obstacles.begin(), new_tracked_obstacles.end());

  // Remove old untracked obstacles and save new ones
  untracked_obstacles_.clear();
  untracked_obstacles_.assign(new_untracked_obstacles.begin(), new_untracked_obstacles.end());
}

double ObstacleTracker::obstacleCostFunction(const CircleObstacle& new_obstacle, const CircleObstacle& old_obstacle) {
  mat distribution = mat(2, 2).zeros();
  vec relative_position = vec(2).zeros();

  double cost = 0.0;
  double penalty = 1.0;
  double tp = 0.1;

  double direction = atan2(old_obstacle.velocity.y, old_obstacle.velocity.x);

  geometry_msgs::Point new_center = transformPoint(new_obstacle.center, 0.0, 0.0, -direction);
  geometry_msgs::Point old_center = transformPoint(old_obstacle.center, 0.0, 0.0, -direction);

  distribution(0, 0) = pow(p_std_correspondence_dev_, 2.0) + squaredLength(old_obstacle.velocity) * pow(tp, 2.0);
  distribution(1, 1) = pow(p_std_correspondence_dev_, 2.0);

  relative_position(0) = new_center.x - old_center.x - tp * length(old_obstacle.velocity);
  relative_position(1) = new_center.y - old_center.y;

  cost = sqrt(pow(new_obstacle.center.x - old_obstacle.center.x, 2.0) + pow(new_obstacle.center.y - old_obstacle.center.y, 2.0) + pow(new_obstacle.radius - old_obstacle.radius, 2.0));

  mat a = -0.5 * trans(relative_position) * distribution * relative_position;
  penalty = exp(a(0, 0));

  // TODO: Check values for cost/penalty in common situations
  // return cost / penalty;
  return cost / 1.0;
}

void ObstacleTracker::calculateCostMatrix(const vector<CircleObstacle>& new_obstacles, mat& cost_matrix) {
  /*
   * Cost between two obstacles represents their difference.
   * The bigger the cost, the less similar they are.
   * N rows of cost_matrix represent new obstacles.
   * T+U columns of cost matrix represent old tracked and untracked obstacles.
   */
  int N = new_obstacles.size();
  int T = tracked_obstacles_.size();
  int U = untracked_obstacles_.size();

  cost_matrix = mat(N, T + U, fill::zeros);

  for (int n = 0; n < N; ++n) {
    for (int t = 0; t < T; ++t)
      cost_matrix(n, t) = obstacleCostFunction(new_obstacles[n], tracked_obstacles_[t].getObstacle());

    for (int u = 0; u < U; ++u)
      cost_matrix(n, u + T) = obstacleCostFunction(new_obstacles[n], untracked_obstacles_[u]);
  }
}

void ObstacleTracker::calculateRowMinIndices(const mat& cost_matrix, vector<int>& row_min_indices) {
  /*
   * Vector of row minimal indices keeps the indices of old obstacles (tracked and untracked)
   * that have the minimum cost related to each of new obstacles, i.e. row_min_indices[n]
   * keeps the index of old obstacle that has the minimum cost with n-th new obstacle.
   */
  int N = cost_matrix.n_rows;
  int T = tracked_obstacles_.size();
  int U = untracked_obstacles_.size();

  row_min_indices.assign(N, -1); // Minimum index -1 means no correspondence has been found

  for (int n = 0; n < N; ++n) {
    double min_cost = p_min_correspondence_cost_;

    for (int t = 0; t < T; ++t) {
      if (cost_matrix(n, t) < min_cost) {
        min_cost = cost_matrix(n, t);
        row_min_indices[n] = t;
      }
    }

    for (int u = 0; u < U; ++u) {
      if (cost_matrix(n, u + T) < min_cost) {
        min_cost = cost_matrix(n, u + T);
        row_min_indices[n] = u + T;
      }
    }
  }
}

void ObstacleTracker::calculateColMinIndices(const mat& cost_matrix, vector<int>& col_min_indices) {
  /*
   * Vector of column minimal indices keeps the indices of new obstacles that has the minimum
   * cost related to each of old (tracked and untracked) obstacles, i.e. col_min_indices[i]
   * keeps the index of new obstacle that has the minimum cost with i-th old obstacle.
   */
  int N = cost_matrix.n_rows;
  int T = tracked_obstacles_.size();
  int U = untracked_obstacles_.size();

  col_min_indices.assign(T + U, -1); // Minimum index -1 means no correspondence has been found

  for (int t = 0; t < T; ++t) {
    double min_cost = p_min_correspondence_cost_;

    for (int n = 0; n < N; ++n) {
      if (cost_matrix(n, t) < min_cost) {
        min_cost = cost_matrix(n, t);
        col_min_indices[t] = n;
      }
    }
  }

  for (int u = 0; u < U; ++u) {
    double min_cost = p_min_correspondence_cost_;

    for (int n = 0; n < N; ++n) {
      if (cost_matrix(n, u + T) < min_cost) {
        min_cost = cost_matrix(n, u + T);
        col_min_indices[u + T] = n;
      }
    }
  }
}

bool ObstacleTracker::fusionObstacleUsed(const int idx, const vector<int> &col_min_indices, const vector<int> &used_new, const vector<int> &used_old) {
  /*
   * This function returns true if:
   * - idx-th old obstacle was already used
   * - obstacle to which idx-th old obstacle corresponds was already used
   * - there is no corresponding obstacle
   */

  return (find(used_old.begin(), used_old.end(), idx) != used_old.end() ||
          find(used_new.begin(), used_new.end(), col_min_indices[idx]) != used_new.end() ||
          col_min_indices[idx] < 0);
}

bool ObstacleTracker::fusionObstaclesCorrespond(const int idx, const int jdx, const vector<int>& col_min_indices, const vector<int>& used_old) {
  /*
   * This function return true if:
   * - both old obstacles correspond to the same new obstacle
   * - jdx-th old obstacle was not yet used
   */

  return (col_min_indices[idx] == col_min_indices[jdx] &&
          find(used_old.begin(), used_old.end(), jdx) == used_old.end());
}

bool ObstacleTracker::fissionObstacleUsed(const int idx, const int T, const vector<int>& row_min_indices, const vector<int>& used_new, const vector<int>& used_old) {
  /*
   * This function return true if:
   * - idx-th new obstacle was already used
   * - obstacle to which idx-th new obstacle corresponds was already used
   * - there is no corresponding obstacle
   * - obstacle to which idx-th new obstacle corresponds is untracked
   */

  return (find(used_new.begin(), used_new.end(), idx) != used_new.end() ||
          find(used_old.begin(), used_old.end(), row_min_indices[idx]) != used_old.end() ||
          row_min_indices[idx] < 0 ||
          row_min_indices[idx] >= T);
}

bool ObstacleTracker::fissionObstaclesCorrespond(const int idx, const int jdx, const vector<int>& row_min_indices, const vector<int>& used_new) {
  /*
   * This function return true if:
   * - both new obstacles correspond to the same old obstacle
   * - jdx-th new obstacle was not yet used
   */

  return (row_min_indices[idx] == row_min_indices[jdx] &&
          find(used_new.begin(), used_new.end(), jdx) == used_new.end());
}

void ObstacleTracker::fuseObstacles(const vector<int>& fusion_indices, const vector<int> &col_min_indices,
                                    vector<TrackedObstacle>& new_tracked, const Obstacles::ConstPtr& new_obstacles) {
  CircleObstacle c;

  double sum_var_x  = 0.0;
  double sum_var_y  = 0.0;
  double sum_var_vx = 0.0;
  double sum_var_vy = 0.0;
  double sum_var_r  = 0.0;

  for (int idx : fusion_indices) {
    c.center.x += tracked_obstacles_[idx].getObstacle().center.x / tracked_obstacles_[idx].getKFx().P(0,0);
    c.center.y += tracked_obstacles_[idx].getObstacle().center.y / tracked_obstacles_[idx].getKFy().P(0,0);
    c.velocity.x += tracked_obstacles_[idx].getObstacle().velocity.x / tracked_obstacles_[idx].getKFx().P(1,1);
    c.velocity.y += tracked_obstacles_[idx].getObstacle().velocity.y / tracked_obstacles_[idx].getKFy().P(1,1);
    c.radius += tracked_obstacles_[idx].getObstacle().radius / tracked_obstacles_[idx].getKFr().P(0,0);

    sum_var_x += 1.0 / tracked_obstacles_[idx].getKFx().P(0,0);
    sum_var_y += 1.0 / tracked_obstacles_[idx].getKFy().P(0,0);
    sum_var_vx += 1.0 / tracked_obstacles_[idx].getKFx().P(1,1);
    sum_var_vy += 1.0 / tracked_obstacles_[idx].getKFy().P(1,1);
    sum_var_r += 1.0 / tracked_obstacles_[idx].getKFr().P(0,0);

    c.obstacle_id += tracked_obstacles_[idx].getObstacle().obstacle_id + "-";
  }

  c.center.x /= sum_var_x;
  c.center.y /= sum_var_y;
  c.velocity.x /= sum_var_vx;
  c.velocity.y /= sum_var_vy;
  c.radius /= sum_var_r;

  c.obstacle_id.pop_back(); // Remove last "-"

  TrackedObstacle to(c);
  to.assignId();
  to.correctState(new_obstacles->circles[col_min_indices[fusion_indices.front()]]);
  for (int i = 0; i < 10; ++i)
    to.updateState();

  new_tracked.push_back(to);
}

void ObstacleTracker::fissureObstacle(const vector<int>& fission_indices, const vector<int>& row_min_indices,
                                      vector<TrackedObstacle>& new_tracked, const Obstacles::ConstPtr& new_obstacles) {
  tracked_obstacles_[row_min_indices[fission_indices.front()]].releaseId();

  // For each new obstacle taking part in fission create a tracked obstacle from the original old one and update it with the new one
  for (int idx : fission_indices) {
    TrackedObstacle to = tracked_obstacles_[row_min_indices[idx]];
    to.assignId();
    to.correctState(new_obstacles->circles[idx]);
    for (int i = 0; i < 10; ++i)
      to.updateState();

    new_tracked.push_back(to);
  }
}

void ObstacleTracker::updateObstacles() {
  for (int i = 0; i < tracked_obstacles_.size(); ++i) {
    if (!tracked_obstacles_[i].hasFaded())
      tracked_obstacles_[i].updateState();
    else {
      tracked_obstacles_[i].releaseId();
      tracked_obstacles_.erase(tracked_obstacles_.begin() + i--);
    }
  }
}

void ObstacleTracker::publishObstacles() {
  obstacles_msg_.header.stamp = ros::Time::now();
  obstacles_msg_.circles.clear();

  for (auto tracked_obstacle : tracked_obstacles_)
    obstacles_msg_.circles.push_back(tracked_obstacle.getObstacle());

  for (auto untracked_obstacle : untracked_obstacles_)
    obstacles_msg_.circles.push_back(untracked_obstacle);

  obstacles_pub_.publish(obstacles_msg_);
}

// Ugly initialization of static members of tracked obstacles...
list<string> TrackedObstacle::s_id_bank_         = {};
int    TrackedObstacle::s_id_size_               = 0;
int    TrackedObstacle::s_fade_counter_size_     = 0;
double TrackedObstacle::s_sampling_time_         = 100.0;
double TrackedObstacle::s_process_variance_      = 0.0;
double TrackedObstacle::s_process_rate_variance_ = 0.0;
double TrackedObstacle::s_measurement_variance_  = 0.0;
