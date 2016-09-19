#include "../include/obstacle_tracker.h"

using namespace obstacle_detector;
using namespace arma;
using namespace std;

#define TRACKER_TESTING1

ObstacleTracker::ObstacleTracker() : nh_(""), nh_local_("~") {
  nh_local_.param<bool>("track_labels", p_track_labels_, true);
  nh_local_.param<double>("loop_rate", p_loop_rate_, 100.0);
  nh_local_.param<double>("tracking_duration", p_tracking_duration_, 2.0);
  nh_local_.param<double>("min_correspondence_cost", p_min_correspondence_cost_, 0.6);
  nh_local_.param<double>("process_variance", p_process_variance_, 0.001);
  nh_local_.param<double>("process_rate_variance", p_process_rate_variance_, 0.001);
  nh_local_.param<double>("measurement_variance", p_measurement_variance_, 1.0);

  TrackedObstacle::setSamplingTime(1.0 / p_loop_rate_);
  TrackedObstacle::setCounterSize(static_cast<int>(p_loop_rate_ * p_tracking_duration_));
  TrackedObstacle::setCovariances(p_process_variance_, p_process_rate_variance_, p_measurement_variance_);

  obstacles_sub_ = nh_.subscribe("obstacles", 10, &ObstacleTracker::obstaclesCallback, this);
  tracked_obstacles_pub_ = nh_.advertise<obstacle_detector::Obstacles>("tracked_obstacles", 10);

  ROS_INFO("Obstacle Tracker [Waiting for first message]");
  ros::topic::waitForMessage<obstacle_detector::Obstacles>("obstacles");

  ROS_INFO("Obstacle Tracker [OK]");
  ros::Rate rate(p_loop_rate_);

  while (ros::ok()) {
    ros::spinOnce();

    updateObstacles();
    publishObstacles();

    rate.sleep();
  }
}

void ObstacleTracker::obstaclesCallback(const obstacle_detector::Obstacles::ConstPtr& new_obstacles) {
  // TODO: Add general possibility of fusion or fission of more than 2 obstacles
  // TODO: Reconsider the way the obstacles are merged when fused or fissured (use KF variances)
  tracked_obstacles_msg_.header.frame_id = new_obstacles->header.frame_id;

  int N = new_obstacles->circles.size();
  int T = tracked_obstacles_.size();
  int U = untracked_obstacles_.size();

  #ifdef TRACKER_TESTING
    cout << "---" << endl;
    cout << "New: " << N << endl;
    cout << "Tracked: " << T << endl;
    cout << "Untracked: " << U << endl;
  #endif

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

  /*
   * Possible situations:
   * If new obstacle does not correspond with any of old obstacles - save it as untracked.
   * If new obstacle corresponds with one and only one tracked obstacle - update it.
   * If new obstacle corresponds with one and only one untracked obstacle - save it as tracked and update it.
   *
   * If two (or more) old obstacles connect into one new obstacle, we call it a fusion.
   * If one old obstacle splits into two (or more) new obstacles, we call it a fission.
   * A fusion occurs if two (or more) old (tracked or not) obstacles have the same corresponding new obstacle - check columnwise.
   * A fission occurs if two (or more) new obstacles have the same corresponding old (tracked or not) obstacle - check rowswise.
   * If a fusion occured - create a tracked obstacle from the two (or more) old obstacles, update it with the new one, and remove the old ones.
   * If a fission occured - create two (or more) tracked obstacles from the single old obstacle and update them with the new ones, them remove the old one.
   */

  // TODO: Forget about taking untracked obstacles into fussion or fission

  vector<int> erase_indices;  // Indcises of old, tracked obstacles that will be removed

  // Check for fission
  for (int i = 0; i < N-1; ++i) {
    for (int j = i+1; j < N; ++j) {
      if (row_min_indices[i] == row_min_indices[j] && row_min_indices[i] >= 0 &&
          find(erase_indices.begin(), erase_indices.end(), row_min_indices[i]) == erase_indices.end()) {

        #ifdef TRACKER_TESTING
          cout << "Fission" << endl;
        #endif

        CircleObstacle c1;
        CircleObstacle c2;

        if (row_min_indices[i] < T) {
          c1 = meanCircObstacle(new_obstacles->circles[i], tracked_obstacles_[row_min_indices[i]].getObstacle());
          c2 = meanCircObstacle(new_obstacles->circles[j], tracked_obstacles_[row_min_indices[j]].getObstacle());

          if (p_track_labels_) {
            string name = tracked_obstacles_[row_min_indices[i]].getObstacle().obstacle_id;

            if (name.find("-") != string::npos) {
              c1.obstacle_id = name.substr(0, name.find("-"));
              c2.obstacle_id = name.substr(name.find("-")+1);
            }
            else {
              c1.obstacle_id = string("");
              c2.obstacle_id = string("");
            }
          }

          erase_indices.push_back(row_min_indices[i]);
        }
        else if (row_min_indices[i] >= T) {
          c1 = meanCircObstacle(new_obstacles->circles[i], untracked_obstacles_[row_min_indices[i] - T]);
          c2 = meanCircObstacle(new_obstacles->circles[j], untracked_obstacles_[row_min_indices[j] - T]);
        }

        TrackedObstacle to1 = TrackedObstacle(c1);
        TrackedObstacle to2 = TrackedObstacle(c2);

        to1.updateMeasurement(new_obstacles->circles[i]);
        to2.updateMeasurement(new_obstacles->circles[j]);

        to1.setFissed();
        to2.setFissed();

        tracked_obstacles_.push_back(to1);
        tracked_obstacles_.push_back(to2);

        // Mark both new obstacles as fissed (correspondence index -3)
        row_min_indices[i] = row_min_indices[j] = -3;

        break;
      }
    }
  }

  // Check for fusion
  for (int i = 0; i < T + U; ++i) {
    for (int j = i+1; j < T + U; ++j) {
      if (col_min_indices[i] == col_min_indices[j] && col_min_indices[i] >= 0 &&
          find(erase_indices.begin(), erase_indices.end(), i) == erase_indices.end() &&
          find(erase_indices.begin(), erase_indices.end(), j) == erase_indices.end()) {

        #ifdef TRACKER_TESTING
          cout << "Fusion" << endl;
        #endif

        CircleObstacle c;

        if (i < T && j < T) {
          c = meanCircObstacle(tracked_obstacles_[i].getObstacle(), tracked_obstacles_[j].getObstacle());
          if (p_track_labels_)
            c.obstacle_id = tracked_obstacles_[i].getObstacle().obstacle_id + "-" + tracked_obstacles_[j].getObstacle().obstacle_id;

          erase_indices.push_back(i);
          erase_indices.push_back(j);
        }
        else if (i < T && j >= T) {
          c = meanCircObstacle(tracked_obstacles_[i].getObstacle(), untracked_obstacles_[j - T]);
          if (p_track_labels_)
            c.obstacle_id = tracked_obstacles_[i].getObstacle().obstacle_id + "-OX";

          erase_indices.push_back(i);
        }
        else if (i >= T && j < T) {
          c = meanCircObstacle(untracked_obstacles_[i - T], tracked_obstacles_[j].getObstacle());
          if (p_track_labels_)
            c.obstacle_id = tracked_obstacles_[j].getObstacle().obstacle_id + "-OX";

          erase_indices.push_back(j);
        }
        else if (i >= T && j >= T) {
          c = meanCircObstacle(untracked_obstacles_[i - T], untracked_obstacles_[j - T]);
        }

        TrackedObstacle to = TrackedObstacle(c);
        to.updateMeasurement(new_obstacles->circles[col_min_indices[j]]);
        to.setFused();

        tracked_obstacles_.push_back(to);

        // Mark both old obstacles as fused (correspondence index -2)
        col_min_indices[i] = col_min_indices[j] = -2;

        break;
      }
    }
  }

#ifdef TRACKER_TESTING
  cout << "Row min indices after: ";
  for (int idx : row_min_indices)
    cout << idx << " ";
  cout << endl;
#endif

#ifdef TRACKER_TESTING
  cout << "Col min indices after: ";
  for (int idx : col_min_indices)
    cout << idx << " ";
  cout << endl;
#endif

  // Check for other possibilities
  vector<CircleObstacle> new_untracked_obstacles;

  for (int n = 0; n < N; ++n) {
    if (row_min_indices[n] == -1) {
      new_untracked_obstacles.push_back(new_obstacles->circles[n]);
    }
    else if (row_min_indices[n] >= 0 && row_min_indices[n] < T) {
      tracked_obstacles_[row_min_indices[n]].updateMeasurement(new_obstacles->circles[n]);
    }
    else if (row_min_indices[n] >= T) {
      CircleObstacle c = meanCircObstacle(new_obstacles->circles[n], untracked_obstacles_[row_min_indices[n] - T]);

      TrackedObstacle to = TrackedObstacle(c);
      to.updateMeasurement(c);

      tracked_obstacles_.push_back(to);
    }
  }

  // Remove tracked obstacles that are no longer existent due to fusion or fission
  // Sort in descending order to remove from back of the list
  sort(erase_indices.rbegin(), erase_indices.rend());
  for (int idx : erase_indices)
    tracked_obstacles_.erase(tracked_obstacles_.begin() + idx);

#ifdef TRACKER_TESTING
  cout << "Erase indices: ";
  for (int idx : erase_indices)
    cout << idx << " ";
  cout << endl;
#endif

  // Remove old untracked obstacles and save new ones
  untracked_obstacles_.clear();
  untracked_obstacles_.assign(new_untracked_obstacles.begin(), new_untracked_obstacles.end());
}

double ObstacleTracker::obstacleCostFunction(const CircleObstacle& new_obstacle, const CircleObstacle& old_obstacle) {
  // TODO: Add gauss elipses for penalties
  return sqrt(pow(new_obstacle.center.x - old_obstacle.center.x, 2.0) + pow(new_obstacle.center.y - old_obstacle.center.y, 2.0) + pow(new_obstacle.radius - old_obstacle.radius, 2.0));
}

void ObstacleTracker::calculateCostMatrix(const std::vector<CircleObstacle>& new_obstacles, arma::mat& cost_matrix) {
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

  #ifdef TRACKER_TESTING
    cout << "Cost matrix:" << endl;
    cout << endl << cost_matrix << endl;
  #endif
}

void ObstacleTracker::calculateRowMinIndices(const arma::mat& cost_matrix, std::vector<int>& row_min_indices) {
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

  #ifdef TRACKER_TESTING
    cout << "Row min indices: ";
    for (int idx : row_min_indices)
      cout << idx << " ";
    cout << endl;
  #endif
}

void ObstacleTracker::calculateColMinIndices(const arma::mat& cost_matrix, std::vector<int>& col_min_indices) {
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

  #ifdef TRACKER_TESTING
    cout << "Col min indices: ";
    for (int idx : col_min_indices)
      cout << idx << " ";
    cout << endl;
  #endif
}

CircleObstacle ObstacleTracker::meanCircObstacle(const CircleObstacle& c1, const CircleObstacle& c2) {
  // TODO: Remove this function
  CircleObstacle c;

  c.center.x = (c1.center.x + c2.center.x) / 2.0;
  c.center.y = (c1.center.y + c2.center.y) / 2.0;
  c.velocity.x = (c1.velocity.x + c2.velocity.x) / 2.0;
  c.velocity.y = (c1.velocity.y + c2.velocity.y) / 2.0;
  c.radius = (c1.radius + c2.radius) / 2.0;
  c.tracked = c1.tracked || c2.tracked;

  return c;
}

void ObstacleTracker::updateObstacles() {
  for (int i = 0; i < tracked_obstacles_.size(); ++i) {
    if (!tracked_obstacles_[i].hasFaded())
      tracked_obstacles_[i].updateTracking();
    else
      tracked_obstacles_.erase(tracked_obstacles_.begin() + i--);
  }
}

void ObstacleTracker::publishObstacles() {
  tracked_obstacles_msg_.header.stamp = ros::Time::now();
  tracked_obstacles_msg_.circles.clear();

  for (auto tracked_obstacle : tracked_obstacles_)
    tracked_obstacles_msg_.circles.push_back(tracked_obstacle.getObstacle());

  for (auto untracked_obstacle : untracked_obstacles_)
    tracked_obstacles_msg_.circles.push_back(untracked_obstacle);

  tracked_obstacles_pub_.publish(tracked_obstacles_msg_);
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "obstacle_tracker");
  ObstacleTracker ot;
  return 0;
}

// Ugly initialization of static members of tracked obstacles...
int    TrackedObstacle::obstacle_number_       = 0;
int    TrackedObstacle::fade_counter_size_     = 0;
double TrackedObstacle::sampling_time_         = 100.0;
double TrackedObstacle::process_variance_      = 0.0;
double TrackedObstacle::process_rate_variance_ = 0.0;
double TrackedObstacle::measurement_variance_  = 0.0;
