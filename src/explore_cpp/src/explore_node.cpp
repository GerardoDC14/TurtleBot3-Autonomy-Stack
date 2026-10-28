// Frontier mission client. All mission state runs in the single-threaded executor.
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdint>
#include <limits>
#include <map>
#include <memory>
#include <optional>
#include <queue>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

using Clock = std::chrono::steady_clock;
using Coord = std::pair<double, double>;
using Cell = std::pair<int, int>;
using Map = nav_msgs::msg::OccupancyGrid;
using Pose = geometry_msgs::msg::PoseStamped;
using Action = nav2_msgs::action::NavigateToPose;
using Handle = rclcpp_action::ClientGoalHandle<Action>;

class ExploreNode : public rclcpp::Node
{
public:
  ExploreNode() : Node("explore_node"), buffer_(get_clock()), listener_(buffer_)
  {
    info_radius_ = declare_parameter("info_radius", 1.0);
    visit_radius_ = declare_parameter("frontier_visit_threshold", 0.3);
    max_attempts_ = declare_parameter("max_attempts", 3);
    interval_ = declare_parameter("timer_interval", 2.0);
    min_size_ = declare_parameter("min_frontier_size", 5);
    free_threshold_ = declare_parameter("free_threshold", 25);
    timeout_ = declare_parameter("navigation_timeout", 120.0);
    cooldown_ = declare_parameter("retry_cooldown", 10.0);
    clearance_ = declare_parameter("clearance_radius", 0.2);
    min_distance_ = declare_parameter("minimum_goal_distance", 0.3);
    distance_weight_ = declare_parameter("distance_weight", 0.5);
    map_frame_ = declare_parameter<std::string>("map_frame", "map");
    robot_frame_ = declare_parameter<std::string>("robot_frame", "base_link");
    for (double value : {info_radius_, visit_radius_, interval_, timeout_}) {
      if (!std::isfinite(value) || value <= 0) {
        throw std::invalid_argument("Radii, interval and timeout must be positive and finite");
      }
    }
    for (double value : {cooldown_, clearance_, min_distance_, distance_weight_}) {
      if (!std::isfinite(value) || value < 0) {
        throw std::invalid_argument("Clearance, distance and cooldown must be nonnegative");
      }
    }
    if (max_attempts_ < 1 || min_size_ < 1 || free_threshold_ < 0 || free_threshold_ > 100) {
      throw std::invalid_argument("Invalid attempt, cluster size or occupancy threshold");
    }
    subscriber_ = create_subscription<Map>(
      "map", rclcpp::QoS(1).transient_local(),
      [this](Map::SharedPtr message) {
        const auto & info = message->info;
        const auto & q = info.origin.orientation;
        const double norm = q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w;
        if (message->header.frame_id != map_frame_ || info.width == 0 || info.height == 0 ||
          !std::isfinite(info.resolution) || info.resolution <= 0 ||
          message->data.size() != static_cast<size_t>(info.width) * info.height ||
          !std::isfinite(info.origin.position.x) || !std::isfinite(info.origin.position.y) ||
          !std::isfinite(norm) || norm < 1e-12 || std::abs(q.x) > 1e-6 || std::abs(q.y) > 1e-6 ||
          std::any_of(message->data.begin(), message->data.end(),
            [](int value) {return value < -1 || value > 100;}))
        {
          RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "Discarding invalid map");
          return;
        }
        latest_map_ = message;
      });
    publisher_ = create_publisher<visualization_msgs::msg::MarkerArray>("frontiers", 10);
    client_ = rclcpp_action::create_client<Action>(this, "navigate_to_pose");
    timer_ = create_wall_timer(std::chrono::duration<double>(interval_), [this]() {cycle();});
    watchdog_ = create_wall_timer(std::chrono::milliseconds(200), [this]() {watchdog();});
  }

  void stop()
  {
    stopping_ = true;
    cancel_requested_ = true;
    cancel();
  }

  bool busy() const {return busy_;}

private:
  double info_radius_, visit_radius_, interval_, timeout_, cooldown_, clearance_;
  double min_distance_, distance_weight_;
  int max_attempts_, min_size_, free_threshold_;
  std::string map_frame_, robot_frame_;
  tf2_ros::Buffer buffer_;
  tf2_ros::TransformListener listener_;
  rclcpp::Subscription<Map>::SharedPtr subscriber_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr publisher_;
  rclcpp_action::Client<Action>::SharedPtr client_;
  rclcpp::TimerBase::SharedPtr timer_, watchdog_;
  Map::SharedPtr latest_map_;
  Handle::SharedPtr handle_;
  std::vector<Coord> visited_;
  std::map<Cell, int> attempts_;
  std::map<Cell, Clock::time_point> retry_after_;
  Coord current_;
  bool busy_ = false, stopping_ = false, cancel_requested_ = false, cancel_sent_ = false;
  std::uint64_t generation_ = 0;
  Clock::time_point started_;

  Cell key(Coord coord) const
  {
    return {static_cast<int>(std::round(coord.first * 100)),
      static_cast<int>(std::round(coord.second * 100))};
  }

  static double yaw(const Map & map)
  {
    const auto & q = map.info.origin.orientation;
    const double norm = q.z*q.z + q.w*q.w;
    return std::atan2(2*q.w*q.z / norm, 1-2*q.z*q.z / norm);
  }

  static Coord world(const Map & map, double col, double row)
  {
    double angle = yaw(map), x = (col+0.5)*map.info.resolution;
    double y = (row+0.5)*map.info.resolution;
    return {map.info.origin.position.x + std::cos(angle)*x - std::sin(angle)*y,
      map.info.origin.position.y + std::sin(angle)*x + std::cos(angle)*y};
  }

  static Cell cell(const Map & map, Coord coord)
  {
    double angle = yaw(map), x = coord.first-map.info.origin.position.x;
    double y = coord.second-map.info.origin.position.y;
    return {static_cast<int>(std::floor((std::cos(angle)*x + std::sin(angle)*y)/map.info.resolution)),
      static_cast<int>(std::floor((-std::sin(angle)*x + std::cos(angle)*y)/map.info.resolution))};
  }

  static bool inside(const Map & map, int x, int y)
  {
    return x >= 0 && y >= 0 && x < static_cast<int>(map.info.width) &&
      y < static_cast<int>(map.info.height);
  }

  bool free(const Map & map, int x, int y) const
  {
    int value = map.data[y*map.info.width+x];
    return value >= 0 && value <= free_threshold_;
  }

  bool valid(const Map & map, Coord coord) const
  {
    auto [col, row] = cell(map, coord);
    if (!inside(map, col, row) || !free(map, col, row)) {return false;}
    int radius = static_cast<int>(std::ceil(clearance_/map.info.resolution));
    for (int dy = -radius; dy <= radius; ++dy) {
      for (int dx = -radius; dx <= radius; ++dx) {
        if (std::hypot(dx, dy)*map.info.resolution > clearance_) {continue;}
        int x = col+dx, y = row+dy;
        if (!inside(map, x, y) || map.data[y*map.info.width+x] > free_threshold_) {
          return false;
        }
      }
    }
    return true;
  }

  std::vector<Coord> frontiers(const Map & map) const
  {
    const int width = map.info.width, height = map.info.height;
    std::vector<bool> frontier(map.data.size(), false), seen(map.data.size(), false);
    for (int y = 0; y < height; ++y) {
      for (int x = 0; x < width; ++x) {
        if (!free(map, x, y)) {continue;}
        for (int dy = -1; dy <= 1; ++dy) {
          for (int dx = -1; dx <= 1; ++dx) {
            if (inside(map, x+dx, y+dy) && map.data[(y+dy)*width+x+dx] == -1) {
              frontier[y*width+x] = true;
            }
          }
        }
      }
    }
    std::vector<Coord> result;
    for (int y = 0; y < height; ++y) {
      for (int x = 0; x < width; ++x) {
        if (!frontier[y*width+x] || seen[y*width+x]) {continue;}
        std::queue<Cell> queue;
        std::vector<Cell> group;
        queue.push({x, y});
        seen[y*width+x] = true;
        double sum_x = 0, sum_y = 0;
        while (!queue.empty()) {
          auto [cx, cy] = queue.front();
          queue.pop();
          group.push_back({cx, cy});
          sum_x += cx;
          sum_y += cy;
          for (int dy = -1; dy <= 1; ++dy) {
            for (int dx = -1; dx <= 1; ++dx) {
              int nx = cx+dx, ny = cy+dy;
              if (inside(map, nx, ny) && frontier[ny*width+nx] && !seen[ny*width+nx]) {
                seen[ny*width+nx] = true;
                queue.push({nx, ny});
              }
            }
          }
        }
        if (group.size() < static_cast<size_t>(min_size_)) {continue;}
        double center_x = sum_x/group.size(), center_y = sum_y/group.size();
        double best = std::numeric_limits<double>::infinity();
        std::optional<Coord> selected;
        for (auto [gx, gy] : group) {
          Coord candidate = world(map, gx, gy);
          double distance = std::hypot(gx-center_x, gy-center_y);
          if (distance < best && valid(map, candidate)) {
            best = distance;
            selected = candidate;
          }
        }
        if (selected) {result.push_back(*selected);}
      }
    }
    return result;
  }

  double gain(const Map & map, Coord coord) const
  {
    auto [cx, cy] = cell(map, coord);
    int radius = static_cast<int>(std::ceil(info_radius_/map.info.resolution));
    int count = 0;
    for (int y = std::max(0, cy-radius); y <= std::min(static_cast<int>(map.info.height)-1, cy+radius); ++y) {
      for (int x = std::max(0, cx-radius); x <= std::min(static_cast<int>(map.info.width)-1, cx+radius); ++x) {
        if (map.data[y*map.info.width+x] == -1) {++count;}
      }
    }
    return count*map.info.resolution*map.info.resolution;
  }

  void cycle()
  {
    if (stopping_ || busy_ || !latest_map_ || !client_->action_server_is_ready()) {return;}
    geometry_msgs::msg::TransformStamped transform;
    try {
      transform = buffer_.lookupTransform(map_frame_, robot_frame_, tf2::TimePointZero);
    } catch (const tf2::TransformException &) {
      return;
    }
    const auto map = latest_map_;
    if (!std::isfinite(transform.transform.translation.x) ||
      !std::isfinite(transform.transform.translation.y)) {return;}
    auto candidates = frontiers(*map);
    visualization_msgs::msg::MarkerArray markers;
    visualization_msgs::msg::Marker clear;
    clear.action = visualization_msgs::msg::Marker::DELETEALL;
    markers.markers.push_back(clear);
    std::optional<Coord> selected;
    double best = -std::numeric_limits<double>::infinity();
    const auto now = Clock::now();
    for (const auto & coord : candidates) {
      const auto index = key(coord);
      double distance = std::hypot(coord.first-transform.transform.translation.x,
        coord.second-transform.transform.translation.y);
      if (attempts_[index] >= max_attempts_ || retry_after_[index] > now || distance < min_distance_ ||
        std::any_of(visited_.begin(), visited_.end(), [&](Coord old) {
          return std::hypot(old.first-coord.first, old.second-coord.second) < visit_radius_;}))
      {
        continue;
      }
      visualization_msgs::msg::Marker marker;
      marker.header.frame_id = map_frame_;
      marker.header.stamp = get_clock()->now();
      marker.ns = "frontiers";
      marker.id = static_cast<int>(markers.markers.size());
      marker.type = visualization_msgs::msg::Marker::SPHERE;
      marker.action = visualization_msgs::msg::Marker::ADD;
      marker.pose.position.x = coord.first;
      marker.pose.position.y = coord.second;
      marker.pose.orientation.w = 1;
      marker.scale.x = marker.scale.y = marker.scale.z = 0.1;
      marker.color.r = marker.color.a = 1;
      markers.markers.push_back(marker);
      double score = gain(*map, coord)-distance_weight_*distance;
      if (score > best) {best = score; selected = coord;}
    }
    publisher_->publish(markers);
    if (!selected) {return;}
    Pose pose;
    pose.header.frame_id = map_frame_;
    pose.header.stamp = get_clock()->now();
    pose.pose.position.x = selected->first;
    pose.pose.position.y = selected->second;
    tf2::Quaternion q;
    q.setRPY(0, 0, std::atan2(selected->second-transform.transform.translation.y,
      selected->first-transform.transform.translation.x));
    pose.pose.orientation = tf2::toMsg(q);
    send(pose, *selected);
  }

  void send(const Pose & pose, Coord coord)
  {
    current_ = coord;
    ++attempts_[key(coord)];
    const auto generation = ++generation_;
    busy_ = true;
    cancel_requested_ = cancel_sent_ = false;
    handle_.reset();
    started_ = Clock::now();
    Action::Goal goal;
    goal.pose = pose;
    rclcpp_action::Client<Action>::SendGoalOptions options;
    options.goal_response_callback = [this, generation](Handle::SharedPtr handle) {
      if (generation != generation_) {return;}
      if (!handle) {finish(false, "rejected"); return;}
      handle_ = handle;
      if (cancel_requested_) {cancel();}
    };
    options.result_callback = [this, generation](const Handle::WrappedResult & result) {
      if (generation != generation_) {return;}
      finish(result.code == rclcpp_action::ResultCode::SUCCEEDED,
        result.code == rclcpp_action::ResultCode::SUCCEEDED ? "succeeded" : "failed/canceled");
    };
    try {
      (void)client_->async_send_goal(goal, options);
    } catch (const std::exception & error) {
      // Execution is uncertain after a transport error: keep the mission held.
      cancel_requested_ = true;
      RCLCPP_ERROR(get_logger(), "Goal submission uncertain; mission held: %s", error.what());
    }
  }

  void finish(bool success, const char * status)
  {
    if (success) {visited_.push_back(current_);}
    else {
      retry_after_[key(current_)] = Clock::now() +
        std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double>(cooldown_));
    }
    busy_ = false;
    handle_.reset();
    cancel_requested_ = cancel_sent_ = false;
    RCLCPP_INFO(get_logger(), "Navigation %s", status);
  }

  void cancel()
  {
    if (!busy_ || !handle_ || cancel_sent_) {return;}
    cancel_sent_ = true;
    try {
      (void)client_->async_cancel_goal(handle_);
    } catch (const std::exception & error) {
      RCLCPP_ERROR(get_logger(), "Cancellation unconfirmed; mission held: %s", error.what());
    }
    // Retain ownership until the result callback confirms a terminal state.
  }

  void watchdog()
  {
    if (busy_ && !cancel_requested_ &&
      std::chrono::duration<double>(Clock::now()-started_).count() >= timeout_)
    {
      cancel_requested_ = true;
      RCLCPP_WARN(get_logger(), "Navigation timeout; requesting cancellation");
      cancel();
    }
  }
};

volatile std::sig_atomic_t stop_requested = 0;
void request_stop(int) {stop_requested = 1;}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv, rclcpp::InitOptions(), rclcpp::SignalHandlerOptions::None);
  std::signal(SIGINT, request_stop);
  std::signal(SIGTERM, request_stop);
  int code = 0;
  try {
    auto node = std::make_shared<ExploreNode>();
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    while (rclcpp::ok() && !stop_requested) {
      executor.spin_some();
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    node->stop();
    auto deadline = Clock::now()+std::chrono::seconds(3);
    while (rclcpp::ok() && node->busy() && Clock::now() < deadline) {
      executor.spin_some();
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    if (node->busy()) {
      RCLCPP_ERROR(node->get_logger(), "Cancellation unconfirmed; verify the base is stopped");
      code = 1;
    }
  } catch (const std::exception & error) {
    RCLCPP_ERROR(rclcpp::get_logger("explore_node"), "%s", error.what());
    code = 1;
  }
  rclcpp::shutdown();
  return code;
}
