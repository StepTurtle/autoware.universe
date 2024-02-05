// Copyright 2023 Autoware Foundation
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "behavior_path_planner/utils/map_update_module.hpp"

#include <utility>

namespace behavior_path_planner
{

// Define a helper function to get x and y coordinates
template <typename T>
auto getX(const T & point) -> decltype(point.x)
{
  return point.x;
}

template <typename T>
auto getY(const T & point) -> decltype(point.y)
{
  return point.y;
}

// Overloads for types with x() and y() methods
template <typename T>
auto getX(const T & point) -> decltype(point.x())
{
  return point.x();
}

template <typename T>
auto getY(const T & point) -> decltype(point.y())
{
  return point.y();
}

template <typename T, typename U>
double norm_xy(const T & p1, const U & p2)
{
  double dx = getX(p1) - getX(p2);
  double dy = getY(p1) - getY(p2);
  return std::sqrt(dx * dx + dy * dy);
}

MapUpdateModule::MapUpdateModule(
  rclcpp::Node * node,
  std::function<void(const autoware_auto_mapping_msgs::msg::HADMapBin::ConstSharedPtr)>
    map_update_function,
  std::string map_frame)
: map_update_function_(std::move(map_update_function)),
  map_frame_(std::move(map_frame)),
  logger_(node->get_logger()),
  clock_(node->get_clock()),
  dynamic_map_loading_grid_size_(5000),
  dynamic_map_loading_update_distance_(dynamic_map_loading_grid_size_ * sqrt(2)),
  dynamic_map_loading_map_radius_(dynamic_map_loading_grid_size_ * sqrt(5))
{
  differential_map_pub_ = node->create_publisher<autoware_auto_mapping_msgs::msg::HADMapBin>(
    "debug/differential_lanelet_map", rclcpp::QoS{1}.transient_local());

  map_loader_client_ = node->create_client<autoware_map_msgs::srv::GetDifferentialLanelet2Map>(
    "/map/get_differential_lanelet2_map");

  odometry_sub_ = node->create_subscription<nav_msgs::msg::Odometry>(
    "~/input/odometry", 100, std::bind(&MapUpdateModule::onOdometry, this, std::placeholders::_1));

  lanelet_map_meta_data_sub_ =
    node->create_subscription<autoware_map_msgs::msg::LaneletMapMetaData>(
      "/map/lanelet_map_meta_data", rclcpp::QoS{1}.transient_local(),
      [this](const autoware_map_msgs::msg::LaneletMapMetaData::SharedPtr msg) {
        for (const auto & data : msg->metadata_list) {
          Lanelet2FileMetaData metadata;
          metadata.id = data.tile_id;
          metadata.origin_lat = data.metadata.origin_lat;
          metadata.origin_lon = data.metadata.origin_lon;
          metadata.mgrs_code = data.metadata.mgrs_code;
          lanelet_map_meta_data_list_.push_back(metadata);
        }
      });

  map_callback_group_ = node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  double map_update_dt = 1.0;
  auto period_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::duration<double>(map_update_dt));
  map_update_timer_ = rclcpp::create_timer(
    node, clock_, period_ns, std::bind(&MapUpdateModule::mapUpdateTimerCallback, this),
    map_callback_group_);
}

void MapUpdateModule::mapUpdateTimerCallback()
{
  if (current_position_ == std::nullopt) {
    RCLCPP_ERROR_STREAM_THROTTLE(
      logger_, *clock_, 1,
      "Cannot find the reference position for lanelet map update. Please check if the EKF odometry "
      "is provided to behavior planner map update module.");
    return;
  }

  if (should_update_map()) {
    RCLCPP_INFO(logger_, "Start updating lanelet map (timer callback)");

    updateMap(current_position_.value());
    last_update_position_ = current_position_;
  }
}

void MapUpdateModule::updateMap(const geometry_msgs::msg::Point & pose)
{
  lanelet::projection::MGRSProjector projector;

  std::vector<int> cache_ids;
  for (const auto & metadata : lanelet_map_meta_data_list_) {
    lanelet::GPSPoint gps_point;
    gps_point.lat = metadata.origin_lat;
    gps_point.lon = metadata.origin_lon;
    gps_point.ele = 0;

    lanelet::BasicPoint3d point = projector.forward(gps_point);

    double distance = norm_xy(point, pose);
    if (distance < dynamic_map_loading_map_radius_) {
      cache_ids.push_back(metadata.id);
    }
  }

  auto request = std::make_shared<autoware_map_msgs::srv::GetDifferentialLanelet2Map::Request>();
  request->osm_file_ids.insert(request->osm_file_ids.end(), cache_ids.begin(), cache_ids.end());

  while (!map_loader_client_->wait_for_service(std::chrono::seconds(1)) && rclcpp::ok()) {
    RCLCPP_INFO(logger_, "Waiting for lanelet loader service");
  }

  auto result{map_loader_client_->async_send_request(
    request,
    [](rclcpp::Client<autoware_map_msgs::srv::GetDifferentialLanelet2Map>::SharedFuture) {})};

  std::future_status status = result.wait_for(std::chrono::seconds(0));
  while (status != std::future_status::ready) {
    switch (status) {
      case std::future_status::ready:
        RCLCPP_DEBUG(logger_, "The future status is (ready).");
        break;
      case std::future_status::timeout:
        RCLCPP_DEBUG(logger_, "The future status is (timed out).");
        break;
      case std::future_status::deferred:
        RCLCPP_DEBUG(logger_, "The future status is (deferred).");
        break;
    }
    RCLCPP_INFO(logger_, "waiting response from lanelet loader service.");
    if (!rclcpp::ok()) {
      return;
    }
    status = result.wait_for(std::chrono::seconds(1));
  }

  autoware_auto_mapping_msgs::msg::HADMapBin::ConstSharedPtr map_ptr{
    std::make_shared<autoware_auto_mapping_msgs::msg::HADMapBin const>(
      result.get()->differential_map)};
  map_update_function_(map_ptr);
  differential_map_pub_->publish(result.get()->differential_map);
}

void MapUpdateModule::onOdometry(const nav_msgs::msg::Odometry::ConstSharedPtr & msg)
{
  current_position_ = msg->pose.pose.position;
}

bool MapUpdateModule::should_update_map() const
{
  if (last_update_position_ == std::nullopt) {
    return true;
  }

  double distance = norm_xy(current_position_.value(), last_update_position_.value());
  return distance > dynamic_map_loading_update_distance_;
}

}  // namespace behavior_path_planner