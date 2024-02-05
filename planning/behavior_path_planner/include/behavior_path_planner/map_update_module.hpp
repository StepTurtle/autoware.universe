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

#ifndef AUTOWARE_MAP_UPDATE_MODULE_HPP
#define AUTOWARE_MAP_UPDATE_MODULE_HPP

#include <rclcpp/rclcpp.hpp>

#include "autoware_auto_mapping_msgs/msg/had_map_bin.hpp"
#include "autoware_map_msgs/msg/lanelet_map_meta_data.hpp"
#include "autoware_map_msgs/srv/get_differential_lanelet2_map.hpp"
#include <geometry_msgs/msg/point.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <lanelet2_extension/projection/mgrs_projector.hpp>

namespace behavior_path_planner
{

struct Lanelet2FileMetaData
{
  int id;
  double origin_lat;
  double origin_lon;
  std::string mgrs_code;
};

class MapUpdateModule
{
public:
  MapUpdateModule(
    rclcpp::Node * node,
    std::function<void(const autoware_auto_mapping_msgs::msg::HADMapBin::ConstSharedPtr)>
      map_update_function,
    std::string map_frame);

private:
  void onOdometry(const nav_msgs::msg::Odometry::ConstSharedPtr & msg);
  void mapUpdateTimerCallback();

  void updateMap(const geometry_msgs::msg::Point & pose);
  bool should_update_map() const;

  rclcpp::Publisher<autoware_auto_mapping_msgs::msg::HADMapBin>::SharedPtr differential_map_pub_;

  rclcpp::Client<autoware_map_msgs::srv::GetDifferentialLanelet2Map>::SharedPtr map_loader_client_;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_sub_;

  rclcpp::Subscription<autoware_map_msgs::msg::LaneletMapMetaData>::SharedPtr
    lanelet_map_meta_data_sub_;

  rclcpp::TimerBase::SharedPtr map_update_timer_;

  rclcpp::CallbackGroup::SharedPtr map_callback_group_;

  std::function<void(const autoware_auto_mapping_msgs::msg::HADMapBin::ConstSharedPtr)>
    map_update_function_;
  std::string map_frame_;
  rclcpp::Logger logger_;
  rclcpp::Clock::SharedPtr clock_;

  std::optional<geometry_msgs::msg::Point> last_update_position_ = std::nullopt;
  std::optional<geometry_msgs::msg::Point> current_position_ = std::nullopt;

  const double dynamic_map_loading_grid_size_;
  const double dynamic_map_loading_update_distance_;
  const double dynamic_map_loading_map_radius_;

  std::vector<Lanelet2FileMetaData> lanelet_map_meta_data_list_;
};

}  // namespace behavior_path_planner

#endif  // AUTOWARE_MAP_UPDATE_MODULE_HPP
