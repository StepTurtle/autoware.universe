#include "autoware/image_projection_based_fusion/mask_pointcloud_fusion/node.hpp"

#include "autoware/image_projection_based_fusion/utils/geometry.hpp"
#include "autoware/image_projection_based_fusion/utils/utils.hpp"

namespace autoware::image_projection_based_fusion {
    MaskPointCloudFusionNode::MaskPointCloudFusionNode(const rclcpp::NodeOptions &options)
            : FusionNode<PointCloud2, PointCloud2, SegmentationMask>(
            "mask_pointcloud_fusion", options) {
        filter_distance_threshold_ = declare_parameter<float>("filter_distance_threshold");
        for (auto &item: keep_instance_label_list_) {
            item.second = declare_parameter<bool>("keep_instance_label." + item.first);
        }
        for (const auto &item: keep_instance_label_list_) {
            RCLCPP_INFO(
                    this->get_logger(), "filter_semantic_label_target: %s %d", item.first.c_str(), item.second);
        }

        pub_cluster_ptr_ = create_publisher<DetectedObjectsWithFeature>("output_cluster", 1);
        pub_color_pointcloud_ptr_ = create_publisher<PointCloud2>("~/output/color_pointcloud", 1);
        pub_debug_mask_ptr_ = create_publisher<Image>("~/debug/mask", 1);
    }

    void MaskPointCloudFusionNode::preprocess(
            __attribute__((unused)) PointCloud2 &pointcloud_msg) {
        return;
    }

    void MaskPointCloudFusionNode::postprocess(
            __attribute__((unused)) PointCloud2 &pointcloud_msg) {
        auto original_cloud = std::make_shared<PointCloud2>(pointcloud_msg);
//        sensor_msgs::msg::PointCloud2 output;
//        output.header = original_cloud->header;
//
//        size_t output_pointcloud_size = selected_instance_id_set_.size() * 4 * sizeof(float);
//        output.data.resize(output_pointcloud_size);
//
//        sensor_msgs::PointCloud2Modifier modifier(output);
//        modifier.setPointCloud2Fields(
//                4, "x", 1, sensor_msgs::msg::PointField::FLOAT32, "y", 1, sensor_msgs::msg::PointField::FLOAT32,
//                "z", 1, sensor_msgs::msg::PointField::FLOAT32, "rgb", 1, sensor_msgs::msg::PointField::FLOAT32);
//        modifier.resize(output_pointcloud_size);
//
//        sensor_msgs::PointCloud2Iterator<float> iter_out_x(output, "x");
//        sensor_msgs::PointCloud2Iterator<float> iter_out_y(output, "y");
//        sensor_msgs::PointCloud2Iterator<float> iter_out_z(output, "z");
//        sensor_msgs::PointCloud2Iterator<uint8_t> iter_out_r(output, "r");
//        sensor_msgs::PointCloud2Iterator<uint8_t> iter_out_g(output, "g");
//        sensor_msgs::PointCloud2Iterator<uint8_t> iter_out_b(output, "b");
//
//        int x_offset = original_cloud->fields[pcl::getFieldIndex(*original_cloud, "x")].offset;
//        int y_offset = original_cloud->fields[pcl::getFieldIndex(*original_cloud, "y")].offset;
//        int z_offset = original_cloud->fields[pcl::getFieldIndex(*original_cloud, "z")].offset;
//
//        for (const auto &pair_offset_id: filter_instance_id_set_) {
//            const size_t global_offset = pair_offset_id.first;
//            const int label_id = pair_offset_id.second;
//            *iter_out_x = *reinterpret_cast<float *>(&original_cloud->data[global_offset + x_offset]);
//            *iter_out_y = *reinterpret_cast<float *>(&original_cloud->data[global_offset + y_offset]);
//            *iter_out_z = *reinterpret_cast<float *>(&original_cloud->data[global_offset + z_offset]);
//            *iter_out_r = instance_id_color_list_[label_id].second[0];
//            *iter_out_g = instance_id_color_list_[label_id].second[1];
//            *iter_out_b = instance_id_color_list_[label_id].second[2];
//            ++iter_out_x;
//            ++iter_out_y;
//            ++iter_out_z;
//            ++iter_out_r;
//            ++iter_out_g;
//            ++iter_out_b;
//        }
//
////        output.data.resize(output_pointcloud_size);
////        output.row_step = output_pointcloud_size / original_cloud->height;
////        output.width = output_pointcloud_size / original_cloud->point_step / original_cloud->height;
//        output.width = output_pointcloud_size;
//        output.height = 1;
//        output.is_dense = false;
////        pub_color_pointcloud_ptr_->publish(output);
//
//        selected_instance_id_set_.clear();
//        filter_instance_id_set_.clear();

        // Clusters

        DetectedObjectsWithFeature output_feature_objects_msg;
        output_feature_objects_msg.header = original_cloud->header;

        for (const auto &cluster_map: cluster_map_list_) {
            const auto &cluster = cluster_map.begin()->second;
            if (cluster.global_offset_list.size() < 8) { continue; }

            DetectedObjectWithFeature feature_object;
            feature_object.object.existence_probability = 1.0;

            // create ROS point cloud from cluster points
            PointCloud2 cluster_pointcloud;
            cluster_pointcloud.header = original_cloud->header;
            size_t cluster_pointcloud_size = cluster.global_offset_list.size() * 3 * sizeof(float);
            cluster_pointcloud.data.resize(cluster_pointcloud_size);
            sensor_msgs::PointCloud2Modifier modifier_cluster(cluster_pointcloud);
            modifier_cluster.setPointCloud2Fields(
                    3, "x", 1, sensor_msgs::msg::PointField::FLOAT32, "y", 1, sensor_msgs::msg::PointField::FLOAT32,
                    "z", 1, sensor_msgs::msg::PointField::FLOAT32);
            modifier_cluster.resize(cluster_pointcloud_size);
            sensor_msgs::PointCloud2Iterator<float> iter_cluster_x(cluster_pointcloud, "x");
            sensor_msgs::PointCloud2Iterator<float> iter_cluster_y(cluster_pointcloud, "y");
            sensor_msgs::PointCloud2Iterator<float> iter_cluster_z(cluster_pointcloud, "z");
            int x_offset = original_cloud->fields[pcl::getFieldIndex(*original_cloud, "x")].offset;
            int y_offset = original_cloud->fields[pcl::getFieldIndex(*original_cloud, "y")].offset;
            int z_offset = original_cloud->fields[pcl::getFieldIndex(*original_cloud, "z")].offset;
            for (const size_t &global_offset: cluster.global_offset_list) {
                *iter_cluster_x = *reinterpret_cast<float *>(&original_cloud->data[global_offset + x_offset]);
                *iter_cluster_y = *reinterpret_cast<float *>(&original_cloud->data[global_offset + y_offset]);
                *iter_cluster_z = *reinterpret_cast<float *>(&original_cloud->data[global_offset + z_offset]);
                ++iter_cluster_x;
                ++iter_cluster_y;
                ++iter_cluster_z;
            }
//            cluster_pointcloud.width = cluster_pointcloud_size;
//            cluster_pointcloud.height = 1;
//            cluster_pointcloud.is_dense = false;
            cluster_pointcloud.data.resize(cluster_pointcloud_size);
            cluster_pointcloud.row_step = cluster_pointcloud_size / original_cloud->height;
            cluster_pointcloud.width = cluster_pointcloud_size / original_cloud->point_step / original_cloud->height;

            pub_color_pointcloud_ptr_->publish(cluster_pointcloud);

            // feature_object.feature.cluster
            feature_object.feature.cluster = cluster_pointcloud;

            // feature_object.object.kinematics.pose_with_covariance.pose.position
            feature_object.object.kinematics.pose_with_covariance.pose.position = getCentroid(cluster_pointcloud);

            autoware_perception_msgs::msg::ObjectClassification classification;
            classification.label = static_cast<uint8_t>(cluster.label_id);
            classification.probability = 1.0f;
            feature_object.object.classification.emplace_back(classification);

            sensor_msgs::msg::RegionOfInterest roi;
            roi.x_offset = cluster.min_x;
            roi.y_offset = cluster.min_y;
            roi.width = cluster.max_x - cluster.min_x;
            roi.height = cluster.max_y - cluster.min_y;
            feature_object.feature.roi = roi;

            output_feature_objects_msg.feature_objects.push_back(feature_object);
        }
        pub_cluster_ptr_->publish(output_feature_objects_msg);

        cluster_map_list_.clear();
    }

    void MaskPointCloudFusionNode::fuseOnSingleImage(
            __attribute__((unused)) const PointCloud2 &input_pointcloud_msg,
            __attribute__((unused)) const std::size_t image_id,
            __attribute__((unused)) const SegmentationMask &input_mask_msg,
            __attribute__((unused)) const sensor_msgs::msg::CameraInfo &camera_info,
            __attribute__((unused)) PointCloud2 &output_pointcloud_msg) {
        auto start = std::chrono::high_resolution_clock::now();

        if (input_pointcloud_msg.data.empty()) {
            return;
        }
        if (!checkCameraInfo(camera_info)) return;
        if (input_mask_msg.image.height == 0 || input_mask_msg.image.width == 0) {
            return;
        }

        // Convert mask to cv::Mat - Resize mask to the same size as the camera image
        cv_bridge::CvImagePtr in_image_ptr;
        try {
            in_image_ptr = cv_bridge::toCvCopy(
                    std::make_shared<sensor_msgs::msg::Image>(input_mask_msg.image),
                    input_mask_msg.image.encoding);
        } catch (const std::exception &e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception:%s", e.what());
            return;
        }
        cv::Mat mask = in_image_ptr->image;
        const int orig_width = camera_info.width;
        const int orig_height = camera_info.height;
        // resize mask to the same size as the camera image
        cv::resize(mask, mask, cv::Size(orig_width, orig_height), 0, 0, cv::INTER_NEAREST);
        image_geometry::PinholeCameraModel pinhole_camera_model;
        pinhole_camera_model.fromCameraInfo(camera_info);

        // Get transform from camera frame to pointcloud frame
        geometry_msgs::msg::TransformStamped transform_stamped;
        {
            const auto transform_stamped_optional = getTransformStamped(
                    tf_buffer_, /*target*/ input_mask_msg.header.frame_id,
                    /*source*/ input_pointcloud_msg.header.frame_id, camera_info.header.stamp);
            if (!transform_stamped_optional) {
                RCLCPP_WARN_STREAM(
                        get_logger(), "Failed to get transform from " << input_mask_msg.header.frame_id << " to "
                                                                      << input_pointcloud_msg.header.frame_id);
                return;
            }
            transform_stamped = transform_stamped_optional.value();
        }

        // Transform pointcloud from frame id to camera optical frame id
        PointCloud2 transformed_cloud;
        tf2::doTransform(input_pointcloud_msg, transformed_cloud, transform_stamped);

        int point_step = input_pointcloud_msg.point_step;
        int x_offset = input_pointcloud_msg.fields[pcl::getFieldIndex(input_pointcloud_msg, "x")].offset;
        int y_offset = input_pointcloud_msg.fields[pcl::getFieldIndex(input_pointcloud_msg, "y")].offset;
        int z_offset = input_pointcloud_msg.fields[pcl::getFieldIndex(input_pointcloud_msg, "z")].offset;

        for (size_t global_offset = 0; global_offset < transformed_cloud.data.size();
             global_offset += point_step) {
            float transformed_x =
                    *reinterpret_cast<float *>(&transformed_cloud.data[global_offset + x_offset]);
            float transformed_y =
                    *reinterpret_cast<float *>(&transformed_cloud.data[global_offset + y_offset]);
            float transformed_z =
                    *reinterpret_cast<float *>(&transformed_cloud.data[global_offset + z_offset]);
            // skip filtering pointcloud behind the camera or too far from camera
            if (transformed_z <= 0.0 || transformed_z > filter_distance_threshold_) {
                continue;
            }

            Eigen::Vector2d projected_point = calcRawImageProjectedPoint(
                    pinhole_camera_model, cv::Point3d(transformed_x, transformed_y, transformed_z));

            bool is_inside_image = projected_point.x() > 0 && projected_point.x() < camera_info.width &&
                                   projected_point.y() > 0 && projected_point.y() < camera_info.height;
            if (!is_inside_image) {
                continue;
            }

            const uint8_t pixel_value = mask.at<uint8_t>(
                    static_cast<uint16_t>(projected_point.y()), static_cast<uint16_t>(projected_point.x()));
            const auto label_id = static_cast<int>(static_cast<uint8_t>(input_mask_msg.config.classification[
                    pixel_value - 1].label));
            if (pixel_value != 0) {

                auto it = std::find_if(cluster_map_list_.begin(), cluster_map_list_.end(),
                                       [pixel_value](const ClusterMap &cluster_map) {
                                           return !cluster_map.empty() && cluster_map.begin()->first == pixel_value;
                                       });

                if (it == cluster_map_list_.end()) {
                    Cluster cluster;
                    cluster.label_id = label_id;
                    cluster.global_offset_list.insert(global_offset);
                    cluster.min_x = camera_info.width;
                    cluster.min_y = camera_info.height;
                    cluster.max_x = 0;
                    cluster.max_y = 0;
                    cluster.min_x = std::min(cluster.min_x, static_cast<uint32_t>(projected_point.x()));
                    cluster.min_y = std::min(cluster.min_y, static_cast<uint32_t>(projected_point.y()));
                    cluster.max_x = std::max(cluster.max_x, static_cast<uint32_t>(projected_point.x()));
                    cluster.max_y = std::max(cluster.max_y, static_cast<uint32_t>(projected_point.y()));
                    cluster_map_list_.push_back({{pixel_value, cluster}});
                } else {
                    auto &cluster = it->begin()->second;
                    cluster.global_offset_list.insert(global_offset);
                    cluster.min_x = std::min(cluster.min_x, static_cast<uint32_t>(projected_point.x()));
                    cluster.min_y = std::min(cluster.min_y, static_cast<uint32_t>(projected_point.y()));
                    cluster.max_x = std::max(cluster.max_x, static_cast<uint32_t>(projected_point.x()));
                    cluster.max_y = std::max(cluster.max_y, static_cast<uint32_t>(projected_point.y()));
                }

                selected_instance_id_set_.insert(global_offset);

                std::pair<size_t, int> pair(global_offset, label_id);
                filter_instance_id_set_.insert(pair);
                continue;
            }

            filter_global_offset_set_.insert(global_offset);
        }

        auto finish = std::chrono::high_resolution_clock::now();
        auto diff = finish - start;
        std::cout << "Mask: " << std::chrono::duration<double, std::milli>(diff).count()
                  << " ms"
                  << std::endl;
    }

    bool MaskPointCloudFusionNode::out_of_scope(__attribute__((unused))
                                                const PointCloud2 &pointcloud) {
        return false;
    }
} // namespace autoware::image_projection_based_fusion

#include <rclcpp_components/register_node_macro.hpp>

RCLCPP_COMPONENTS_REGISTER_NODE(autoware::image_projection_based_fusion::MaskPointCloudFusionNode)