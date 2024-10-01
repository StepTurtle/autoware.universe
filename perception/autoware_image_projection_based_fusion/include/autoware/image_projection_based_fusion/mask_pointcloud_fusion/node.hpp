#include "autoware/image_projection_based_fusion/fusion_node.hpp"

#include <autoware/image_projection_based_fusion/utils/utils.hpp>

#include <cv_bridge/cv_bridge.h>

#include "sensor_msgs/point_cloud2_iterator.hpp"

#include <string>
#include <vector>
#include <unordered_set>

namespace autoware::image_projection_based_fusion
{
    struct PairHash {
        std::size_t operator()(const std::pair<size_t, int>& p) const {
            return std::hash<size_t>()(p.first);
        }
    };

    struct PairEqual {
        bool operator()(const std::pair<size_t, int>& lhs, const std::pair<size_t, int>& rhs) const {
            return lhs.first == rhs.first;  // Only compare the first elements
        }
    };

    struct Cluster {
        std::unordered_set<size_t> global_offset_list;
        uint8_t label_id;
        uint32_t min_x;
        uint32_t min_y;
        uint32_t max_x;
        uint32_t max_y;
    };

    using ClusterMap = std::map<uint8_t, Cluster>;

    class MaskPointCloudFusionNode
        : public FusionNode<PointCloud2, PointCloud2, SegmentationMask>
    {
    public:
        explicit MaskPointCloudFusionNode(const rclcpp::NodeOptions & options);

    protected:
        void preprocess(PointCloud2 &pointcloud_msg) override;

        void postprocess(PointCloud2 &pointcloud_msg) override;

        void fuseOnSingleImage(
                const PointCloud2 & input_pointcloud_msg, const std::size_t image_id,
                const SegmentationMask & input_mask_msg,
                const sensor_msgs::msg::CameraInfo & camera_info, PointCloud2 & output_pointcloud_msg) override;
        bool out_of_scope(const PointCloud2 & pointcloud) override;

        inline void copyPointCloud(
                const PointCloud2 & input, const int point_step, const size_t global_offset,
                PointCloud2 & output, size_t & output_pointcloud_size)
        {
            std::memcpy(&output.data[output_pointcloud_size], &input.data[global_offset], point_step);
            output_pointcloud_size += point_step;
        }

    private:
        rclcpp::Publisher<DetectedObjectsWithFeature>::SharedPtr pub_cluster_ptr_;
        rclcpp::Publisher<PointCloud2>::SharedPtr pub_color_pointcloud_ptr_;
        rclcpp::Publisher<Image>::SharedPtr pub_debug_mask_ptr_;


        float filter_distance_threshold_;
        std::vector<std::pair<std::string, bool>> keep_instance_label_list_ = {
                {"UNKNOWN",    true},
                {"CAR",        true},
                {"TRUCK",      true},
                {"BUS",        true},
                {"TRAILER",    true},
                {"MOTORCYCLE", true},
                {"BICYCLE",    true},
                {"PEDESTRIAN", true}};
        std::vector<std::pair<int , cv::Vec3b>> instance_id_color_list_ = {
                {0, cv::Vec3b(0, 0, 0)},
                {1, cv::Vec3b(255, 0, 255)},
                {2, cv::Vec3b(255, 165, 0)},
                {3, cv::Vec3b(0, 255, 76)},
                {4, cv::Vec3b(43, 172, 220)},
                {5, cv::Vec3b(255, 0, 0)},
                {6, cv::Vec3b(200, 0, 142)},
                {7, cv::Vec3b(220, 20, 60)}};
        std::unordered_set<size_t> filter_global_offset_set_;
        std::unordered_set<size_t> selected_instance_id_set_;
        std::unordered_set<std::pair<size_t, int>, PairHash, PairEqual> filter_instance_id_set_;

        std::vector<ClusterMap> cluster_map_list_;
    };
} // namespace autoware::image_projection_based_fusion