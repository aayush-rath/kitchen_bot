#ifndef SEMANTIC_MAP_H
#define SEMANTIC_MAP_H

#include <Eigen/Dense>
#include <pcl/visualization/pcl_visualizer.h>

#include <visualization_msgs/msg/marker_array.hpp>
#include <rclcpp/rclcpp.hpp>

struct SemanticVoxel {
    Eigen::Vector3f center;
    Eigen::Vector3i index;
    Eigen::Vector3f color;
    float alpha;
};

struct VoxelHash {
    std::size_t operator()(const Eigen::Vector3i& v) const {
        return std::hash<int>()(v.x()) ^ std::hash<int>()(v.y()) ^ std::hash<int>()(v.z());
    }
};

using SemanticMap = std::unordered_map<Eigen::Vector3i, SemanticVoxel, VoxelHash>;

void visualizeSemanticMap(const SemanticMap& semantic_map, float voxel_size = 0.001f);

class SemanticVoxelPublisher : public rclcpp::Node {
public:
    SemanticVoxelPublisher(const SemanticMap& input_map);

private:
    SemanticMap semantic_map_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    float voxel_size_ = 0.01f;

    void load_dummy_map();
    void publish_voxels();
};

#endif