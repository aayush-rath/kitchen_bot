#include "semantic_map.h"

void visualizeSemanticMap(const SemanticMap& semantic_map, float voxel_size) {
    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("Semantic Map Viewer"));
    viewer->setBackgroundColor(0, 0, 0);
    
    int id = 0;
    for (const auto& [index, voxel] : semantic_map) {
        std::stringstream cube_id;
        cube_id << "voxel_" << id++;

        const auto& c = voxel.center;
        const auto& col = voxel.color;
        float a = voxel.alpha;

        viewer->addCube(
            c.x() - 20*voxel_size, c.x() + 20*voxel_size,
            c.y() - 20*voxel_size, c.y() + 20*voxel_size,
            c.z() - 20*voxel_size, c.z() + 20*voxel_size,
            col.x(), col.y(), col.z(), cube_id.str()
        );
        viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, a, cube_id.str());
    }

    viewer->addCoordinateSystem(1.0);
    viewer->initCameraParameters();

    while (!viewer->wasStopped()) {
        viewer->spinOnce(100);
    }
}

SemanticVoxelPublisher::SemanticVoxelPublisher(const SemanticMap& input_map) 
: Node("semantic_voxel_publisher"), semantic_map_(input_map){
    marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("semantic_voxels", 10);
    timer_ = this->create_wall_timer(
        std::chrono::seconds(1),
        std::bind(&SemanticVoxelPublisher::publish_voxels, this)
    );
    if(semantic_map_.empty()) load_dummy_map();
}

void SemanticVoxelPublisher::load_dummy_map() {
    for (int x = -3; x <= 3; ++x) {
        for (int y = -3; y <= 3; ++y) {
            for (int z = 0; z <= 1; ++z) {
                Eigen::Vector3i idx(x, y, z);
                Eigen::Vector3f center = voxel_size_ * idx.cast<float>();
                Eigen::Vector3f color(float(x + 3) / 6.0f, float(y + 3) / 6.0f, float(z + 1) / 2.0f);
                semantic_map_[idx] = SemanticVoxel{center, idx, color, 0.8f};
            }
        }
    }
}

void SemanticVoxelPublisher::publish_voxels() {
    visualization_msgs::msg::MarkerArray marker_array;
    int id = 0;
    for (const auto& [idx, voxel] : semantic_map_) {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = this->get_clock()->now();
        marker.ns = "semantic_voxels";
        marker.id = id++;
        marker.type = visualization_msgs::msg::Marker::CUBE;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.pose.position.x = voxel.center.x();
        marker.pose.position.y = voxel.center.y();
        marker.pose.position.z = voxel.center.z();
        marker.pose.orientation.w = 1.0;
        marker.scale.x = voxel_size_;
        marker.scale.y = voxel_size_;
        marker.scale.z = voxel_size_;
        marker.color.r = voxel.color.x();
        marker.color.g = voxel.color.y();
        marker.color.b = voxel.color.z();
        marker.color.a = voxel.alpha;
        marker.lifetime = rclcpp::Duration::from_seconds(0);
        marker_array.markers.push_back(marker);
    }
    marker_pub_->publish(marker_array);
}