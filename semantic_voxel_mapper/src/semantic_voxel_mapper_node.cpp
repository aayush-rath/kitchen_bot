#include "semantic_map.h"
#include "file_parser.h"

#include <assimp/Importer.hpp>
#include <assimp/scene.h>
#include <assimp/postprocess.h>
#include <assimp/material.h>
#include <assimp/vector3.h>

#include <ament_index_cpp/get_package_share_directory.hpp>

SemanticMap semantic_map;

int main(int argc, char const *argv[]) {
    std::string package_path = ament_index_cpp::get_package_share_directory("kitchen_bot_description");
    std::string world_file_path = package_path + "/world/kitchen_world.sdf";
    std::unordered_map<std::string, std::vector<double>> models;

    read_world_file(world_file_path, models);
    Assimp::Importer importer;

    for (const auto& element : models) {
        std::string model_root = package_path + "/src/kitchen_bot_description/models";
        std::string model_sdf_path = model_root + "/" + element.first + "/model.sdf";
        std::string mesh_path = get_mesh_path(model_sdf_path, model_root, element.first);

        const aiScene* scene = importer.ReadFile(mesh_path, aiProcess_Triangulate | aiProcess_GenSmoothNormals | aiProcess_FlipUVs);
        if (!scene) {
            std::cerr << "[ERROR] Assimp failed to load mesh: " << mesh_path << std::endl;
            std::cerr << importer.GetErrorString() << std::endl;
            continue;
        }

        std::vector<Eigen::Vector3f> points;
        std::vector<Eigen::Vector3f> colors;

        for (unsigned int m = 0; m < scene->mNumMeshes; ++m) {
            aiMesh* mesh = scene->mMeshes[m];
            // aiMaterial* mat = scene->mMaterials[mesh->mMaterialIndex];

            Eigen::Vector3f color = Eigen::Vector3f::Random();

            for (unsigned int i = 0; i < mesh->mNumVertices; ++i) {
                aiVector3D v = mesh->mVertices[i];
                Eigen::Vector3f point(v.x, v.y, v.z);

                points.push_back(point);
                colors.push_back(color);
            }
        }

        Eigen::Affine3f pose = Eigen::Translation3f(element.second[0], element.second[1], element.second[2]) *
                       Eigen::AngleAxisf(element.second[5], Eigen::Vector3f::UnitZ()) *
                       Eigen::AngleAxisf(element.second[4], Eigen::Vector3f::UnitY()) *
                       Eigen::AngleAxisf(element.second[3], Eigen::Vector3f::UnitX());

        for (size_t i = 0; i < points.size(); ++i) {
            points[i] = pose * points[i];
        }

        float voxel_size = 0.05f;
        for (size_t i = 0; i < points.size(); ++i) {
            Eigen::Vector3i idx = (points[i] / voxel_size).array().floor().cast<int>();

            if (semantic_map.find(idx) == semantic_map.end()) {
                SemanticVoxel voxel;
                voxel.index = idx;
                voxel.center = idx.cast<float>() * voxel_size + Eigen::Vector3f(0.5f, 0.5f, 0.5f) * voxel_size;
                voxel.color = colors[i];
                voxel.alpha = 1.0f;
                semantic_map[idx] = voxel;
            } else {
                semantic_map[idx].color = 0.5f * (semantic_map[idx].color + colors[i]);
            }
        }
    }

    rclcpp::init(argc, argv);
    auto node = std::make_shared<SemanticVoxelPublisher>(semantic_map);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}