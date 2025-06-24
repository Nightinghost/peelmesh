#ifndef PEELMESH_UTILITY_H
#define PEELMESH_UTILITY_H

#include <peelmesh/pipeline.hpp>
#include <memory>
#include <filesystem>

std::filesystem::path GetProgramDirPath();

struct MeshInfo
{
    std::vector<Eigen::Vector3d> vertices;
    std::vector<Eigen::Vector3i> triangles;
    std::vector<int> landmarks;
};

MeshInfo load_mesh_and_landmarks(const std::filesystem::path &mesh_file);
void visualize_segments(const std::vector<std::shared_ptr<peelmesh::TriangleMesh>> &segments, peelmesh::Path *boundary = nullptr, std::unordered_set<peelmesh::TriangleMesh::Vertex *> *vertices = nullptr);
void visualize_pipeline(const peelmesh::PeelMeshPipeline &pipe);

#endif /* PEELMESH_UTILITY_H */
