#include <peelmesh/pipeline.hpp>

int main()
{
    std::vector<Eigen::Vector3d> vertices = {{0, 0, 0}, {1, 0, 0}, {0, 1, 0}, {1, 1, 0}};
    std::vector<Eigen::Vector3i> triangles = {{0, 1, 2}, {1, 3, 2}};

    auto mesh = std::make_shared<peelmesh::TriangleMesh>(vertices, triangles);

    // Halfedge information of the mesh
    mesh->PrintMesh();

    // Create a pipeline object
    peelmesh::PeelMeshPipeline pipe(mesh);

    // Compute geodesic paths from vertices[start] to vertices[end]
    pipe.AddGeodesicPath(0, 1);
    pipe.AddGeodesicPath(1, 3);
    pipe.AddGeodesicPath(3, 0);

    // Extract the segment bounded by the geodesics based on the given vertices sequence
    auto segment = pipe.PeelOffMesh({0, 1, 3, 0});

    // Get the vertices and triangles of the segmented new mesh
    const auto &[verts, tris] = segment->getMeshData();

    return 0;
}