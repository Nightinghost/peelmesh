#include <iostream>
#include <peelmesh/geometry.hpp>

#include <open3d/Open3D.h>

peelmesh::TriangleMesh CreateHexagonMesh()
{
    std::vector<Eigen::Vector3d> verts = {{0, 0, 0},
                                          {1, 0, 0},
                                          {0.5, 1, 0},
                                          {-0.5, 1, 0},
                                          {-1, 0, 0},
                                          {-0.5, -1, 0},
                                          {0.5, -1, 0}};
    std::vector<Eigen::Vector3i> tris = {{0, 1, 2}, {0, 2, 3}, {0, 3, 4}, {0, 4, 5}, {0, 5, 6}, {0, 6, 1}};

    peelmesh::TriangleMesh mesh(verts, tris);
    return mesh;
}

peelmesh::TriangleMesh CreateHexagonMeshWithBoundary()
{
    std::vector<Eigen::Vector3d> verts = {{0, 0, 0},
                                          {1, 0, 0},
                                          {0.5, 1, 0},
                                          {-0.5, 1, 0},
                                          {-1, 0, 0},
                                          {-0.5, -1, 0},
                                          {0.5, -1, 0}};
    std::vector<Eigen::Vector3i> tris = {{0, 1, 2}, {0, 2, 3}, {0, 3, 4}, {0, 4, 5}, {0, 5, 6}};

    peelmesh::TriangleMesh mesh(verts, tris);
    return mesh;
}

void Draw(const peelmesh::TriangleMesh &mesh)
{
    const auto &[verts, tris] = mesh.getMeshData();
    auto o3d_mesh = std::make_shared<open3d::geometry::TriangleMesh>(verts, tris);
    o3d_mesh->ComputeVertexNormals();
    open3d::visualization::DrawGeometries({o3d_mesh});
}

int main()
{
    auto mesh = CreateHexagonMeshWithBoundary();
    Draw(mesh);

    const auto &[n1, n2] = mesh.GetOneRingNeighborIndicesStartFrom(4, 0);

    if (!n1.empty())
    {
        std::cout << "CounterClockWise Neighbors: ";
        std::copy(n1.begin(), n1.end(), std::ostream_iterator<int>(std::cout, " "));
        std::cout << std::endl;
    }
    if (!n2.empty())
    {
        std::cout << "ClockWise Neighbors: ";
        std::copy(n2.begin(), n2.end(), std::ostream_iterator<int>(std::cout, " "));
    }
    return 0;
}