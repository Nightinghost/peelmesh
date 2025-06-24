#include "peelmesh/utility.hpp"
#include "peelmesh/geometry.hpp"
#include <open3d/Open3D.h>

std::filesystem::path GetProgramDirPath()
{
    char exeFullPath[1024];
    std::string strPath = "";
    GetModuleFileName(NULL, exeFullPath, 1024);
    strPath = std::string(exeFullPath);
    int pos = strPath.find_last_of(std::filesystem::path::preferred_separator, strPath.length());
    return std::filesystem::path(strPath.substr(0, pos));
}

MeshInfo load_mesh_and_landmarks(const std::filesystem::path &mesh_file)
{
    std::fstream in(mesh_file);
    if (!in.is_open())
    {
        std::cerr << "Cannot open file: " << mesh_file << std::endl;
        return {};
    }
    MeshInfo mesh;
    std::string line;
    while (std::getline(in, line))
    {
        std::stringstream ss(line);
        std::string type;
        ss >> type;
        if (type == "v")
        {
            double x, y, z;
            ss >> x >> y >> z;
            mesh.vertices.push_back({x, y, z});
        }
        else if (type == "f")
        {
            Eigen::Vector3i tri;
            for (int i = 0; i < 3; i++)
            {
                std::string tmp;
                ss >> tmp;
                tri[i] = std::stoi(tmp.substr(0, tmp.find_first_of("/"))) - 1;
            }
            mesh.triangles.push_back(tri);
        }
    }

    auto landmark_file = mesh_file.parent_path() / "landmarks.txt";
    std::ifstream landmark_in(landmark_file);
    while (std::getline(landmark_in, line))
    {
        mesh.landmarks.push_back(std::stoi(line));
    }

    return mesh;
}

Eigen::Vector3d rgb(int r, int g, int b)
{
    return {r / 255.0, g / 255.0, b / 255.0};
}

std::vector<Eigen::Vector3d> colors = {
    rgb(236, 212, 82),
    rgb(177, 213, 200),
    rgb(119, 80, 57),
    rgb(238, 121, 89),
    rgb(221, 118, 148),
    rgb(225, 59, 63),
    rgb(119, 150, 73),
    rgb(69, 70, 94),
    rgb(0, 113, 117),
    rgb(126, 82, 127),
    rgb(184, 53, 112),
    rgb(212, 201, 170),
    rgb(16, 104, 152),
    rgb(248, 198, 181),
    rgb(46, 204, 113)};

void visualize_segments(const std::vector<std::shared_ptr<peelmesh::TriangleMesh>> &segments, peelmesh::Path *boundary, std::unordered_set<peelmesh::TriangleMesh::Vertex *> *vertices)
{
    std::vector<std::shared_ptr<open3d::geometry::Geometry3D>> meshes;

    int i = 0;
    for (const auto &segment : segments)
    {
        const auto &[verts, tris] = segment->getMeshData();
        auto mesh = std::make_shared<open3d::geometry::TriangleMesh>(verts, tris);
        mesh->ComputeVertexNormals();
        mesh->PaintUniformColor(colors[(i++) % colors.size()]);
        meshes.push_back(mesh);
    }

    if (boundary)
    {
        auto lineset = std::make_shared<open3d::geometry::LineSet>();
        auto points = boundary->GetVertices();
        std::for_each(points.begin(), points.end(), [&lineset](const auto &v)
                      { lineset->points_.push_back(v->position); });

        lineset->PaintUniformColor({1, 0, 0});
        meshes.push_back(lineset);
    }

    if (vertices)
    {
        auto pcd = std::make_shared<open3d::geometry::PointCloud>();
        std::for_each(vertices->begin(), vertices->end(), [&](peelmesh::TriangleMesh::Vertex *v)
                      { pcd->points_.push_back(v->position); });

        pcd->PaintUniformColor({0, 0, 1});
        meshes.push_back(pcd);
    }
    open3d::visualization::DrawGeometries({meshes.begin(), meshes.end()});
}

void visualize_pipeline(const peelmesh::PeelMeshPipeline &pipe)
{
    const auto &[verts, tris] = pipe.GetMeshData();
    auto mesh = std::make_shared<open3d::geometry::TriangleMesh>(verts, tris);
    mesh->ComputeVertexNormals();

    std::vector<std::shared_ptr<open3d::geometry::Geometry3D>> geometries;
    geometries.push_back(mesh);

    const auto &paths = pipe.GetPaths();

    for (const auto &path : paths)
    {
        auto lineset = std::make_shared<open3d::geometry::LineSet>();
        auto points = path.GetVertices();

        std::for_each(points.begin(), points.end(), [&lineset](const auto &v)
                      { lineset->points_.push_back(v->position); });
        for (int i = 0; i < points.size() - 1; i++)
            lineset->lines_.push_back({i, i + 1});

        lineset->PaintUniformColor({0, 1, 0});
        if (path.index == paths.size() - 1)
            lineset->PaintUniformColor({1, 0, 0});
        geometries.push_back(lineset);
    }

    open3d::visualization::DrawGeometries({geometries.begin(), geometries.end()});
}