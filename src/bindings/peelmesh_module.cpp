#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/eigen.h>
#include <peelmesh/pipeline.hpp>

namespace py = pybind11;

using namespace peelmesh;

class PyTriangleMesh
{
public:
    std::shared_ptr<TriangleMesh> mesh;

public:
    PyTriangleMesh() = default;
    PyTriangleMesh(const std::vector<Eigen::Vector3d> &verts, const std::vector<Eigen::Vector3i> &tris)
        : mesh(std::make_shared<TriangleMesh>(verts, tris)) {}
    PyTriangleMesh(std::shared_ptr<TriangleMesh> mesh)
        : mesh(mesh) {}

    int numVertices() const { return mesh->numVertices(); }
    int numHalfEdges() const { return mesh->numHalfEdges(); }
    int numFaces() const { return mesh->numFaces(); }

    double SurfaceArea() const { return mesh->SurfaceArea(); }
    std::vector<double> GaussianCurvature() { return mesh->GaussianCurvature(); }
    std::vector<double> MeanCurvature() { return mesh->MeanCurvature(); }
    std::vector<double> MinPrincipalCurvature() { return mesh->MinPrincipalCurvature(); }
    std::vector<double> MaxPrincipalCurvature() { return mesh->MaxPrincipalCurvature(); }

    std::tuple<std::vector<Eigen::Vector3d>, std::vector<Eigen::Vector3i>> getMeshData() const
    {
        return mesh->getMeshData();
    }

    void PrintStatisticalInfo() const
    {
        std::cout << "Statistical Info:\n";
        std::cout << "Number of Vertices: " << numVertices() << "\n";
        std::cout << "Number of HalfEdges: " << numHalfEdges() << "\n";
        std::cout << "Number of Faces: " << numFaces() << "\n";
    }
};

class PyPeelMeshPipeline
{
public:
    PyPeelMeshPipeline(const std::vector<Eigen::Vector3d> &verts, const std::vector<Eigen::Vector3i> &tris)
        : pipe(std::make_unique<PeelMeshPipeline>(verts, tris)) {}

    PyTriangleMesh GetMesh() const { return {PyTriangleMesh(pipe->GetMesh())}; }

    void AddPath(const std::vector<Eigen::Vector3d> &points)
    {
        pipe->AddPath(points);
    }
    void AddGeodesicPath(int start, int end) { pipe->AddGeodesicPath(start, end); }

    PyTriangleMesh PeelOffMesh(const std::vector<int> &boundary_landmarks) const
    {
        auto res = pipe->PeelOffMesh(boundary_landmarks);
        return {res};
    }

    std::vector<PyTriangleMesh> AutoSegmentation() const
    {
        std::vector<PyTriangleMesh> segments;

        auto res = pipe->AutoSegmentation();

        for (const auto &segment : res)
        {
            segments.push_back({segment});
        }

        return segments;
    }

    std::vector<double> GetGeodesicDistances() const { return pipe->GetGeodesicDistances(); }
    double GetGeodesicDistanceBetween(int start, int end) const { return pipe->GetGeodesicDistanceBetween(start, end); }

private:
    std::unique_ptr<PeelMeshPipeline> pipe;
};

PYBIND11_MODULE(peelmesh, m)
{
    m.doc() = "peelmesh pipeline for geodesic bounded region extraction.";

    py::class_<PyTriangleMesh>(m, "TriangleMesh")
        .def(py::init<const std::vector<Eigen::Vector3d> &, const std::vector<Eigen::Vector3i> &>())
        .def("get_mesh_data", &PyTriangleMesh::getMeshData)
        .def("num_vertices", &PyTriangleMesh::numVertices)
        .def("num_halfedges", &PyTriangleMesh::numHalfEdges)
        .def("num_faces", &PyTriangleMesh::numFaces)
        .def("surface_area", &PyTriangleMesh::SurfaceArea)
        .def("gaussian_curvature", &PyTriangleMesh::GaussianCurvature)
        .def("mean_curvature", &PyTriangleMesh::MeanCurvature)
        .def("min_principal_curvature", &PyTriangleMesh::MinPrincipalCurvature)
        .def("max_principal_curvature", &PyTriangleMesh::MaxPrincipalCurvature)
        .def("__str__", &PyTriangleMesh::PrintStatisticalInfo);

    py::class_<PyPeelMeshPipeline>(m, "PeelMeshPipeline")
        .def(py::init<const std::vector<Eigen::Vector3d> &, const std::vector<Eigen::Vector3i> &>())
        .def("get_mesh", &PyPeelMeshPipeline::GetMesh)
        .def("add_path", &PyPeelMeshPipeline::AddPath, py::arg("points"))
        .def("add_geodesic_path", &PyPeelMeshPipeline::AddGeodesicPath, py::arg("start"), py::arg("end"))
        .def("peeloff_mesh", &PyPeelMeshPipeline::PeelOffMesh, py::arg("boundary_landmarks"))
        .def("auto_segmentation", &PyPeelMeshPipeline::AutoSegmentation)
        .def("get_geodesic_distances", &PyPeelMeshPipeline::GetGeodesicDistances)
        .def("get_geodesic_distance_between", &PyPeelMeshPipeline::GetGeodesicDistanceBetween);
}