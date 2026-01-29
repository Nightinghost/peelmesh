#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/eigen.h>
#include <peelmesh/pipeline.hpp>

namespace py = pybind11;

using namespace peelmesh;

struct PyHalfEdge
{
    int v = -1;
    int next_v = -1;
    int prev_v = -1;
    int tri = -1;
};

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

    /// @brief Get the mesh data (vertices and triangles)
    /// @return [vertices, triangles]
    std::tuple<std::vector<Eigen::Vector3d>, std::vector<Eigen::Vector3i>> getMeshData() const
    {
        return mesh->getMeshData();
    }

    std::string StatisticalInfo() const
    {
        std::stringstream ss;
        ss << "Number of Vertices: " << numVertices() << "\n";
        ss << "Number of HalfEdges: " << numHalfEdges() << "\n";
        ss << "Number of Faces: " << numFaces() << "\n";
        return ss.str();
    }

    void PrintStatisticalInfo() const
    {
        std::cout << StatisticalInfo() << "\n";
    }

    /// @brief Flip an edge [start, end] if exists
    /// @param start
    /// @param end
    void FlipEdge(int start, int end)
    {
        mesh->flipEdge(start, end);
    }

    /// @brief Get the indices of vertices in the one-ring of vertex vi, start from vj
    /// @param vi center vertex index
    /// @param vj first vertex index in the one-ring
    /// @return a tuple of two vectors, the first one is the indices of vertices in the counterclockwise order, the second one is the indices of vertices in the clockwise order
    std::tuple<std::vector<int>, std::vector<int>>
    GetOneRingVertexIndicesFrom(int vi, int vj)
    {
        return mesh->GetOneRingNeighborIndicesStartFrom(vi, vj);
    }

    /// @brief Get the indices of vertices in the one-ring of vertex vi
    /// @param v_idx center vertex index
    /// @return a tuple of two vectors, the first one is the indices of vertices in the counterclockwise order, the second one is the indices of vertices in the clockwise order
    std::tuple<std::vector<int>, std::vector<int>> GetOneRingVertexIndices(int v_idx)
    {
        return mesh->GetOneRingNeighborIndices(v_idx);
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

    std::vector<Eigen::Vector3d> GetGeodesicPath(int start, int end) const
    {
        return pipe->GetGeodesicPath(start, end);
    }

    std::vector<Eigen::Vector3d> GetShortestPath(int start, int end) const
    {
        return pipe->GetShortestPath(start, end);
    }

    std::vector<Eigen::Vector3d> GetAddedPath(int index) const
    {
        return pipe->GetPaths().at(index).GetVertexPositions();
    }

    std::vector<std::vector<Eigen::Vector3d>> GetAllAddedPaths() const
    {
        std::vector<std::vector<Eigen::Vector3d>> all_paths;
        for (const auto &path : pipe->GetPaths())
        {
            all_paths.push_back(path.GetVertexPositions());
        }
        return all_paths;
    }

private:
    std::unique_ptr<PeelMeshPipeline> pipe;
};

PYBIND11_MODULE(peelmesh, m)
{
    m.doc() = R"pbdoc(
PeelMesh: Geodesic-bounded region extraction pipeline.

This module provides Python bindings for geodesic-based mesh segmentation and triangle mesh with halfedges.

Classes
--------
TriangleMesh
    Basic halfedge triangle mesh structure with curvature and topology analysis.

PeelMeshPipeline
    High-level segmentation pipeline that supports path-based and geodesic-driven mesh peeling.
)pbdoc";

    // ---- TriangleMesh ----
    py::class_<PyTriangleMesh>(m, "TriangleMesh",
                               R"pbdoc(
Triangle mesh representation.

This class provides geometric and topological operations on 3D meshes,
including curvature computation, surface area evaluation, and one-ring queries.
)pbdoc")
        .def(py::init<const std::vector<Eigen::Vector3d> &, const std::vector<Eigen::Vector3i> &>(),
             R"pbdoc(Create a TriangleMesh from vertex and face arrays.)pbdoc")
        .def("get_mesh_data", &PyTriangleMesh::getMeshData,
             R"pbdoc(Return the underlying vertex and face data as numpy arrays.)pbdoc")
        .def("num_vertices", &PyTriangleMesh::numVertices, "Return number of vertices in the mesh.")
        .def("num_halfedges", &PyTriangleMesh::numHalfEdges, "Return number of half-edges in the mesh.")
        .def("num_faces", &PyTriangleMesh::numFaces, "Return number of faces in the mesh.")
        .def("surface_area", &PyTriangleMesh::SurfaceArea, "Compute total surface area of the mesh.")
        .def("gaussian_curvature", &PyTriangleMesh::GaussianCurvature, "Compute Gaussian curvature at each vertex.")
        .def("mean_curvature", &PyTriangleMesh::MeanCurvature, "Compute mean curvature at each vertex.")
        .def("min_principal_curvature", &PyTriangleMesh::MinPrincipalCurvature, "Compute minimum principal curvature.")
        .def("max_principal_curvature", &PyTriangleMesh::MaxPrincipalCurvature, "Compute maximum principal curvature.")
        .def("flip_edge", &PyTriangleMesh::FlipEdge, py::arg("start"), py::arg("end"), "Flip an edge [start, end] if exists.")
        .def("get_one_ring_vertex_indices_from", &PyTriangleMesh::GetOneRingVertexIndicesFrom,
             py::arg("vi"), py::arg("vj"),
             R"pbdoc(
Return the one-ring vertices around vertex vi starting from neighbor vj.

Parameters
----------
vi : int
    The index of the center vertex.
vj : int
    The index of the neighbor to start from.
)pbdoc")
        .def("get_one_ring_vertex_indices", &PyTriangleMesh::GetOneRingVertexIndices,
             py::arg("v_idx"),
             "Return all vertex indices connected to the given vertex.")
        .def("print_statistical_info", &PyTriangleMesh::PrintStatisticalInfo,
             "Print mesh statistics such as vertex/face counts and area distribution.")
        .def("__str__", &PyTriangleMesh::StatisticalInfo, "Return a string summary of the mesh.");

    // ---- PeelMeshPipeline ----
    py::class_<PyPeelMeshPipeline>(m, "PeelMeshPipeline",
                                   R"pbdoc(
Pipeline for geodesic-bounded mesh segmentation.

This class encapsulates mesh peeling, path construction, and geodesic computations.
Typical usage involves loading a mesh, adding boundary paths, and performing peel-off segmentation.
)pbdoc")
        .def(py::init<const std::vector<Eigen::Vector3d> &, const std::vector<Eigen::Vector3i> &>(),
             "Construct a pipeline from mesh vertices and faces.")
        .def("get_mesh", &PyPeelMeshPipeline::GetMesh,
             "Return the underlying TriangleMesh object.")
        .def("add_path", &PyPeelMeshPipeline::AddPath,
             py::arg("points"),
             "Add a sequence of points as user-defined path as boundary input.")
        .def("add_geodesic_path", &PyPeelMeshPipeline::AddGeodesicPath,
             py::arg("start"), py::arg("end"),
             "Add a path following the shortest geodesic between two vertices.")
        .def("peeloff_mesh", &PyPeelMeshPipeline::PeelOffMesh,
             py::arg("boundary_landmarks"),
             "Perform mesh peeling based on provided boundary landmarks.")
        .def("auto_segmentation", &PyPeelMeshPipeline::AutoSegmentation,
             "Perform automatic segmentation using pre-defined geodesic criteria.")
        .def("get_geodesic_distances", &PyPeelMeshPipeline::GetGeodesicDistances,
             "Compute geodesic distance field over the mesh.")
        .def("get_geodesic_distance_between", &PyPeelMeshPipeline::GetGeodesicDistanceBetween,
             "Compute geodesic distance between two given vertices.")
        .def("get_geodesic_path", &PyPeelMeshPipeline::GetGeodesicPath,
             py::arg("start"), py::arg("end"),
             "Return the geodesic path between two vertices.")
        .def("get_shortest_path", &PyPeelMeshPipeline::GetShortestPath,
             py::arg("start"), py::arg("end"),
             "Return the shortest path between two vertices.")
        .def("get_added_path", &PyPeelMeshPipeline::GetAddedPath,
             py::arg("index"),
             "Return the added path at the specified index.")
        .def("get_all_added_paths", &PyPeelMeshPipeline::GetAllAddedPaths,
             "Return all added paths as a list of point sequences.");
}