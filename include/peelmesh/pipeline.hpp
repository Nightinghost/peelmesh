#ifndef PEELMESH_PIPELINE_H
#define PEELMESH_PIPELINE_H

#include <peelmesh/geometry.hpp>
#include <peelmesh/solver.hpp>
#include <peelmesh/path.hpp>

#include <unordered_set>
#include <variant>

namespace peelmesh
{
    class PeelMeshPipeline
    {
        using Vertex = TriangleMesh::Vertex;
        using HalfEdge = TriangleMesh::HalfEdge;
        using Face = TriangleMesh::Face;

    public:
        enum class PositionType
        {
            INTERIOR, // interior of the triangle
            EXTERIOR, // exterior of the triangle
            EDGE,     // on one of the edge of the triangle
            VERTEX    // on one of the vertex of the triangle
        };

        using PositionTypeResult = std::variant<Vertex *, HalfEdge *, Face *>;

    private:
        std::shared_ptr<TriangleMesh> mesh_;
        std::unique_ptr<GeodesicSolver> solver_;

        std::vector<Path> paths_;

        bool paths_crossing_ = false;

    public:
        explicit PeelMeshPipeline(const std::vector<Eigen::Vector3d> &verts, const std::vector<Eigen::Vector3i> &tris);
        explicit PeelMeshPipeline(std::shared_ptr<TriangleMesh> mesh);

        std::shared_ptr<TriangleMesh> GetMesh() const { return mesh_; }

        void AddPath(const std::vector<Eigen::Vector3d> &points);

        // Compute the geodesic between <start> and <end> and add the path to the mesh.
        void AddGeodesicPath(int start, int end);

        // Compute the shortest path between <start> and <end> and add the path to the mesh.
        void AddShortestPath(int start, int end);

        // Compute the geodesic between <start> and <end> and return the path.
        std::vector<Eigen::Vector3d> ComputeGeodesic(int start, int end) const;

        // Compute the shortest path between <start> and <end> and return the path.
        std::vector<Eigen::Vector3d> ComputeShortestPath(int start, int end) const;

        std::vector<Path> GetPaths() const { return paths_; }

        // Return the updated mesh data.
        // Include the added vertices and faces.
        std::tuple<std::vector<Eigen::Vector3d>, std::vector<Eigen::Vector3i>> GetMeshData() const { return mesh_->getMeshData(); }

        // Peel Off the sub-mesh along the given boundary paths (must be closed).
        // @param boundary_paths: a list of pairs of <path_id, reversed> indicating the boundary path.
        // @attention if there exists a path (id=0) from vertex 0 to vertex 1 in the mesh, and the boundary needed the path to be reversed (from vertex 1 to vertex 0), then boundary_paths = {{0, true}}.
        // @return Return a new TriangleMesh object containing the peeled off sub-mesh.
        std::shared_ptr<TriangleMesh> PeelOffMesh(const std::vector<std::pair<int, bool>> &boundary_paths) const;

        // Peel Off the sub-mesh along the given landmark sequence.
        // @param boundary_points: a list of vertex indices indicating the boundary path, e.g. {0,1,2,0}.
        // @return A new TriangleMesh object containing the peeled off sub-mesh.
        std::shared_ptr<TriangleMesh> PeelOffMesh(const std::vector<int> &boundary_points) const;

        // Auto segmentation of the mesh based on minimal closed region implied by paths.
        // @return A list of TriangleMesh objects containing the sub-meshes.
        std::vector<std::shared_ptr<TriangleMesh>> AutoSegmentation() const;

        // properties
    public:
        // @return The geodesic distances of all paths added to the mesh.
        std::vector<double> GetGeodesicDistances() const;
        // @return The geodesic distance between two vertices.
        double GetGeodesicDistanceBetween(int start, int end) const;

    private:
        std::tuple<std::vector<int>, std::vector<double>> GetKNearestNeighbors(const Eigen::Vector3d &pos, int knn = 1) const;
        std::unordered_set<Face *> GetAdjacentTriangles(std::vector<Vertex *> verts) const;
        std::unordered_set<Face *> GetAdjacentTriangles(std::vector<int> verts) const;

        // @brief Given point p, compute the position of p on the plane of the triangle t.
        // @return [position, distance] The position at the plane, and the distance to the plane.
        std::tuple<Eigen::Vector3d, double> ProjectedPositionOfTriangle(const Eigen::Vector3d &p, const Face *t) const;

        // @brief Given a point p and an edge, compute the position of p on the line of the edge.
        // @return A tuple of <position, distance, in_seg, in_front>.
        // position: the position of p on the line.
        // distance: the distance of p to the line.
        // in_seg: whether p is in the segment of the edge.
        // in_front: whether p is in the front of the edge. the length of (p-v0) is greater than the length of (v1-v0).
        std::tuple<Eigen::Vector3d, double, bool, bool> ProjectedPositionOfEdge(const Eigen::Vector3d &p, const Eigen::Vector2i &edge) const;
        std::tuple<Eigen::Vector3d, double, bool, bool> ProjectedPositionOfEdge(const Eigen::Vector3d &p, const HalfEdge *he) const;

        // Given a point p and a triangle t, compute the position of p on the surface of the triangle, and it's relationship with the triangle.
        // return a tuple of <position_type, position_result>.
        // position_type: the type of the position, see PositionType.
        // position_result: the corresponding element [vertex, halfedge, triangle] with respect to the position_type.
        std::tuple<PositionType, PositionTypeResult> GetRelationshipWSTTriangle(const Eigen::Vector3d &p, Face *t) const;

        // Given a line segment [start, end] which crossing edges, and does not appeared in the topology of the mesh, compute the intersection points.
        std::vector<Vertex *> ProcessMultiCrossEdge(const Vertex *start, const Vertex *end, int max_iteration = 50);

        Eigen::Vector3d GetIntersection(const Vertex *v1, const Vertex *v2, const Vertex *p1, const Vertex *p2) const;
    };
} // namespace peelmesh

#endif /* PEELMESH_PIPELINE_H */
