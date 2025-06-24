#ifndef PEELMESH_GEOMETRY_H
#define PEELMESH_GEOMETRY_H

#include <Eigen/Eigen>

#include <vector>
#include <tuple>
#include <unordered_map>
#include <unordered_set>
#include <iostream>

#include <peelmesh/kdtree.hpp>
#include <peelmesh/solver.hpp>

template <>
struct std::hash<Eigen::Vector2i>
{
    std::size_t operator()(const Eigen::Vector2i &v) const
    {
        return std::hash<int>()(v[0]) ^ std::hash<int>()(v[1]);
    }
};

template <>
struct std::hash<Eigen::Vector3i>
{
    std::size_t operator()(const Eigen::Vector3i &v) const
    {
        return std::hash<int>()(v[0]) ^ std::hash<int>()(v[1]) ^ std::hash<int>()(v[2]);
    }
};

namespace geometrycentral
{
    namespace surface
    {
        class ManifoldSurfaceMesh;
        class VertexPositionGeometry;
    }
};

namespace peelmesh
{
    class TriangleMesh
    {
    public:
        struct HalfEdge;
        struct Face;
        struct Vertex;
        struct Vertex
        {
            Eigen::Vector3d position;
            HalfEdge *halfedge = nullptr;

            bool isActive = true;
            int index = -1;

            std::vector<HalfEdge *> outgoing_halfedges;
            std::vector<HalfEdge *> incident_halfedges;
        };

        struct HalfEdge
        {
            Vertex *target = nullptr;
            Face *face = nullptr;

            // half edge indices
            HalfEdge *next = nullptr;
            HalfEdge *prev = nullptr;
            HalfEdge *twin = nullptr;

            bool isActive = true;
            int index = -1;

            // belongs to which path
            int path_id = -1;
        };

        struct Face
        {
            HalfEdge *halfedge = nullptr;

            bool isActive = true;
            int index = -1;

            Eigen::Vector3d normal = Eigen::Vector3d::Zero();
        };

    public:
        TriangleMesh()
        {
            vertices.reserve(100);
            halfEdges.reserve(100);
            faces.reserve(100);

            tree_ = std::make_unique<DynamicKDTree>(this);
        };
        TriangleMesh(const std::vector<Eigen::Vector3d> &verts, const std::vector<Eigen::Vector3i> &tris);
        TriangleMesh(const std::unordered_set<Face *> &faces);

    public:
        std::vector<Vertex> vertices;
        std::vector<HalfEdge> halfEdges;
        std::vector<Face> faces;

        std::vector<int> freeVertexIndices;
        std::vector<int> freeHalfEdgeIndices;
        std::vector<int> freeFaceIndices;

        std::unordered_map<Eigen::Vector2i, HalfEdge *> edgeMap;
        std::unordered_map<Eigen::Vector3i, Face *> faceMap;

        std::unique_ptr<DynamicKDTree> tree_;

        void InitMesh(const std::vector<Eigen::Vector3d> &verts, const std::vector<Eigen::Vector3i> &tris);

        Vertex *addVertex(const Eigen::Vector3d &position);
        HalfEdge *addHalfEdge(Vertex *target, Face *face = nullptr, HalfEdge *next = nullptr, HalfEdge *prev = nullptr, HalfEdge *twin = nullptr);
        Face *addFace(HalfEdge *halfEdgeIndex);

        HalfEdge *findHalfEdge(int start, int end);
        HalfEdge *findHalfEdge(Vertex *start, Vertex *end);
        HalfEdge *findEdge(int start, int end);

        void deleteVertex(Vertex *vertex);
        void deleteVertex(int vertexIndex);

        void deleteHalfEdge(HalfEdge *halfEdge, bool delete_twin = false);
        void deleteEdge(int start, int end);

        void deleteFace(Face *face);
        void deleteFace(int faceIndex);

        std::unordered_set<Face *> GetAdjacentTriangles(const Vertex *) const;
        std::unordered_set<Face *> GetAdjacentTriangles(int index) const;

        void flipEdge(int start, int end);

        Vertex *InsertVertexAtEdge(const Eigen::Vector3d &position, int start, int end);
        Vertex *InsertVertexAtEdge(const Eigen::Vector3d &position, HalfEdge *he);
        Vertex *InsertVertexAtFace(const Eigen::Vector3d &position, Face *tri);

        // Returns a tuple of vectors containing the vertex positions and triangle indices
        // auto& [verts, tris] = mesh.getMeshData();
        std::tuple<std::vector<Eigen::Vector3d>, std::vector<Eigen::Vector3i>> getMeshData() const;

        void PrintMesh(std::ostream &os = std::cout, bool print_vertex = true, bool print_halfedge = true, bool print_triangle = true);

        inline int SearchKNN(const Eigen::Vector3d &query,
                             int knn,
                             std::vector<int> &indicies,
                             std::vector<double> &distances2) const
        {
            return tree_->SearchKNN(query, knn, indicies, distances2);
        }

        // properties
    public:
        int numVertices() const { return vertices.size(); }
        int numHalfEdges() const { return halfEdges.size(); }
        int numFaces() const { return faces.size(); }

        double SurfaceArea() const;
        std::vector<double> GaussianCurvature();
        std::vector<double> MeanCurvature();
        std::vector<double> MinPrincipalCurvature();
        std::vector<double> MaxPrincipalCurvature();

    private:
        void FreeVertex(Vertex *v);
        void FreeHalfEdge(HalfEdge *he);
        void FreeFace(Face *f);

        void UpdateEdgeMap(HalfEdge *he);
        void UpdateFaceMap(Face *f);
        void EraseFromFaceMap(Face *f);
        void UpdateVertexOutgoingHalfEdges(std::vector<Vertex> &vertices);
        void UpdateVertexOutgoingHalfEdges(std::vector<Vertex *> &vertices);
        void UpdateFaceNormal(Face *f);

        // for nanoflann
    public:
        const TriangleMesh &derived() const { return *this; }
        TriangleMesh &derived() { return *this; }
        inline size_t kdtree_get_point_count() const { return vertices.size(); }
        inline double kdtree_get_pt(const size_t idx, const size_t dim) const
        {
            auto vert = vertices.at(idx);
            return vert.position[dim];
        }

        template <class BBOX>
        bool kdtree_get_bbox(BBOX &bb) const
        {
            return false;
        }

    private:
        std::unique_ptr<GeometrySolver> solver;
    };
} // namespace peelmesh

#endif /* PEELMESH_GEOMETRY_H */
