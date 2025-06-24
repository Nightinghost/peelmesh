#ifndef PEELMESH_PATH_H
#define PEELMESH_PATH_H

#include <peelmesh/geometry.hpp>

namespace peelmesh
{
    class Path
    {
    private:
        std::vector<TriangleMesh::Vertex *> vertices_;
        std::shared_ptr<TriangleMesh> mesh_;

    public:
        int index = -1;

    public:
        explicit Path(std::shared_ptr<TriangleMesh> mesh) : mesh_(mesh) {}

        void AddVertex(TriangleMesh::Vertex *v);
        void InsertVertex(int pos, TriangleMesh::Vertex *v);
        void InsertVertices(int pos, std::vector<TriangleMesh::Vertex *> &vertices);
        void InsertVertexBetween(TriangleMesh::Vertex *v1, TriangleMesh::Vertex *v2, TriangleMesh::Vertex *v);
        bool Contains(TriangleMesh::Vertex *v);

        std::vector<TriangleMesh::Vertex *> GetVertices() const
        {
            return vertices_;
        }
        std::array<TriangleMesh::Vertex *, 2> GetEndpoints() const { return {vertices_.front(), vertices_.back()}; }

        inline bool IsEmpty() const { return vertices_.empty(); }
        inline int Size() const { return vertices_.size(); }

        bool IsLoop() const;
        bool IsTopologicalCorrect() const;
        bool IsPathOverlap() const;

        void UpdateBoundaryHalfedges();

        double Length() const;
    };
}

#endif /* PEELMESH_PATH_H */
