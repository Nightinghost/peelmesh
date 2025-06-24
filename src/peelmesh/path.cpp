#include <peelmesh/path.hpp>

namespace peelmesh
{
    void Path::AddVertex(TriangleMesh::Vertex *v)
    {
        if (vertices_.empty())
        {
            vertices_.push_back(v);
        }
        else if (v->index != vertices_.back()->index)
        {
            vertices_.push_back(v);
        }
    }

    void Path::InsertVertex(int pos, TriangleMesh::Vertex *v)
    {
        if (pos <= vertices_.size())
        {
            vertices_.insert(vertices_.begin() + pos, v);
        }
    }

    void Path::InsertVertices(int pos, std::vector<TriangleMesh::Vertex *> &vertices)
    {
        if (pos <= vertices_.size())
        {
            vertices_.insert(vertices_.begin() + pos, vertices.begin(), vertices.end());
        }
    }

    void Path::InsertVertexBetween(TriangleMesh::Vertex *v1, TriangleMesh::Vertex *v2, TriangleMesh::Vertex *v)
    {
        auto v1_it = std::find(vertices_.begin(), vertices_.end(), v1);
        auto v2_it = std::find(vertices_.begin(), vertices_.end(), v2);
        if (v1_it != vertices_.end() && v2_it != vertices_.end())
        {
            if (v1_it > v2_it)
                std::swap(v1_it, v2_it);
            vertices_.insert(v1_it + 1, v);
        }
    }

    bool Path::Contains(TriangleMesh::Vertex *v)
    {
        auto it = std::find(vertices_.begin(), vertices_.end(), v);
        return it != vertices_.end();
    }

    bool Path::IsTopologicalCorrect() const
    {
        for (int i = 0; i < vertices_.size() - 1; i++)
        {
            auto he = mesh_->findEdge(vertices_[i]->index, vertices_[i + 1]->index);

            if (he == nullptr)
            {
                std::cerr << "Edge not found in mesh: " << vertices_[i]->index << " " << vertices_[i + 1]->index << std::endl;
                return false;
            }

            if (he->path_id == -1)
                return false;
            if (he->twin != nullptr && he->twin->path_id == -1)
                return false;
        }
        return true;
    }

    bool Path::IsLoop() const
    {
        return vertices_.at(0) == vertices_.back();
    }

    bool Path::IsPathOverlap() const
    {
        std::unordered_set<Eigen::Vector2i> edgeMap;

        for (int i = 0; i < vertices_.size() - 1; i++)
        {
            int a = vertices_[i]->index;
            int b = vertices_[i + 1]->index;
            if (a > b)
                std::swap(a, b);

            if (edgeMap.count({a, b}) == 0)
                edgeMap.insert({a, b});
            else
                return true;
        }
        return false;
    }

    void Path::UpdateBoundaryHalfedges()
    {
        for (int i = 0; i < vertices_.size() - 1; i++)
        {
            auto he = mesh_->findEdge(vertices_[i]->index, vertices_[i + 1]->index);
            if (he != nullptr)
            {
                he->path_id = index;
                if (he->twin != nullptr)
                    he->twin->path_id = index;
            }
        }
    }

    double Path::Length() const
    {
        double length = 0.0;
        for (int i = 0; i < vertices_.size() - 1; i++)
        {
            length += (vertices_[i + 1]->position - vertices_[i]->position).norm();
        }
        return length;
    }
} // namespace peelmesh
