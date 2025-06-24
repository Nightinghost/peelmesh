#include <peelmesh/geometry.hpp>

#include <map>
#include <array>

#include <format>
#include <chrono>
#include <fstream>
#include <functional>
#include <memory>

namespace peelmesh
{

    TriangleMesh::Vertex *TriangleMesh::addVertex(const Eigen::Vector3d &position)
    {
        int index = -1;
        if (!freeVertexIndices.empty())
        {
            index = freeVertexIndices.back();
            freeVertexIndices.pop_back();
            vertices[index] = {position, nullptr, true, index};
        }
        else
        {
            index = vertices.size();
            vertices.push_back({position, nullptr, true, index});
        }

        // update kdtree;
        tree_->AddPoint(index);

        return &vertices[index];
    }

    TriangleMesh::HalfEdge *TriangleMesh::addHalfEdge(Vertex *target, Face *face, HalfEdge *next, HalfEdge *prev, HalfEdge *twin)
    {
        int index = -1;
        if (!freeHalfEdgeIndices.empty())
        {
            index = freeHalfEdgeIndices.back();
            freeHalfEdgeIndices.pop_back();
            halfEdges[index] = {target, face, next, prev, twin, true, index};
        }
        else
        {
            index = halfEdges.size();
            halfEdges.push_back({target, face, next, prev, twin, true, index});
        }

        return &halfEdges[index];
    }

    TriangleMesh::Face *TriangleMesh::addFace(HalfEdge *halfEdge)
    {
        int index = -1;
        if (!freeFaceIndices.empty())
        {
            index = freeFaceIndices.back();
            freeFaceIndices.pop_back();
            faces[index] = {halfEdge, true, index};
        }
        else
        {
            index = faces.size();
            faces.push_back({halfEdge, true, index});
        }
        return &faces[index];
    }

    void TriangleMesh::FreeVertex(Vertex *v)
    {
        freeVertexIndices.push_back(v->index);

        v->isActive = false;
    }

    void TriangleMesh::FreeHalfEdge(HalfEdge *he)
    {
        freeHalfEdgeIndices.push_back(he->index);

        auto a = he->prev->target->index, b = he->target->index;
        if (a > b)
            std::swap(a, b);

        if (auto it = edgeMap.find({a, b}); it != edgeMap.end())
        {
            if (it->second->index == he->index)
            {
                edgeMap.erase(it);

                if (he->twin != nullptr && he->twin->isActive)
                {
                    edgeMap[{a, b}] = he->twin;
                }
            }
        }

        he->isActive = false;
    }

    void TriangleMesh::FreeFace(Face *f)
    {
        f->isActive = false;
        freeFaceIndices.push_back(f->index);

        if (f->halfedge == nullptr || f->halfedge->next == nullptr || f->halfedge->prev == nullptr)
            return;

        EraseFromFaceMap(f);
    }

    void TriangleMesh::deleteVertex(Vertex *vertex)
    {
        if (vertex == nullptr || !vertex->isActive)
        {
            return;
        }

        // delete edges
        auto startEdge = vertex->halfedge;
        auto currentEdge = startEdge;

        FreeVertex(vertex);

        for (auto &he : vertex->outgoing_halfedges)
        {
            deleteHalfEdge(he, true);
        }

        // do
        // {
        //     // if (currentEdge == nullptr)
        //     // {
        //     //     break;
        //     // }

        //     // deleteHalfEdge(currentEdge->twin, true);

        //     // auto twin = currentEdge->twin;

        //     // currentEdge = twin->next;

        // } while (currentEdge->index != startEdge->index);

        // update kdtree;
        tree_->RemovePoint(vertex->index);
    }

    void TriangleMesh::deleteVertex(int vertexIndex)
    {
        deleteVertex(&vertices[vertexIndex]);
    }

    void TriangleMesh::deleteHalfEdge(HalfEdge *halfEdge, bool delete_twin)
    {
        if (halfEdge == nullptr || !halfEdge->isActive)
        {
            return;
        }

        auto startEdge = halfEdge;
        auto currentEdge = startEdge;
        do
        {
            FreeHalfEdge(currentEdge);
            currentEdge = currentEdge->next;
        } while (currentEdge->index != startEdge->index);

        FreeFace(currentEdge->face);

        if (halfEdge->twin != nullptr && delete_twin)
        {
            startEdge = halfEdge->twin;
            currentEdge = startEdge;
            do
            {
                FreeHalfEdge(currentEdge);
                currentEdge = currentEdge->next;
            } while (currentEdge->index != startEdge->index);

            FreeFace(currentEdge->face);

            auto start = halfEdge->twin->target->index, end = halfEdge->target->index;
            if (start > end)
                std::swap(start, end);
            edgeMap.erase({start, end});
        }
    }

    void TriangleMesh::deleteEdge(int start, int end)
    {
        if (start > end)
        {
            std::swap(start, end);
        }

        auto &he = edgeMap[{start, end}];

        deleteHalfEdge(he, true);
    }

    void TriangleMesh::deleteFace(Face *face)
    {
        if (!face->isActive)
        {
            return;
        }

        auto startEdge = face->halfedge;
        auto currentEdge = startEdge;

        do
        {
            FreeHalfEdge(currentEdge);
            currentEdge = currentEdge->next;
        } while (currentEdge->index != startEdge->index);

        FreeFace(face);
    }

    void TriangleMesh::deleteFace(int faceIndex)
    {
        deleteFace(&faces[faceIndex]);
    }

    std::unordered_set<TriangleMesh::Face *> TriangleMesh::GetAdjacentTriangles(const Vertex *v) const
    {
        std::unordered_set<Face *> adjacentTriangles;
        for (auto he : v->incident_halfedges)
        {
            adjacentTriangles.insert(he->face);
        }
        return adjacentTriangles;
    }

    std::unordered_set<TriangleMesh::Face *> TriangleMesh::GetAdjacentTriangles(int index) const
    {
        if (index < vertices.size() && index >= 0 && vertices[index].isActive)
        {
            return GetAdjacentTriangles(&vertices[index]);
        }
        return {};
    }

    std::tuple<std::vector<Eigen::Vector3d>, std::vector<Eigen::Vector3i>> TriangleMesh::getMeshData() const
    {
        std::vector<Eigen::Vector3d> verts;
        std::vector<Eigen::Vector3i> tris;

        std::map<int, int> vertexIndexMap;

        for (int i = 0; i < vertices.size(); i++)
        {
            if (vertices[i].isActive)
            {
                verts.push_back(vertices[i].position);
                vertexIndexMap[i] = verts.size() - 1;
            }
        }

        for (int i = 0; i < faces.size(); i++)
        {
            if (faces[i].isActive)
            {
                auto currentEdge = faces[i].halfedge;

                Eigen::Vector3i tri{-1, -1, -1};
                tri[0] = vertexIndexMap[currentEdge->target->index];
                tri[1] = vertexIndexMap[currentEdge->next->target->index];
                tri[2] = vertexIndexMap[currentEdge->prev->target->index];

                tris.push_back(tri);
            }
        }

        return {verts, tris};
    }

    TriangleMesh::TriangleMesh(const std::vector<Eigen::Vector3d> &verts, const std::vector<Eigen::Vector3i> &tris)
    {
        InitMesh(verts, tris);
    }

    TriangleMesh::TriangleMesh(const std::unordered_set<Face *> &faces)
    {
        std::unordered_set<Vertex *> vertices;
        std::vector<Eigen::Vector3d> verts;
        std::vector<Eigen::Vector3i> tris;
        std::map<int, int> old_to_new_v;

        for (auto t : faces)
        {
            auto he = t->halfedge;
            vertices.insert(he->target);
            vertices.insert(he->next->target);
            vertices.insert(he->prev->target);
        }

        int id_ = 0;

        std::for_each(vertices.begin(), vertices.end(), [&](Vertex *v)
                      {
            old_to_new_v.insert({v->index, id_++});
            verts.push_back(v->position); });

        for (auto t : faces)
        {
            auto he = t->halfedge;
            auto v0 = he->target->index;
            auto v1 = he->next->target->index;
            auto v2 = he->prev->target->index;

            tris.push_back({old_to_new_v[v0], old_to_new_v[v1], old_to_new_v[v2]});
        }

        InitMesh(verts, tris);
    }

    void TriangleMesh::UpdateVertexOutgoingHalfEdges(std::vector<Vertex> &vertices)
    {
#pragma omp parallel for
        for (auto &v : vertices)
        {
            v.outgoing_halfedges.clear();
            for (auto he : v.incident_halfedges)
            {
                if (he->next != nullptr)
                    v.outgoing_halfedges.push_back(he->next);
            }
        }
    }

    void TriangleMesh::UpdateVertexOutgoingHalfEdges(std::vector<Vertex *> &vertices)
    {
#pragma omp parallel for
        for (auto &v : vertices)
        {
            v->outgoing_halfedges.clear();
            for (auto he : v->incident_halfedges)
            {
                if (he->next != nullptr)
                    v->outgoing_halfedges.push_back(he->next);
            }
        }
    }

    void TriangleMesh::InitMesh(const std::vector<Eigen::Vector3d> &verts, const std::vector<Eigen::Vector3i> &tris)
    {
        tree_ = std::make_unique<DynamicKDTree>(this);

        vertices.clear();
        halfEdges.clear();
        faces.clear();
        edgeMap.clear();

        vertices.reserve(verts.size() < 1000 ? 2000 : verts.size() * 3);
        halfEdges.reserve(tris.size() < 1000 ? 4000 : tris.size() * 6);
        faces.reserve(tris.size() < 1000 ? 2000 : tris.size() * 3);

        std::for_each(verts.begin(), verts.end(), [this](const Eigen::Vector3d &v)
                      { addVertex(v); });

        for (const auto &tri : tris)
        {
            auto v0 = tri[0];
            auto v1 = tri[1];
            auto v2 = tri[2];

            auto he0 = addHalfEdge(&vertices[v1], nullptr, nullptr, nullptr, nullptr);
            auto he1 = addHalfEdge(&vertices[v2], nullptr, nullptr, nullptr, nullptr);
            auto he2 = addHalfEdge(&vertices[v0], nullptr, nullptr, nullptr, nullptr);
            he0->target->incident_halfedges.push_back(he0);
            he1->target->incident_halfedges.push_back(he1);
            he2->target->incident_halfedges.push_back(he2);

            if (vertices[v0].halfedge == nullptr)
                vertices[v0].halfedge = he0;
            if (vertices[v1].halfedge == nullptr)
                vertices[v1].halfedge = he1;
            if (vertices[v2].halfedge == nullptr)
                vertices[v2].halfedge = he2;

            auto face = addFace(he0);

            std::array<HalfEdge *, 3> vec = {he0, he1, he2};

            for (int i = 0; i < 3; i++)
            {
                auto he = vec[i];
                auto next = vec[(i + 1) % 3];
                auto prev = vec[(i + 2) % 3];

                he->next = next;
                he->prev = prev;
                he->face = face;

                auto s = he->prev->target->index;
                auto t = he->target->index;
                if (s > t)
                {
                    std::swap(s, t);
                }

                if (auto it = edgeMap.find({s, t}); it != edgeMap.end())
                {
                    it->second->twin = he;
                    he->twin = it->second;
                }
                else
                {
                    edgeMap[{s, t}] = he;
                }
            }

            // compute normal after halfedges are set
            UpdateFaceNormal(face);
            UpdateFaceMap(face);
        }
        UpdateVertexOutgoingHalfEdges(vertices);
    }

    TriangleMesh::HalfEdge *TriangleMesh::findEdge(int start, int end)
    {
        if (start > end)
            std::swap(start, end);

        auto it = edgeMap.find({start, end});
        if (it == edgeMap.end())
            return nullptr;
        else
            return it->second;
    }

    TriangleMesh::HalfEdge *TriangleMesh::findHalfEdge(int start, int end)
    {
        int a = start, b = end;
        if (a > b)
        {
            std::swap(a, b);
        }

        auto edge = edgeMap.find({a, b});
        if (edge == edgeMap.end())
        {
            return nullptr;
        }

        if (edge->second->target->index == end)
        {
            return edge->second;
        }
        else
        {
            if (edge->second->twin != nullptr &&
                edge->second->twin->isActive &&
                edge->second->twin->target->index == end)
                return edge->second->twin;
            return nullptr;
        }
    }

    TriangleMesh::HalfEdge *TriangleMesh::findHalfEdge(Vertex *start, Vertex *end)
    {
        return findHalfEdge(start->index, end->index);
    }

    void TriangleMesh::flipEdge(int start, int end)
    {
        auto he = findHalfEdge(start, end);

        if (he == nullptr || he->twin == nullptr)
        {
            return;
        }

        std::vector<Vertex *> changed_vertices = {he->target, he->twin->target, he->next->target, he->twin->next->target};

        auto f0 = he->face;
        auto f1 = he->twin->face;

        EraseFromFaceMap(f0);
        EraseFromFaceMap(f1);

        auto h0 = he;
        auto h1 = he->next;
        auto h2 = he->prev;
        auto h3 = he->twin;
        auto h4 = he->twin->next;
        auto h5 = he->twin->prev;

        h0->target->incident_halfedges.erase(std::remove(h0->target->incident_halfedges.begin(), h0->target->incident_halfedges.end(), h0), h0->target->incident_halfedges.end());
        h3->target->incident_halfedges.erase(std::remove(h3->target->incident_halfedges.begin(), h3->target->incident_halfedges.end(), h3), h3->target->incident_halfedges.end());
        h1->target->incident_halfedges.push_back(h0);
        h4->target->incident_halfedges.push_back(h3);

        he->target->halfedge = h1;
        h1->target->halfedge = h2;
        h2->target->halfedge = h4;
        h4->target->halfedge = h5;

        h0->target = h1->target;
        h0->next = h2;
        h0->prev = h4;
        h0->twin = h3;
        h0->face = f0;

        h2->next = h4;
        h2->prev = h0;
        h2->face = f0;

        h4->next = h0;
        h4->prev = h2;
        h4->face = f0;

        f0->halfedge = h0;

        h3->target = h4->target;
        h3->next = h5;
        h3->prev = h1;
        h3->twin = h0;
        h3->face = f1;

        h5->next = h1;
        h5->prev = h3;
        h5->face = f1;

        h1->next = h3;
        h1->prev = h5;
        h1->face = f1;

        f1->halfedge = h3;

        if (start > end)
            std::swap(start, end);
        edgeMap.erase({start, end});

        start = he->twin->target->index;
        end = he->target->index;
        if (start > end)
            std::swap(start, end);
        edgeMap[{start, end}] = he;

        UpdateVertexOutgoingHalfEdges(changed_vertices);
        UpdateFaceNormal(f0);
        UpdateFaceNormal(f1);
        UpdateFaceMap(f0);
        UpdateFaceMap(f1);
    }

    void TriangleMesh::UpdateFaceMap(Face *f)
    {
        auto v0 = f->halfedge->target;
        auto v1 = f->halfedge->next->target;
        auto v2 = f->halfedge->prev->target;

        faceMap[{v0->index, v1->index, v2->index}] = f;
        faceMap[{v1->index, v2->index, v0->index}] = f;
        faceMap[{v2->index, v0->index, v1->index}] = f;
    }

    void TriangleMesh::EraseFromFaceMap(Face *f)
    {
        auto v0 = f->halfedge->target;
        auto v1 = f->halfedge->next->target;
        auto v2 = f->halfedge->prev->target;

        faceMap.erase({v0->index, v1->index, v2->index});
        faceMap.erase({v1->index, v2->index, v0->index});
        faceMap.erase({v2->index, v0->index, v1->index});
    }

    void TriangleMesh::UpdateEdgeMap(HalfEdge *he)
    {
        auto s = he->prev->target->index;
        auto t = he->target->index;
        if (s > t)
            std::swap(s, t);
        edgeMap[{s, t}] = he;
    }

    TriangleMesh::Vertex *TriangleMesh::InsertVertexAtEdge(const Eigen::Vector3d &position, HalfEdge *he)
    {
        auto v = addVertex(position);

        auto a = he->prev->target->index, b = he->target->index;
        if (a > b)
            std::swap(a, b);
        edgeMap.erase({a, b});

        auto he0 = he;
        auto he0_target = he0->target;
        auto he0_twin = he0->twin;

        auto he1 = he0->next;
        auto he2 = he0->prev;

        FreeHalfEdge(he0);
        FreeFace(he0->face);

        auto he3 = addHalfEdge(v);
        auto he4 = addHalfEdge(he0_target);
        auto he5 = addHalfEdge(he1->target);
        auto he6 = addHalfEdge(v);

        he0_target->incident_halfedges.erase(std::remove(he0_target->incident_halfedges.begin(), he0_target->incident_halfedges.end(), he0), he0_target->incident_halfedges.end());
        he0_target->incident_halfedges.push_back(he4);
        he1->target->incident_halfedges.push_back(he5);
        v->incident_halfedges.push_back(he3);
        v->incident_halfedges.push_back(he6);

        v->halfedge = he4;

        auto f0 = addFace(he3);
        auto f1 = addFace(he4);

        he3->next = he5;
        he3->prev = he2;
        he3->face = f0;

        he2->next = he3;
        he2->prev = he5;
        he2->face = f0;

        he5->next = he2;
        he5->prev = he3;
        he5->face = f0;
        he5->twin = he6;

        he4->next = he1;
        he4->prev = he6;
        he4->face = f1;

        he1->next = he6;
        he1->prev = he4;
        he1->face = f1;

        he6->next = he4;
        he6->prev = he1;
        he6->face = f1;
        he6->twin = he5;

        UpdateEdgeMap(he3);
        UpdateEdgeMap(he4);
        UpdateEdgeMap(he5);

        UpdateFaceNormal(f0);
        UpdateFaceNormal(f1);
        UpdateFaceMap(f0);
        UpdateFaceMap(f1);

        if (he0_twin != nullptr)
        {
            auto he7 = he0_twin;
            auto he7_target = he7->target;

            auto he8 = he7->next;
            auto he9 = he7->prev;

            FreeHalfEdge(he7);
            FreeFace(he7->face);

            auto he10 = addHalfEdge(v);
            auto he11 = addHalfEdge(he7_target);
            auto he12 = addHalfEdge(he8->target);
            auto he13 = addHalfEdge(v);

            he7_target->incident_halfedges.erase(std::remove(he7_target->incident_halfedges.begin(), he7_target->incident_halfedges.end(), he7), he7_target->incident_halfedges.end());
            he7_target->incident_halfedges.push_back(he11);
            he8->target->incident_halfedges.push_back(he12);
            v->incident_halfedges.push_back(he10);
            v->incident_halfedges.push_back(he13);

            auto f2 = addFace(he10);
            auto f3 = addFace(he11);

            he10->next = he12;
            he10->prev = he9;
            he10->twin = he4;
            he4->twin = he10;
            he10->face = f2;

            he9->next = he10;
            he9->prev = he12;
            he9->face = f2;

            he12->next = he9;
            he12->prev = he10;
            he12->face = f2;
            he12->twin = he13;

            he11->next = he8;
            he11->prev = he13;
            he11->twin = he3;
            he3->twin = he11;
            he11->face = f3;

            he8->next = he13;
            he8->prev = he11;
            he8->face = f3;

            he13->next = he11;
            he13->prev = he8;
            he13->face = f3;
            he13->twin = he12;

            UpdateEdgeMap(he12);

            UpdateFaceNormal(f2);
            UpdateFaceNormal(f3);
            UpdateFaceMap(f2);
            UpdateFaceMap(f3);
        }

        std::vector<Vertex *> changed_vertices = {he0_target, he1->target, he2->target, v};
        if (he0_twin != nullptr)
        {
            changed_vertices.push_back(he3->twin->next->target);
        }
        UpdateVertexOutgoingHalfEdges(changed_vertices);
        return &vertices[v->index];
    }

    TriangleMesh::Vertex *TriangleMesh::InsertVertexAtEdge(const Eigen::Vector3d &position, int start, int end)
    {
        auto he = findHalfEdge(start, end);
        return InsertVertexAtEdge(position, he);
    }

    TriangleMesh::Vertex *TriangleMesh::InsertVertexAtFace(const Eigen::Vector3d &position, Face *tri)
    {

        auto he0 = tri->halfedge;
        auto he1 = he0->next;
        auto he2 = he0->prev;

        FreeFace(tri);

        auto v = addVertex(position);

        auto he3 = addHalfEdge(v);
        auto he4 = addHalfEdge(he0->target);
        auto he5 = addHalfEdge(v);
        auto he6 = addHalfEdge(he1->target);
        auto he7 = addHalfEdge(v);
        auto he8 = addHalfEdge(he2->target);

        v->incident_halfedges.push_back(he3);
        v->incident_halfedges.push_back(he5);
        v->incident_halfedges.push_back(he7);

        he0->target->incident_halfedges.push_back(he4);
        he1->target->incident_halfedges.push_back(he6);
        he2->target->incident_halfedges.push_back(he8);

        auto f0 = addFace(he0);
        auto f1 = addFace(he1);
        auto f2 = addFace(he2);

        he0->next = he3;
        he0->prev = he8;

        he3->next = he8;
        he3->prev = he0;
        he3->twin = he4;
        he3->face = f0;

        he8->next = he0;
        he8->prev = he3;
        he8->twin = he7;
        he8->face = f0;

        he1->next = he5;
        he1->prev = he4;

        he5->next = he4;
        he5->prev = he1;
        he5->twin = he6;
        he5->face = f1;

        he4->next = he1;
        he4->prev = he5;
        he4->twin = he3;
        he4->face = f1;

        he2->next = he7;
        he2->prev = he6;

        he7->next = he6;
        he7->prev = he2;
        he7->twin = he8;
        he7->face = f2;

        he6->next = he2;
        he6->prev = he7;
        he6->twin = he5;
        he6->face = f2;

        UpdateEdgeMap(he3);
        UpdateEdgeMap(he5);
        UpdateEdgeMap(he7);

        v->halfedge = he8;

        UpdateFaceNormal(f0);
        UpdateFaceNormal(f1);
        UpdateFaceNormal(f2);
        UpdateFaceMap(f0);
        UpdateFaceMap(f1);
        UpdateFaceMap(f2);

        std::vector<Vertex *> changed_vertices = {he0->target, he1->target, he2->target, v};
        UpdateVertexOutgoingHalfEdges(changed_vertices);

        return &vertices[v->index];
    }

    void TriangleMesh::UpdateFaceNormal(Face *f)
    {
        f->normal = (f->halfedge->next->target->position - f->halfedge->target->position).cross(f->halfedge->prev->target->position - f->halfedge->target->position).normalized();
    }

    void TriangleMesh::PrintMesh(std::ostream &os, bool print_vertex, bool print_halfedge, bool print_triangle)
    {
        if (print_vertex)
        {
            os << "Vertex:\n";
            for (int i = 0; i < vertices.size(); i++)
            {
                auto &v = vertices[i];
                auto &x = v.position;
                os << std::format("{}:\tindex: {}\t pos: ({}, {}, {})\t halfedge: {}\t alive: {}\n", i, v.index, x[0], x[1], x[2], v.halfedge->index, v.isActive);
            }
        }

        if (print_halfedge)
        {
            os << std::format("Halfedges:\tHalfedge Size: {}\tEdgeMap Size: {}\n", halfEdges.size(), edgeMap.size()) << std::endl;
            for (int i = 0; i < halfEdges.size(); i++)
            {
                auto &e = halfEdges[i];
                os << std::format("{}:\t index: {}\t target: {}\t face: {}\t next: {}\t prev: {}\t twin: {}\t alive: {}\n",
                                  i, e.index, e.target->index, e.face->index, e.next->index, e.prev->index,
                                  e.twin != nullptr ? e.twin->index : -1, e.isActive);
            }
        }

        if (print_triangle)
        {
            os << std::format("Triangles:\tFace Size: {}\tFaceMap Size: {}\n", faces.size(), faceMap.size()) << std::endl;
            for (int i = 0; i < faces.size(); i++)
            {
                auto &f = faces[i];
                os << std::format("{}:\t index: {}\t vertices: ({}, {}, {})\thalfedge: {}\t alive: {}\n", i, f.index, f.halfedge->target->index, f.halfedge->next->target->index, f.halfedge->prev->target->index, f.halfedge->index, f.isActive);
            }
        }
        os << std::endl;
    }

    double TriangleMesh::SurfaceArea() const
    {
        double area = 0.0;
        for (auto &f : faces)
        {
            auto v0 = f.halfedge->target->position;
            auto v1 = f.halfedge->next->target->position;
            auto v2 = f.halfedge->prev->target->position;
            area += 0.5 * (v1 - v0).cross(v2 - v0).norm();
        }
        return area;
    }

    std::vector<double> TriangleMesh::GaussianCurvature()
    {
        if (solver == nullptr)
        {
            const auto &[verts, tris] = getMeshData();
            solver = std::make_unique<GeometrySolver>(verts, tris);
        }

        return solver->GetVertexGaussianCurvature();
    }
    std::vector<double> TriangleMesh::MeanCurvature()
    {
        if (!solver)
        {
            const auto &[verts, tris] = getMeshData();
            solver = std::make_unique<GeometrySolver>(verts, tris);
        }
        return solver->GetVertexMeanCurvature();
    }
    std::vector<double> TriangleMesh::MinPrincipalCurvature()
    {
        if (!solver)
        {
            const auto &[verts, tris] = getMeshData();
            solver = std::make_unique<GeometrySolver>(verts, tris);
        }
        return solver->GetVertexMinPrincipalCurvature();
    }
    std::vector<double> TriangleMesh::MaxPrincipalCurvature()
    {
        if (!solver)
        {
            const auto &[verts, tris] = getMeshData();
            solver = std::make_unique<GeometrySolver>(verts, tris);
        }
        return solver->GetVertexMaxPrincipalCurvature();
    }

} // namespace peelmesh