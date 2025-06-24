#include <peelmesh/pipeline.hpp>

#include <format>

namespace peelmesh
{
    constexpr double VERTEX_ADSORPTION_DISTANCE_THRESHOLD = 1e-6;
    constexpr double EDGE_ADSORPTION_DISTANCE_THRESHOLD = 1e-6;
    constexpr double DEGENERATE_THRESHOLD = 1e-6;
    using Vertex = TriangleMesh::Vertex;
    using HalfEdge = TriangleMesh::HalfEdge;
    using Face = TriangleMesh::Face;

    PeelMeshPipeline::PeelMeshPipeline(const std::vector<Eigen::Vector3d> &verts, const std::vector<Eigen::Vector3i> &tris)
        : mesh_(std::make_shared<TriangleMesh>(verts, tris)),
          solver_(std::make_unique<GeodesicSolver>(verts, tris))
    {
    }

    PeelMeshPipeline::PeelMeshPipeline(std::shared_ptr<TriangleMesh> mesh)
        : mesh_(mesh)
    {
        const auto &[verts, tris] = mesh->getMeshData();
        solver_ = std::make_unique<GeodesicSolver>(verts, tris);
    }

    void PeelMeshPipeline::AddPath(const std::vector<Eigen::Vector3d> &points)
    {
        Path path{mesh_};
        const Vertex *last_vertex = nullptr;

        std::string out_dir_p = "";

        for (int i = 0; i < points.size(); i++)
        {

            const auto &pos = points[i];
            const auto &[closest, dist2] = GetKNearestNeighbors(pos, 15);

            if (dist2[0] < VERTEX_ADSORPTION_DISTANCE_THRESHOLD)
            {
                auto v = &mesh_->vertices[closest[0]];
                path.AddVertex(v);
                last_vertex = v;
            }
            else
            {
                // Create and insert a new vertex at proper position
                auto neighbor_tris = GetAdjacentTriangles(closest);

                double min_dis_to_plane = std::numeric_limits<double>::max();
                Eigen::Vector3d final_pos;
                Face *final_tri;
                PositionType final_type = PositionType::EXTERIOR;
                PositionTypeResult final_pos_result;

                for (auto tri : neighbor_tris)
                {
                    const auto &[projected_pos, dis_to_plane] = ProjectedPositionOfTriangle(pos, tri);
                    const auto &[type, res] = GetRelationshipWSTTriangle(projected_pos, tri);

                    if (type != PositionType::EXTERIOR && dis_to_plane < min_dis_to_plane)
                    {
                        min_dis_to_plane = dis_to_plane;
                        final_pos = projected_pos;
                        final_tri = tri;
                        final_type = type;
                        final_pos_result = res;
                    }
                }

                const Vertex *v = nullptr;

                switch (final_type)
                {
                case PositionType::VERTEX:
                {
                    auto v = std::get<0>(final_pos_result);
                    path.AddVertex(v);
                    break;
                }
                case PositionType::EDGE:
                {
                    auto he = std::get<1>(final_pos_result);

                    auto he_path_id = he->path_id;
                    auto v0 = he->prev->target;
                    auto v1 = he->target;

                    auto v = mesh_->InsertVertexAtEdge(final_pos, he);
                    path.AddVertex(v);

                    if (he_path_id != -1)
                    {
                        auto &target_path = paths_.at(he_path_id);
                        target_path.InsertVertexBetween(v0, v1, v);

                        auto v0_v_he = mesh_->findHalfEdge(v0->index, v->index);
                        v0_v_he->path_id = he_path_id;
                        if (v0_v_he->twin != nullptr)
                            v0_v_he->twin->path_id = he_path_id;

                        auto v1_v_he = mesh_->findHalfEdge(v1->index, v->index);
                        v1_v_he->path_id = he_path_id;
                        if (v1_v_he->twin != nullptr)
                            v1_v_he->twin->path_id = he_path_id;

                        paths_crossing_ = true;
                    }

                    break;
                }
                case PositionType::INTERIOR:
                {
                    auto v = mesh_->InsertVertexAtFace(final_pos, final_tri);
                    path.AddVertex(v);
                    break;
                }
                case PositionType::EXTERIOR:
                {
                    std::cout << "[PeelMeshPipeline] In AddPath(), this should not happen!" << std::endl;
                    break;
                }
                default:
                    break;
                }
            }
        }

        std::vector<std::pair<int, std::vector<Vertex *>>> new_verts;

        int path_id = paths_.size();
        std::vector<Vertex *> path_verts = path.GetVertices();
        for (int i = 0;;)
        {
            if (i == path_verts.size() - 1)
                break;

            auto edge = mesh_->findEdge(path_verts[i]->index, path_verts[i + 1]->index);

            if (edge != nullptr)
            {
                i++;
                // set path id to edge
                edge->path_id = path_id;
                if (edge->twin != nullptr)
                    edge->twin->path_id = path_id;
                continue;
            }

            try
            {
                auto verts = ProcessMultiCrossEdge(path_verts[i], path_verts[i + 1]);

                int pos = i + 1;
                path_verts.insert(path_verts.begin() + pos, verts.begin(), verts.end());
                path.InsertVertices(pos, verts);
            }
            catch (const std::exception &e)
            {
                std::cerr << e.what() << '\n';
                break;
            }
        }

        bool valid = path.IsTopologicalCorrect();
        if (!valid)
            std::cout << "[PeelMeshPipeline] Path is not topological correct!" << std::endl;

        path.index = path_id;
        paths_.push_back(path);
    }

    void PeelMeshPipeline::AddGeodesicPath(int start, int end)
    {
        auto geodesic = solver_->GetGeodesicPath(start, end);
        AddPath(geodesic);
    }

    void PeelMeshPipeline::AddShortestPath(int start, int end)
    {
        auto path = solver_->GetShortestPath(start, end);
        AddPath(path);
    }

    std::vector<Eigen::Vector3d> PeelMeshPipeline::ComputeGeodesic(int start, int end) const
    {
        return solver_->GetGeodesicPath(start, end);
    }

    std::vector<Eigen::Vector3d> PeelMeshPipeline::ComputeShortestPath(int start, int end) const
    {
        return solver_->GetShortestPath(start, end);
    }

    std::tuple<std::vector<int>, std::vector<double>> PeelMeshPipeline::GetKNearestNeighbors(const Eigen::Vector3d &pos, int knn) const
    {
        std::vector<int> indicies;
        std::vector<double> distances2;
        mesh_->SearchKNN(pos, knn, indicies, distances2);
        return {indicies, distances2};
    }

    std::unordered_set<Face *> PeelMeshPipeline::GetAdjacentTriangles(std::vector<Vertex *> verts) const
    {
        std::unordered_set<Face *> adjacent_tris;
        for (auto &v : verts)
        {
            auto outgoing_halfedges = v->outgoing_halfedges;
            for (auto &he : outgoing_halfedges)
            {
                adjacent_tris.insert(he->face);
            }
        }
        return adjacent_tris;
    }

    std::unordered_set<Face *> PeelMeshPipeline::GetAdjacentTriangles(std::vector<int> verts) const
    {
        std::vector<Vertex *> vertices;
        for (auto &v : verts)
        {
            vertices.push_back(&mesh_->vertices[v]);
        }
        return GetAdjacentTriangles(vertices);
    }

    std::tuple<Eigen::Vector3d, double> PeelMeshPipeline::ProjectedPositionOfTriangle(const Eigen::Vector3d &p, const Face *t) const
    {
        const Eigen::Vector3d &v0 = t->halfedge->target->position;
        const Eigen::Vector3d &v1 = t->halfedge->next->target->position;
        const Eigen::Vector3d &v2 = t->halfedge->next->next->target->position;

        auto normal = t->normal;

        auto dis_to_plane = (p - v0).dot(normal);

        return {p - normal * dis_to_plane, std::abs(dis_to_plane)};
    }

    std::tuple<Eigen::Vector3d, double, bool, bool> PeelMeshPipeline::ProjectedPositionOfEdge(const Eigen::Vector3d &p, const Eigen::Vector2i &edge) const
    {
        auto he = mesh_->findHalfEdge(edge[0], edge[1]);
        return ProjectedPositionOfEdge(p, he);
    }

    std::tuple<Eigen::Vector3d, double, bool, bool> PeelMeshPipeline::ProjectedPositionOfEdge(const Eigen::Vector3d &p, const HalfEdge *he) const
    {
        const auto &v0 = he->prev->target->position;
        const auto &v1 = he->target->position;

        auto normalized_v0_v1 = (v1 - v0).normalized();
        auto edge_length = (v1 - v0).norm();

        auto projected_length = (p - v0).dot(normalized_v0_v1);
        auto dis_to_edge = ((v1 - v0).cross(p - v0)).norm() / edge_length;

        bool in_seg = (projected_length > 0 && projected_length < edge_length) ? true : false;
        bool front = projected_length > edge_length ? true : false;

        return {v0 + projected_length * normalized_v0_v1, dis_to_edge, in_seg, front};
    }

    std::tuple<PeelMeshPipeline::PositionType, PeelMeshPipeline::PositionTypeResult> PeelMeshPipeline::GetRelationshipWSTTriangle(const Eigen::Vector3d &p, Face *t) const
    {
        // at vertex
        auto he = t->halfedge;
        do
        {
            auto v = he->target->position;
            if ((v - p).norm() < VERTEX_ADSORPTION_DISTANCE_THRESHOLD)
                return {PositionType::VERTEX, he->target};
            he = he->next;
        } while (he != t->halfedge);

        // at edge
        he = t->halfedge;
        do
        {
            const auto &[pos, dis, in_seg, front] = ProjectedPositionOfEdge(p, he);
            if (dis < EDGE_ADSORPTION_DISTANCE_THRESHOLD && in_seg)
                return {PositionType::EDGE, he};
            he = he->next;
        } while (he != t->halfedge);

        // at triangle
        // barycentric coordinates
        const Eigen::Vector3d &v0 = he->target->position;
        const Eigen::Vector3d &v1 = he->next->target->position;
        const Eigen::Vector3d &v2 = he->prev->target->position;

        auto e1 = v1 - v0;
        auto e2 = v2 - v0;
        auto v = p - v0;

        auto e1_dot_e1 = e1.dot(e1);
        auto e2_dot_e2 = e2.dot(e2);
        auto e1_dot_e2 = e1.dot(e2);
        auto v_dot_e1 = v.dot(e1);
        auto v_dot_e2 = v.dot(e2);

        auto denominator = e1_dot_e1 * e2_dot_e2 - e1_dot_e2 * e1_dot_e2;

        auto t1 = (v_dot_e1 * e2_dot_e2 - v_dot_e2 * e1_dot_e2) / denominator;
        auto u1 = (v_dot_e2 * e1_dot_e1 - v_dot_e1 * e1_dot_e2) / denominator;

        if (t1 > 0 && u1 > 0 && t1 + u1 < 1)
            return {PositionType::INTERIOR, t};
        else
            return {PositionType::EXTERIOR, t};
    }

    Eigen::Vector3d PeelMeshPipeline::GetIntersection(const Vertex *v1, const Vertex *v2, const Vertex *p1, const Vertex *p2) const
    {
        if (v1 == p1 || v1 == p2 || v2 == p1 || v2 == p2)
            return Eigen::Vector3d::Zero();

        const auto &A = v1->position;
        const auto &B = v2->position;
        const auto &C = p1->position;
        const auto &D = p2->position;

        Eigen::Vector3d AB = B - A;
        Eigen::Vector3d AC = C - A;

        Eigen::Vector3d CD = D - C;

        Eigen::Vector3d n = AB.cross(CD);
        double denominator = n.dot(n);

        if (!((D - C).cross(A - C).dot((D - C).cross(B - C)) < 0 &&
              (B - A).cross(C - A).dot((B - A).cross(D - A)) < 0))
        {
            return Eigen::Vector3d::Zero();
        }

        if (std::abs(denominator) < DEGENERATE_THRESHOLD)
        {
            return Eigen::Vector3d::Zero();
        }

        double t = AC.cross(CD).dot(n) / denominator;
        double u = AC.cross(AB).dot(n) / denominator;

        if (t >= 0 && t <= 1 && u >= 0 && u <= 1)
        {
            return A + t * AB;
        }
        return Eigen::Vector3d::Zero();
    }

    std::vector<Vertex *> PeelMeshPipeline::ProcessMultiCrossEdge(const Vertex *start, const Vertex *end, int max_iteration)
    {
        std::vector<Vertex *> inner_verts;
        int iter = 0;
        while (mesh_->findEdge(start->index, end->index) == nullptr)
        {
            std::unordered_set<Eigen::Vector2i> unique_edges;

            auto tris = mesh_->GetAdjacentTriangles(start);
            for (auto tri : tris)
            {
                auto he = tri->halfedge;
                do
                {
                    auto a = he->prev->target->index;
                    auto b = he->target->index;
                    if (a > b)
                        std::swap(a, b);
                    unique_edges.insert({a, b});
                    he = he->next;
                } while (he != tri->halfedge);
            }

            for (auto &edge : unique_edges)
            {
                auto he = mesh_->findEdge(edge[0], edge[1]);
                auto v0 = he->prev->target;
                auto v1 = he->target;
                auto intersection = GetIntersection(v0, v1, start, end);
                if (intersection != Eigen::Vector3d::Zero())
                {
                    auto path_id = he->path_id;

                    auto v = mesh_->InsertVertexAtEdge(intersection, he);
                    inner_verts.push_back(v);

                    auto new_he = mesh_->findHalfEdge(start->index, v->index);
                    new_he->path_id = paths_.size();
                    if (new_he->twin != nullptr)
                        new_he->twin->path_id = paths_.size();
                    start = v;

                    // may cross other path edges, need insert new vertex between them
                    if (path_id != -1)
                    {
                        paths_crossing_ = true;

                        auto &path = paths_.at(path_id);
                        path.InsertVertexBetween(v0, v1, v);

                        auto v0_v_he = mesh_->findEdge(v0->index, v->index);
                        v0_v_he->path_id = path_id;
                        if (v0_v_he->twin != nullptr)
                            v0_v_he->twin->path_id = path_id;

                        auto v1_v_he = mesh_->findEdge(v1->index, v->index);
                        v1_v_he->path_id = path_id;
                        if (v1_v_he->twin != nullptr)
                            v1_v_he->twin->path_id = path_id;
                    }

                    break;
                }
            }

            if (iter++ > max_iteration)
            {
                throw std::logic_error("PeelMeshPipeline::ProcessMultiCrossEdge() reach max iteration!");
            }
            iter++;
        }

        return inner_verts;
    }

    std::shared_ptr<TriangleMesh> PeelMeshPipeline::PeelOffMesh(const std::vector<std::pair<int, bool>> &boundary_paths) const
    {
        if (paths_crossing_)
        {
            static bool info_printed = false;
            if (!info_printed)
            {
                std::cout << "[PeelMeshPipeline] The mesh has crossing paths, peeling off mesh may not work correctly." << std::endl;
                info_printed = true;
            }
        }

        Path boundary{mesh_};

        for (const auto &[path_id, reverse] : boundary_paths)
        {
            auto path = paths_.at(path_id).GetVertices();
            if (reverse)
            {
                std::for_each(path.crbegin(), path.crend(), [&boundary](Vertex *v)
                              { boundary.AddVertex(v); });
            }
            else
            {
                std::for_each(path.cbegin(), path.cend(), [&boundary](Vertex *v)
                              { boundary.AddVertex(v); });
            }
        }

        if (!boundary.IsLoop())
        {
            std::cerr << "[PeelMeshPipeline] Boundary path is not a loop!\n\tExtraction failed!" << std::endl;
            return std::make_shared<TriangleMesh>();
        }

        if (!boundary.IsTopologicalCorrect())
        {
            std::cerr << "[PeelMeshPipeline] Boundary path is not topological correct!\n\tExtraction failed!" << std::endl;
            return std::make_shared<TriangleMesh>();
        }

        if (boundary.IsPathOverlap())
        {
            std::cerr << "[PeelMeshPipeline] Boundary path is overlap!\n\tExtraction failed!" << std::endl;
            return std::make_shared<TriangleMesh>();
        }

        auto boundary_vertices = boundary.GetVertices();
        if (boundary_vertices.front() == boundary_vertices.back())
            boundary_vertices.pop_back();

        int p_size = boundary_vertices.size();

        std::unordered_set<Face *> res_tris;
        std::unordered_set<Vertex *> inner_vertices;
        std::unordered_map<Vertex *, bool> visited;

        // most outer triangles
        for (int i = 1; i <= p_size; i++)
        {
            int center_idx = boundary_vertices[i % p_size]->index;
            int start_idx = boundary_vertices[(i + 1) % p_size]->index;
            int end_idx = boundary_vertices[i - 1]->index;

            const auto &adj_tris = mesh_->GetAdjacentTriangles(center_idx);
            visited[boundary_vertices[i % p_size]] = true;

            while (start_idx != end_idx)
            {
                auto he = mesh_->findHalfEdge(center_idx, start_idx);
                if (he == nullptr)
                    break;

                auto t = he->face;
                res_tris.insert(t);

                auto inner_vert = he->next->target;
                start_idx = inner_vert->index;

                if (!visited[inner_vert] && !boundary.Contains(inner_vert))
                {
                    inner_vertices.insert(inner_vert);
                    visited[inner_vert] = true;
                }
            }
        }

        // inner triangles
        while (!inner_vertices.empty())
        {
            std::unordered_set<Vertex *> new_inner_vertices;

            for (const auto &v : inner_vertices)
            {
                const auto &adj_tris = mesh_->GetAdjacentTriangles(v->index);
                res_tris.insert(adj_tris.begin(), adj_tris.end());

                for (auto t : adj_tris)
                {
                    auto he = t->halfedge;
                    do
                    {
                        auto v = he->target;
                        if (!visited[v])
                        {
                            new_inner_vertices.insert(v);
                            visited[v] = true;
                        }
                        he = he->next;
                    } while (he != t->halfedge);
                }
            }

            inner_vertices = new_inner_vertices;
        }

        return std::make_shared<TriangleMesh>(res_tris);
    }

    std::shared_ptr<TriangleMesh> PeelMeshPipeline::PeelOffMesh(const std::vector<int> &boundary_points) const
    {
        std::vector<std::pair<int, bool>> res;
        for (int i = 0; i < boundary_points.size() - 1; i++)
        {
            auto it = std::find_if(paths_.begin(), paths_.end(), [&](const Path &p)
                                   { auto endpoints = p.GetEndpoints();
                    auto v0 = endpoints[0]->index;
                    auto v1 = endpoints[1]->index;
                    if ((v0 == boundary_points[i] && v1 == boundary_points[i + 1 ]) || (v0 == boundary_points[i + 1] && v1 == boundary_points[i]))
                    return true;
                    else return false; });

            if (it != paths_.end())
            {
                auto endpoints = (*it).GetEndpoints();
                auto v0 = endpoints[0]->index;
                auto v1 = endpoints[1]->index;

                if (v0 == boundary_points[i] && v1 == boundary_points[(i + 1)])
                    res.push_back({(*it).index, false});
                else
                    res.push_back({(*it).index, true});
            }
            else
            {
                std::cout << std::format("[PeelMeshPipeline] Path [{}]-[{}] is not added to the mesh.\n\tExtraction failed!", boundary_points[i], boundary_points[i + 1]) << std::endl;
                return std::make_shared<TriangleMesh>();
            }
        }

        return PeelOffMesh(res);
    }

    std::vector<std::shared_ptr<TriangleMesh>> PeelMeshPipeline::AutoSegmentation() const
    {
        if (paths_crossing_)
            std::cout << "[PeelMeshPipeline] The mesh has crossing paths, auto segmentation may not work correctly." << std::endl;

        std::vector<std::shared_ptr<TriangleMesh>> segments;

        int tri_size = mesh_->faces.size();
        auto faces = mesh_->faces;

        // std::unordered_map<Face *, bool> classified;
        std::map<int, bool> classified;
        std::for_each(faces.begin(), faces.end(), [&classified](Face &f)
                      { classified[f.index] = false; });

        auto remain = tri_size;
        while (1)
        {
            std::unordered_set<Face *> segment;
            std::unordered_set<Face *> current;

            auto first_not_classified = std::find_if(classified.begin(), classified.end(), [&](auto &p)
                                                     { return p.second == false; });

            if (first_not_classified != classified.end())
            {
                current.insert(&faces[first_not_classified->first]);
            }
            else
                break;

            while (!current.empty())
            {
                std::unordered_set<Face *> new_tris;

                for (const auto &tri : current)
                {
                    segment.insert(tri);
                    classified[tri->index] = true;

                    auto he = tri->halfedge;
                    do
                    {
                        if (he->path_id == -1 && he->twin != nullptr)
                        {
                            auto opposite_face = he->twin->face;
                            if (classified[opposite_face->index] == false)
                                new_tris.insert(opposite_face);
                        }
                        he = he->next;
                    } while (he != tri->halfedge);
                }
                current = new_tris;
            }

            remain -= segment.size();

            if (!segment.empty())
                segments.push_back(std::make_shared<TriangleMesh>(segment));
        }

        return segments;
    }

    std::vector<double> PeelMeshPipeline::GetGeodesicDistances() const
    {
        std::vector<double> distances(paths_.size(), 0.0);

#pragma omp parallel for
        for (int i = 0; i < paths_.size(); i++)
        {
            distances[i] = paths_[i].Length();
        }

        return distances;
    }

    double PeelMeshPipeline::GetGeodesicDistanceBetween(int start, int end) const
    {
        const auto &points = solver_->GetGeodesicPath(start, end);
        if (points.size() < 2)
            return 0.0;

        double distance = 0.0;
        for (int i = 0; i < points.size() - 1; i++)
        {
            distance += (points[i + 1] - points[i]).norm();
        }
        return distance;
    }
} // namespace peelmesh
