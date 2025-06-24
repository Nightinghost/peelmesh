#include <peelmesh/solver.hpp>
#include <format>
namespace peelmesh
{
    Solver::Solver(const std::vector<Eigen::Vector3d> &verts, const std::vector<Eigen::Vector3i> &tris)
    {
        SetGeometry(verts, tris);
    }

    void Solver::SetGeometry(const std::vector<Eigen::Vector3d> &verts, const std::vector<Eigen::Vector3i> &tris)
    {
        using namespace geometrycentral::surface;
        std::vector<std::vector<size_t>> polygons;
        std::vector<geometrycentral::Vector3> coords;
        std::for_each(begin(tris), end(tris), [&](const Eigen::Vector3i &tri)
                      { polygons.push_back({static_cast<size_t>(tri[0]), static_cast<size_t>(tri[1]), static_cast<size_t>(tri[2])}); });
        std::for_each(begin(verts), end(verts), [&](const Eigen::Vector3d &vert)
                      { coords.push_back({vert[0], vert[1], vert[2]}); });

        SimplePolygonMesh spm(polygons, coords);
        auto lvals = makeManifoldSurfaceMeshAndGeometry(spm.polygons, spm.vertexCoordinates);

        mesh = std::move(std::get<0>(lvals));
        geom = std::move(std::get<1>(lvals));

        geom->requireVertexIndices();
        geom->requireEdgeIndices();
        geom->requireFaceIndices();
    }

    GeodesicSolver::GeodesicSolver(const std::vector<Eigen::Vector3d> &verts, const std::vector<Eigen::Vector3i> &tris)
    {
        SetGeometry(verts, tris);
    }

    void GeodesicSolver::SetGeometry(const std::vector<Eigen::Vector3d> &verts, const std::vector<Eigen::Vector3i> &tris)
    {
        using namespace geometrycentral::surface;
        Super::SetGeometry(verts, tris);

        std::unique_ptr<FlipEdgeNetwork> network(new FlipEdgeNetwork(*mesh, *geom, {}));
        flipNetwork = std::move(network);
        flipNetwork->supportRewinding = true;
        flipNetwork->posGeom = geom.get();
    }

    std::vector<Eigen::Vector3d> GeodesicSolver::GetGeodesicPath(int start, int end) const
    {
        if (start == end)
        {
            std::cout << std::format("[PeelMesh]: Geodesic path start and end are the same. Returning empty path.") << std::endl;
            return {};
        }
        using namespace geometrycentral::surface;
        const auto &vStart = mesh->vertex(start);
        const auto &vEnd = mesh->vertex(end);
        std::vector<Halfedge> dijkstra_path = shortestEdgePath(*geom, vStart, vEnd);
        flipNetwork->reinitializePath({dijkstra_path});
        flipNetwork->iterativeShorten();

        auto path_3d = flipNetwork->getPathPolyline3D().front();
        std::vector<Eigen::Vector3d> path;

        for (auto &p : path_3d)
            path.push_back({p.x, p.y, p.z});

        flipNetwork->rewind();

        return path;
    }

    std::vector<Eigen::Vector3d> GeodesicSolver::GetShortestPath(int start, int end) const
    {
        if (start == end)
        {
            std::cout << std::format("[PeelMesh]: Geodesic path start and end are the same. Returning empty path.") << std::endl;
            return {};
        }
        using namespace geometrycentral::surface;
        const auto &vStart = mesh->vertex(start);
        const auto &vEnd = mesh->vertex(end);
        std::vector<Halfedge> dijkstra_path = shortestEdgePath(*geom, vStart, vEnd);

        std::vector<Eigen::Vector3d> points;
        auto first = geom->vertexPositions[start];
        points.push_back({first.x, first.y, first.z});

        for (auto &he : dijkstra_path)
        {
            const auto &v = geom->vertexPositions[he.tipVertex().getIndex()];
            points.push_back({v.x, v.y, v.z});
        }
        auto last = geom->vertexPositions[end];
        points.push_back({last.x, last.y, last.z});
        return points;
    }

    GeometrySolver::GeometrySolver(const std::vector<Eigen::Vector3d> &verts, const std::vector<Eigen::Vector3i> &tris)
    {
        Super::SetGeometry(verts, tris);
    }

    std::vector<double> GeometrySolver::GetVertexGaussianCurvature() const
    {
        geom->requireVertexGaussianCurvatures();

        std::vector<double> res(mesh->nVertices());
        for (int i = 0; i < mesh->nVertices(); i++)
        {
            res[i] = geom->vertexGaussianCurvatures[i];
        }
        geom->unrequireVertexGaussianCurvatures();
        return res;
    }

    std::vector<double> GeometrySolver::GetVertexMeanCurvature() const
    {
        geom->requireVertexMeanCurvatures();
        std::vector<double> res(mesh->nVertices());
        for (int i = 0; i < mesh->nVertices(); i++)
        {
            res[i] = geom->vertexMeanCurvatures[i];
        }
        geom->unrequireVertexMeanCurvatures();
        return res;
    }

    std::vector<double> GeometrySolver::GetVertexMinPrincipalCurvature() const
    {
        geom->requireVertexMinPrincipalCurvatures();
        std::vector<double> res(mesh->nVertices());
        for (int i = 0; i < mesh->nVertices(); i++)
        {
            res[i] = geom->vertexMinPrincipalCurvatures[i];
        }
        geom->unrequireVertexMinPrincipalCurvatures();
        return res;
    }

    std::vector<double> GeometrySolver::GetVertexMaxPrincipalCurvature() const
    {
        geom->requireVertexMaxPrincipalCurvatures();
        std::vector<double> res(mesh->nVertices());
        for (int i = 0; i < mesh->nVertices(); i++)
        {
            res[i] = geom->vertexMaxPrincipalCurvatures[i];
        }
        geom->unrequireVertexMaxPrincipalCurvatures();
        return res;
    }
}