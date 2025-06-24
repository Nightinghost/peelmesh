#ifndef PEELMESH_SOLVER_H
#define PEELMESH_SOLVER_H

#include <geometrycentral/surface/meshio.h>
#include <geometrycentral/surface/flip_geodesics.h>
#include <geometrycentral/surface/simple_polygon_mesh.h>
#include <geometrycentral/surface/surface_mesh_factories.h>
#include <geometrycentral/surface/mesh_graph_algorithms.h>
#include <memory>

namespace peelmesh
{
    class Solver
    {
    public:
        Solver() = default;
        Solver(const std::vector<Eigen::Vector3d> &verts, const std::vector<Eigen::Vector3i> &tris);
        virtual ~Solver() = default;
        void SetGeometry(const std::vector<Eigen::Vector3d> &verts, const std::vector<Eigen::Vector3i> &tris);

    protected:
        std::unique_ptr<geometrycentral::surface::ManifoldSurfaceMesh> mesh;
        std::unique_ptr<geometrycentral::surface::VertexPositionGeometry> geom;
    };

    class GeodesicSolver : public Solver
    {
        using Super = Solver;

    public:
        GeodesicSolver() = default;
        GeodesicSolver(const std::vector<Eigen::Vector3d> &verts, const std::vector<Eigen::Vector3i> &tris);
        virtual ~GeodesicSolver() = default;

        void SetGeometry(const std::vector<Eigen::Vector3d> &verts, const std::vector<Eigen::Vector3i> &tris);

        std::vector<Eigen::Vector3d> GetGeodesicPath(int start, int end) const;
        std::vector<Eigen::Vector3d> GetShortestPath(int start, int end) const;

    private:
        std::unique_ptr<geometrycentral::surface::FlipEdgeNetwork> flipNetwork;
    };

    class GeometrySolver : public Solver
    {
        using Super = Solver;

    public:
        GeometrySolver() = default;
        GeometrySolver(const std::vector<Eigen::Vector3d> &verts, const std::vector<Eigen::Vector3i> &tris);
        virtual ~GeometrySolver() = default;

        std::vector<double> GetVertexGaussianCurvature() const;
        std::vector<double> GetVertexMeanCurvature() const;
        std::vector<double> GetVertexMinPrincipalCurvature() const;
        std::vector<double> GetVertexMaxPrincipalCurvature() const;
    };
}

#endif /* PEELMESH_SOLVER_H */
