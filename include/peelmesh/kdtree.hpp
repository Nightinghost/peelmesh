#ifndef PEELMESH_KDTREE_H
#define PEELMESH_KDTREE_H

#include <nanoflann.hpp>
#include <Eigen/Eigen>
#include <memory>

namespace peelmesh
{
    class TriangleMesh;

    class DynamicKDTree
    {
        TriangleMesh *mesh_;

        using kdtree_t = nanoflann::KDTreeSingleIndexDynamicAdaptor<
            nanoflann::L2_Simple_Adaptor<double, TriangleMesh>,
            TriangleMesh, 3>;

        std::unique_ptr<kdtree_t> tree_;

    public:
        DynamicKDTree(TriangleMesh *mesh);

        int SearchKNN(const Eigen::Vector3d &query,
                      int knn,
                      std::vector<int> &indices,
                      std::vector<double> &distances2) const;

        inline void AddPoint(int new_point_index)
        {
            tree_->addPoints(new_point_index, new_point_index);
        }

        inline void RemovePoint(int point_index)
        {
            tree_->removePoint(point_index);
        }
    };
} // namespace peelmesh

#endif /* PEELMESH_KDTREE_H */
