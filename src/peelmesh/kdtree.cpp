#include <peelmesh/kdtree.hpp>
#include <peelmesh/geometry.hpp>

namespace peelmesh
{
    DynamicKDTree::DynamicKDTree(TriangleMesh *mesh)
        : mesh_(mesh)
    {
        tree_ = std::make_unique<kdtree_t>(3, *mesh,
                                           nanoflann::KDTreeSingleIndexAdaptorParams(10));
    }

    int DynamicKDTree::SearchKNN(const Eigen::Vector3d &query,
                                 int knn,
                                 std::vector<int> &indicies,
                                 std::vector<double> &distances2) const
    {
        if (mesh_->vertices.empty())
        {
            return -1;
        }
        indicies.resize(knn);
        distances2.resize(knn);
        std::vector<size_t> indices_eigen(knn);

        nanoflann::KNNResultSet<double> resultSet(knn);
        resultSet.init(indices_eigen.data(), distances2.data());
        tree_->findNeighbors(resultSet, query.data(), {10});

        int k = resultSet.size();
        indicies.resize(k);
        distances2.resize(k);
        std::copy_n(indices_eigen.begin(), k, indicies.begin());
        return k;
    }
} // namespace peelmesh
