#ifndef PEELMESH_MESHIO_H
#define PEELMESH_MESHIO_H

#include <Eigen/Eigen>

#include <tuple>

namespace peelmesh
{
    std::tuple<std::vector<Eigen::Vector3d>, std::vector<Eigen::Vector3i>> ReadObj(const std::string &filename);
} // namespace peelmesh

#endif /* PEELMESH_MESHIO_H */
