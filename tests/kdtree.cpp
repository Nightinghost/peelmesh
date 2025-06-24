#include <gtest/gtest.h>
#include <random>
#include <chrono>

#include <peelmesh/geometry.hpp>
#include <peelmesh/kdtree.hpp>
using namespace peelmesh;

class TriangleMeshKDTreeTest : public ::testing::Test
{
protected:
    void SetUp() override
    {
        std::vector<Eigen::Vector3d> grid;
        for (int x = 0; x < 3; ++x)
        {
            for (int y = 0; y < 3; ++y)
            {
                grid.emplace_back(x, y, 0);
            }
        }
        mesh.InitMesh(grid, {});
    }

    TriangleMesh mesh;
};

TEST_F(TriangleMeshKDTreeTest, BasicKNN)
{
    std::vector<int> indices;
    std::vector<double> dists;

    mesh.SearchKNN({1.0, 1.0, 0}, 1, indices, dists);
    ASSERT_EQ(indices.size(), 1);
    EXPECT_EQ(mesh.vertices[indices[0]].position, Eigen::Vector3d(1, 1, 0));
    EXPECT_DOUBLE_EQ(dists[0], 0.0);

    mesh.SearchKNN({2.6, 2.6, 0}, 3, indices, dists);
    ASSERT_EQ(indices.size(), 3);
    EXPECT_EQ(indices, std::vector<int>({8, 5, 7}));
}

TEST_F(TriangleMeshKDTreeTest, DynamicUpdates)
{

    std::vector<int> indices;
    std::vector<double> dists;
    mesh.SearchKNN({0.6, 0.6, 0}, 1, indices, dists);
    ASSERT_EQ(indices.front(), 4); // (0,0,0)

    auto *newV = mesh.addVertex({0.6, 0.6, 0});
    mesh.SearchKNN({0.6, 0.6, 0}, 1, indices, dists);
    EXPECT_EQ(indices.front(), newV->index);

    mesh.deleteVertex(newV);
    mesh.SearchKNN({0.6, 0.6, 0}, 1, indices, dists);
    ASSERT_EQ(indices.front(), 4);
}

TEST_F(TriangleMeshKDTreeTest, InvalidInputs)
{
    std::vector<int> indices;
    std::vector<double> dists;

    TriangleMesh emptyMesh;
    EXPECT_EQ(emptyMesh.SearchKNN({0, 0, 0}, 1, indices, dists), -1);
}

TEST_F(TriangleMeshKDTreeTest, DistanceAccuracy)
{

    mesh.addVertex({1.0, 2.0, 3.0});
    std::vector<int> indices;
    std::vector<double> dists;

    mesh.SearchKNN({1.5, 2.5, 3.5}, 1, indices, dists);
    const double expected = 0.5 * 0.5 + 0.5 * 0.5 + 0.5 * 0.5;
    EXPECT_DOUBLE_EQ(dists[0], expected);
}

TEST(TriangleMeshKDTreeStressTest, LargeDataset)
{
    TriangleMesh mesh;
    std::mt19937 gen(42);
    std::uniform_real_distribution<double> dist(-100, 100);

    for (int i = 0; i < 10000; ++i)
    {
        mesh.addVertex({dist(gen), dist(gen), dist(gen)});
    }

    std::vector<int> indices;
    std::vector<double> dists;
    auto start = std::chrono::high_resolution_clock::now();
    int found = mesh.SearchKNN({0, 0, 0}, 10, indices, dists);
    auto end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);

    EXPECT_EQ(found, 10);
    EXPECT_LT(duration.count(), 50);

    for (size_t i = 1; i < dists.size(); ++i)
    {
        EXPECT_LE(dists[i - 1], dists[i]);
    }
}

TEST_F(TriangleMeshKDTreeTest, DeletionConsistency)
{
    std::vector<int> initialIndices;
    std::vector<double> initialDists;
    mesh.SearchKNN({2.1, 2.1, 0}, 1, initialIndices, initialDists);

    mesh.deleteVertex(initialIndices[0]);

    std::vector<int> newIndices;
    std::vector<double> newDists;
    mesh.SearchKNN({2.1, 2.1, 0}, 1, newIndices, newDists);
    EXPECT_NE(newIndices[0], initialIndices[0]);
    EXPECT_TRUE(mesh.vertices[newIndices[0]].isActive);
}