#include <gtest/gtest.h>
#include <peelmesh/pipeline.hpp>

using namespace peelmesh;

// ============================================================================
// Test Fixture: coplanar strip mesh
//
//   v3=(0,2) -- v4=(1,2) -- v5=(2,2)
//     |    \      |     /    |
//     |      \    |   /      |
//     |        \  | /        |
//   v0=(0,0) -- v1=(1,0) -- v2=(2,0)
//
// Triangles: (0,1,4), (0,4,3), (1,2,4), (2,5,4)
//
// Key property: segment v0→v2 passes through vertex v1 (not an edge interior),
// which is exactly the case that triggers the Phase 1 collinearity fallback in
// ProcessMultiCrossEdge.
// ============================================================================
class PipelineStripMeshTest : public ::testing::Test
{
protected:
    void SetUp() override
    {
        verts = {{0, 0, 0}, {1, 0, 0}, {2, 0, 0}, {0, 2, 0}, {1, 2, 0}, {2, 2, 0}};
        tris  = {{0, 1, 4}, {0, 4, 3}, {1, 2, 4}, {2, 5, 4}};
    }

    std::vector<Eigen::Vector3d> verts;
    std::vector<Eigen::Vector3i> tris;
};

// ============================================================================
// Test Fixture: square mesh with single diagonal
//
//   v3=(0,2) ----- v2=(2,2)
//     |         /    |
//     |       /      |
//     |     /        |
//     |   /          |
//     | /            |
//   v0=(0,0) ----- v1=(2,0)
//
// Triangles: (0,1,3), (1,2,3)   — diagonal edge is (1,3)
//
// Key property: segment v0→v2 crosses edge (1,3) at its interior point (1,1).
// This tests the normal intersection path (no fallback needed).
// ============================================================================
class PipelineSquareMeshTest : public ::testing::Test
{
protected:
    void SetUp() override
    {
        verts = {{0, 0, 0}, {2, 0, 0}, {2, 2, 0}, {0, 2, 0}};
        tris  = {{0, 1, 3}, {1, 2, 3}};
    }

    std::vector<Eigen::Vector3d> verts;
    std::vector<Eigen::Vector3i> tris;
};

// ---------------------------------------------------------------------------
// Test: segment passes through a vertex (Phase 1 collinearity fallback)
//
// Path v0 → v2: the 3D segment (0,0)→(2,0) lies on y=0 and passes exactly
// through v1=(1,0).  GetIntersection() returns Zero for both opposite edges
// in v0's one-ring — edge (1,4) because (v1-start) ∥ (end-start) makes the
// cross product vanish, and edge (4,3) because the segment does not overlap
// it at all.  Phase 1 detects v1 is collinear and walks to it.
// ---------------------------------------------------------------------------
TEST_F(PipelineStripMeshTest, SegmentPassesThroughVertex)
{
    PeelMeshPipeline pipe(verts, tris);

    // v0 → v2: no direct edge, line passes through v1
    pipe.AddPath({verts[0], verts[2]});

    auto paths = pipe.GetPaths();
    ASSERT_EQ(paths.size(), 1);
    EXPECT_TRUE(paths[0].IsTopologicalCorrect());

    auto pathVerts = paths[0].GetVertices();
    ASSERT_GE(pathVerts.size(), 3u)
        << "Expected at least 3 vertices (v0, v1, v2) in the resolved path";
    EXPECT_EQ(pathVerts[0]->index, 0); // v0
    EXPECT_EQ(pathVerts[1]->index, 1); // v1 (walked through)
    EXPECT_EQ(pathVerts[2]->index, 2); // v2
}

// ---------------------------------------------------------------------------
// Test: normal edge-interior crossing (no fallback triggered)
//
// Path v0 → v2: the 3D segment (0,0)→(2,2) crosses the diagonal edge (1,3)
// at (1,1).  GetIntersection() correctly returns the intersection point.
// A new vertex is inserted there.
// ---------------------------------------------------------------------------
TEST_F(PipelineSquareMeshTest, SegmentCrossesEdgeInterior)
{
    PeelMeshPipeline pipe(verts, tris);

    // v0 → v2: segment crosses the diagonal edge interior
    pipe.AddPath({verts[0], verts[2]});

    auto paths = pipe.GetPaths();
    ASSERT_EQ(paths.size(), 1);
    EXPECT_TRUE(paths[0].IsTopologicalCorrect());

    auto pathVerts = paths[0].GetVertices();
    ASSERT_EQ(pathVerts.size(), 3u)
        << "Expected 3 vertices: v0, new vertex at (1,1), v2";
    EXPECT_EQ(pathVerts[0]->index, 0); // v0
    EXPECT_EQ(pathVerts[2]->index, 2); // v2

    // The middle vertex should be the newly inserted one (index 4)
    EXPECT_EQ(pathVerts[1]->index, 4);
    EXPECT_TRUE(pathVerts[1]->position.isApprox(
        Eigen::Vector3d(1.0, 1.0, 0.0), 1e-9));
}

// ---------------------------------------------------------------------------
// Test: vertices already connected by a direct edge
//
// ProcessMultiCrossEdge should never be entered because findEdge succeeds.
// ---------------------------------------------------------------------------
TEST_F(PipelineStripMeshTest, AlreadyConnectedVertices)
{
    PeelMeshPipeline pipe(verts, tris);

    // v0 → v1: share a direct edge
    pipe.AddPath({verts[0], verts[1]});

    auto paths = pipe.GetPaths();
    ASSERT_EQ(paths.size(), 1);
    EXPECT_TRUE(paths[0].IsTopologicalCorrect());

    auto pathVerts = paths[0].GetVertices();
    ASSERT_EQ(pathVerts.size(), 2u)
        << "Expected exactly 2 vertices for a direct edge";
    EXPECT_EQ(pathVerts[0]->index, 0);
    EXPECT_EQ(pathVerts[1]->index, 1);
}

// ---------------------------------------------------------------------------
// Test: multi-segment path with no ProcessMultiCrossEdge needed
//
// Every consecutive pair in the path already shares a mesh edge.
// ---------------------------------------------------------------------------
TEST_F(PipelineStripMeshTest, MultiSegmentAlreadyConnected)
{
    PeelMeshPipeline pipe(verts, tris);

    // v0 → v1 → v4 → v5: each consecutive pair shares a direct edge
    pipe.AddPath({verts[0], verts[1], verts[4], verts[5]});

    auto paths = pipe.GetPaths();
    ASSERT_EQ(paths.size(), 1);
    EXPECT_TRUE(paths[0].IsTopologicalCorrect());
    EXPECT_FALSE(paths[0].IsPathOverlap());

    auto pathVerts = paths[0].GetVertices();
    ASSERT_EQ(pathVerts.size(), 4u);
    EXPECT_EQ(pathVerts[0]->index, 0);
    EXPECT_EQ(pathVerts[1]->index, 1);
    EXPECT_EQ(pathVerts[2]->index, 4);
    EXPECT_EQ(pathVerts[3]->index, 5);
}

// ---------------------------------------------------------------------------
// Test: path closed as a loop
// ---------------------------------------------------------------------------
TEST_F(PipelineSquareMeshTest, ClosedLoopPath)
{
    PeelMeshPipeline pipe(verts, tris);

    // v0 → v1 → v2 → v3 → v0 (all direct edges exist)
    pipe.AddPath({verts[0], verts[1], verts[2], verts[3], verts[0]});

    auto paths = pipe.GetPaths();
    ASSERT_EQ(paths.size(), 1);
    EXPECT_TRUE(paths[0].IsLoop());
    EXPECT_TRUE(paths[0].IsTopologicalCorrect());
}

// ---------------------------------------------------------------------------
// Test: ProcessMultiCrossEdge reaches max_iteration on a malformed input
//
// When start and end are on disconnected components, the algorithm should
// break (not throw, after our fix) rather than loop forever.
// ---------------------------------------------------------------------------
TEST(PipelineDisconnectedMeshTest, DisconnectedVertices)
{
    // Two disconnected triangles
    std::vector<Eigen::Vector3d> verts = {
        {0, 0, 0}, {1, 0, 0}, {0, 1, 0},  // triangle A
        {5, 0, 0}, {6, 0, 0}, {5, 1, 0}   // triangle B (disconnected)
    };
    std::vector<Eigen::Vector3i> tris = {
        {0, 1, 2},
        {3, 4, 5}
    };

    PeelMeshPipeline pipe(verts, tris);

    // v0 (triangle A) → v3 (triangle B): no path exists on the mesh
    // ProcessMultiCrossEdge should detect it's stuck and break, not throw
    EXPECT_NO_THROW(pipe.AddPath({verts[0], verts[3]}));

    auto paths = pipe.GetPaths();
    ASSERT_EQ(paths.size(), 1);
    // The path may be incomplete but should not crash
    EXPECT_FALSE(paths[0].IsTopologicalCorrect())
        << "Path between disconnected components cannot be topologically correct";
}

// ---------------------------------------------------------------------------
// Test: path with an intermediate point that snaps to an edge interior
// (exercises AddPath's edge-insertion branch, then ProcessMultiCrossEdge)
// ---------------------------------------------------------------------------
TEST_F(PipelineStripMeshTest, IntermediatePointOnEdge)
{
    PeelMeshPipeline pipe(verts, tris);

    // Point at (0.5, 0.5, 0) lies on the interior of a triangle or edge
    // v0 → mid_point → v2
    Eigen::Vector3d mid(0.5, 0.5, 0.0);
    pipe.AddPath({verts[0], mid, verts[2]});

    auto paths = pipe.GetPaths();
    ASSERT_EQ(paths.size(), 1);
    EXPECT_TRUE(paths[0].IsTopologicalCorrect());

    // Should not be empty or malformed
    auto pathVerts = paths[0].GetVertices();
    EXPECT_GE(pathVerts.size(), 3u);
    EXPECT_FALSE(paths[0].IsPathOverlap());
}

// ---------------------------------------------------------------------------
// Test: PeelOffMesh after ProcessMultiCrossEdge resolves paths
// ---------------------------------------------------------------------------
TEST_F(PipelineSquareMeshTest, PeelOffAfterPathResolution)
{
    PeelMeshPipeline pipe(verts, tris);

    // Add three geodesic paths forming a closed boundary
    pipe.AddPath({verts[0], verts[1]}); // v0 → v1 (direct edge)
    pipe.AddPath({verts[1], verts[2]}); // v1 → v2 (direct edge)
    pipe.AddPath({verts[2], verts[0]}); // v2 → v0 (crosses diagonal → inserts vertex)

    // Peel off the region bounded by these three paths
    // The boundary landmark sequence is [0, 1, 2, 0]
    auto segment = pipe.PeelOffMesh(std::vector<int>{0, 1, 2, 0});

    // The peeled mesh should contain at least some triangles
    const auto &[segVerts, segTris] = segment->getMeshData();
    EXPECT_GT(segVerts.size(), 0u);
    EXPECT_GT(segTris.size(), 0u);
}

// ---------------------------------------------------------------------------
// Test: Reset() restores the pipeline to its initial state
// ---------------------------------------------------------------------------
TEST_F(PipelineStripMeshTest, ResetRestoresInitialState)
{
    PeelMeshPipeline pipe(verts, tris);

    // Add some paths to modify the pipeline state
    pipe.AddPath({verts[0], verts[1]});
    pipe.AddPath({verts[1], verts[2]});

    ASSERT_EQ(pipe.GetPaths().size(), 2u);

    // Reset the pipeline
    pipe.Reset();

    // After reset, paths should be cleared
    EXPECT_EQ(pipe.GetPaths().size(), 0u);

    // The mesh should be back to the original state (6 vertices, 4 faces)
    auto mesh = pipe.GetMesh();
    EXPECT_EQ(mesh->numVertices(), 6);
    EXPECT_EQ(mesh->numFaces(), 4);

    // The mesh should still be functional after reset
    const auto &[v, t] = mesh->getMeshData();
    EXPECT_EQ(v.size(), 6u);
    EXPECT_EQ(t.size(), 4u);
}

// ---------------------------------------------------------------------------
// Test: Reset() allows re-adding paths and peeling after reset
// ---------------------------------------------------------------------------
TEST_F(PipelineSquareMeshTest, ResetAndReuse)
{
    PeelMeshPipeline pipe(verts, tris);

    // Add a path and peel
    pipe.AddPath({verts[0], verts[1]});
    pipe.AddPath({verts[1], verts[2]});
    pipe.AddPath({verts[2], verts[0]});
    auto segment1 = pipe.PeelOffMesh(std::vector<int>{0, 1, 2, 0});

    // Reset
    pipe.Reset();

    // Re-add paths and peel again — should work identically
    pipe.AddPath({verts[0], verts[1]});
    pipe.AddPath({verts[1], verts[2]});
    pipe.AddPath({verts[2], verts[0]});
    auto segment2 = pipe.PeelOffMesh(std::vector<int>{0, 1, 2, 0});

    const auto &[v1, t1] = segment1->getMeshData();
    const auto &[v2, t2] = segment2->getMeshData();
    EXPECT_EQ(v1.size(), v2.size());
    EXPECT_EQ(t1.size(), t2.size());
}

int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
