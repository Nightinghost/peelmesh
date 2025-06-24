#include <gtest/gtest.h>
#include <peelmesh/geometry.hpp>

#include <unordered_set>

using namespace peelmesh;

class WatertightTriangleMeshTest : public ::testing::Test
{
protected:
    void SetUp() override
    {
        verts = {{0, 0, 0}, {1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
        tris = {{0, 2, 1}, {0, 1, 3}, {0, 3, 2}, {1, 2, 3}};
        mesh.InitMesh(verts, tris);
    }

    TriangleMesh mesh;
    std::vector<Eigen::Vector3d> verts;
    std::vector<Eigen::Vector3i> tris;
};

TEST_F(WatertightTriangleMeshTest, MeshInitialization)
{
    EXPECT_EQ(mesh.vertices.size(), 4);
    EXPECT_EQ(mesh.faces.size(), 4);
    EXPECT_EQ(mesh.halfEdges.size(), 12);

    auto *he = mesh.faces[0].halfedge;
    EXPECT_NE(he->twin, nullptr);
    EXPECT_EQ(he->twin->twin, he);

    auto v0 = verts[tris[0][0]], v1 = verts[tris[0][1]], v2 = verts[tris[0][2]];
    auto norm = (v1 - v0).cross(v2 - v0).normalized();
    EXPECT_TRUE(he->face->normal.isApprox(norm));
}

TEST_F(WatertightTriangleMeshTest, AddVertex)
{
    auto *v = mesh.addVertex({5, 5, 5});
    EXPECT_EQ(v->index, 4);
    EXPECT_EQ(mesh.vertices.size(), 5);
    EXPECT_TRUE(v->isActive);
    EXPECT_EQ(v->position, Eigen::Vector3d(5, 5, 5));
}

TEST_F(WatertightTriangleMeshTest, DeleteFace)
{
    mesh.deleteFace(0);
    EXPECT_FALSE(mesh.faces[0].isActive);
    EXPECT_EQ(mesh.freeFaceIndices.size(), 1);

    auto *he = mesh.faces[0].halfedge;
    EXPECT_FALSE(he->isActive);
    EXPECT_FALSE(he->next->isActive);
    EXPECT_FALSE(he->prev->isActive);
}

TEST_F(WatertightTriangleMeshTest, DeleteSingleHalfEdge)
{
    auto edge = mesh.findHalfEdge(0, 1);
    mesh.deleteHalfEdge(edge);

    EXPECT_EQ(mesh.edgeMap.count({0, 1}), 1);
    EXPECT_EQ(mesh.edgeMap.at({0, 1}), edge->twin);

    EXPECT_TRUE(mesh.faces[0].isActive);
    EXPECT_FALSE(mesh.faces[1].isActive);
}

TEST_F(WatertightTriangleMeshTest, DeleteBothHalfEdge)
{
    auto edge = mesh.findHalfEdge(0, 1);
    mesh.deleteHalfEdge(edge, true);

    EXPECT_EQ(mesh.edgeMap.count({0, 1}), 0);

    EXPECT_FALSE(mesh.faces[0].isActive);
    EXPECT_FALSE(mesh.faces[1].isActive);
}

TEST_F(WatertightTriangleMeshTest, MeshDataConsistency)
{
    auto [verts, tris] = mesh.getMeshData();

    EXPECT_EQ(verts.size(), 4);
    EXPECT_EQ(tris.size(), 4);

    EXPECT_EQ(verts[0], Eigen::Vector3d(0, 0, 0));
}

TEST_F(WatertightTriangleMeshTest, FindExistingHalfEdge)
{
    auto he = mesh.findHalfEdge(0, 1);
    ASSERT_NE(he, nullptr);

    EXPECT_EQ(he->target->index, 1);
    ASSERT_NE(he->twin, nullptr);
    EXPECT_EQ(he->twin->target->index, 0);

    EXPECT_NE(mesh.edgeMap.find({0, 1}), mesh.edgeMap.end());
}

TEST_F(WatertightTriangleMeshTest, FindReverseHalfEdge)
{
    auto he = mesh.findHalfEdge(1, 0);
    mesh.PrintMesh();
    ASSERT_NE(he, nullptr);
    EXPECT_EQ(he->target->index, 0);
    EXPECT_EQ(he->twin->target->index, 1);
}

TEST_F(WatertightTriangleMeshTest, FindNonExistentEdge)
{
    mesh.addVertex({5, 5, 5});
    EXPECT_EQ(mesh.findHalfEdge(0, 4), nullptr);
}

TEST_F(WatertightTriangleMeshTest, FindAfterEdgeDeletion)
{
    mesh.deleteEdge(0, 1);
    EXPECT_EQ(mesh.findHalfEdge(0, 1), nullptr);
    EXPECT_EQ(mesh.findHalfEdge(1, 0), nullptr);
}

TEST_F(WatertightTriangleMeshTest, InvalidVertexIndices)
{
    EXPECT_EQ(mesh.findHalfEdge(-1, 0), nullptr);

    EXPECT_EQ(mesh.findHalfEdge(0, 10), nullptr);
}

TEST_F(WatertightTriangleMeshTest, FindAfterVertexDeletion)
{
    mesh.deleteVertex(1);

    EXPECT_EQ(mesh.findHalfEdge(0, 1), nullptr);
    EXPECT_EQ(mesh.findHalfEdge(1, 0), nullptr);
}

TEST(TriangleMeshSpecialCaseTest, PartialMesh)
{
    TriangleMesh mesh;

    mesh.addVertex({0, 0, 0});
    mesh.addVertex({1, 0, 0});

    EXPECT_EQ(mesh.findHalfEdge(0, 1), nullptr);
}

TEST_F(WatertightTriangleMeshTest, VertexIncidentEdgesInitialization)
{
    for (const auto &v : mesh.vertices)
    {
        EXPECT_FALSE(v.incident_halfedges.empty());

        for (TriangleMesh::HalfEdge *he : v.incident_halfedges)
        {
            ASSERT_NE(he, nullptr);
            EXPECT_EQ(he->target->index, v.index);
        }

        EXPECT_EQ(v.incident_halfedges.size(), 3);
    }

    TriangleMesh::Vertex &v0 = mesh.vertices[0];
    std::unordered_set<TriangleMesh::HalfEdge *> unique_edges(v0.incident_halfedges.begin(),
                                                              v0.incident_halfedges.end());
    EXPECT_EQ(unique_edges.size(), v0.incident_halfedges.size());
}

TEST_F(WatertightTriangleMeshTest, EdgeFlipIncidentUpdate)
{
    TriangleMesh::HalfEdge *he = mesh.findHalfEdge(0, 1);
    TriangleMesh::Vertex &v0 = mesh.vertices[0];
    TriangleMesh::Vertex &v1 = mesh.vertices[1];

    const size_t v0_initial = v0.incident_halfedges.size();
    const size_t v1_initial = v1.incident_halfedges.size();

    mesh.flipEdge(0, 1);

    EXPECT_EQ(v0.incident_halfedges.size(), v0_initial - 1);
    EXPECT_EQ(v1.incident_halfedges.size(), v1_initial - 1);

    TriangleMesh::Vertex &v2 = mesh.vertices[2];
    TriangleMesh::Vertex &v3 = mesh.vertices[3];
    EXPECT_EQ(v2.incident_halfedges.size(), 3 + 1);
    EXPECT_EQ(v3.incident_halfedges.size(), 3 + 1);
}

TEST_F(WatertightTriangleMeshTest, VertexOutgoingEdgesInitialization)
{
    for (const auto &v : mesh.vertices)
    {
        EXPECT_FALSE(v.outgoing_halfedges.empty())
            << "Vertex " << v.index << " has no outgoing edges";

        for (TriangleMesh::HalfEdge *he : v.outgoing_halfedges)
        {
            ASSERT_NE(he, nullptr);
            EXPECT_EQ(he->twin->target->index, v.index)
                << "Edge " << he->index << " start mismatch";
        }

        EXPECT_EQ(v.outgoing_halfedges.size(), 3);
    }

    TriangleMesh::Vertex &v0 = mesh.vertices[0];
    std::unordered_set<TriangleMesh::HalfEdge *> unique_edges(v0.outgoing_halfedges.begin(),
                                                              v0.outgoing_halfedges.end());
    EXPECT_EQ(unique_edges.size(), v0.outgoing_halfedges.size());
}

TEST_F(WatertightTriangleMeshTest, EdgeFlipOutgoingUpdate)
{
    TriangleMesh::HalfEdge *he = mesh.findHalfEdge(0, 1);
    TriangleMesh::Vertex &v0 = mesh.vertices[0];
    TriangleMesh::Vertex &v1 = mesh.vertices[1];

    const size_t v0_initial = v0.outgoing_halfedges.size();
    const size_t v1_initial = v1.outgoing_halfedges.size();

    mesh.flipEdge(0, 1);

    EXPECT_EQ(v0.outgoing_halfedges.size(), v0_initial - 1);
    EXPECT_EQ(v1.outgoing_halfedges.size(), v1_initial - 1);

    TriangleMesh::Vertex &v2 = mesh.vertices[2];
    TriangleMesh::Vertex &v3 = mesh.vertices[3];
    EXPECT_EQ(v2.outgoing_halfedges.size(), 3 + 1);
    EXPECT_EQ(v3.outgoing_halfedges.size(), 3 + 1);
}

TEST_F(WatertightTriangleMeshTest, OutgoingEdgesConsistency)
{
    for (const auto &v : mesh.vertices)
    {
        for (TriangleMesh::HalfEdge *he : v.outgoing_halfedges)
        {
            EXPECT_EQ(he->twin->target->index, v.index)
                << "Edge " << he->index << " twin mismatch at vertex " << v.index;

            EXPECT_TRUE(he->isActive)
                << "Inactive edge in outgoing list of vertex " << v.index;
        }
    }
}

TEST_F(WatertightTriangleMeshTest, FaceMapInitialization)
{
    EXPECT_EQ(mesh.faces.size(), 4);
    EXPECT_EQ(mesh.faceMap.size(), 4 * 3);

    EXPECT_EQ(mesh.faceMap.count({0, 1, 3}), 1);
    EXPECT_EQ(mesh.faceMap.count({1, 3, 0}), 1);
    EXPECT_EQ(mesh.faceMap.count({3, 0, 1}), 1);

    EXPECT_EQ(mesh.faceMap.at({0, 1, 3}), &mesh.faces[1]);
    EXPECT_EQ(mesh.faceMap.at({1, 3, 0}), &mesh.faces[1]);
    EXPECT_EQ(mesh.faceMap.at({3, 0, 1}), &mesh.faces[1]);
}

TEST_F(WatertightTriangleMeshTest, FaceMapAfterFaceDeletion)
{
    mesh.deleteFace(1);

    EXPECT_EQ(mesh.faces.size(), 4);
    EXPECT_FALSE(mesh.faces[1].isActive);
    EXPECT_EQ(mesh.freeFaceIndices.size(), 1);
    EXPECT_EQ(mesh.faceMap.size(), 3 * 3);

    EXPECT_EQ(mesh.faceMap.count({0, 1, 3}), 0);
    EXPECT_EQ(mesh.faceMap.count({1, 3, 0}), 0);
    EXPECT_EQ(mesh.faceMap.count({3, 0, 1}), 0);
}

TEST_F(WatertightTriangleMeshTest, FaceMapAfterSingleHalfEdgeDeletion)
{
    mesh.deleteHalfEdge(mesh.findHalfEdge(1, 3));

    EXPECT_EQ(mesh.faces.size(), 4);
    EXPECT_FALSE(mesh.faces[1].isActive);
    EXPECT_EQ(mesh.freeFaceIndices.size(), 1);
    EXPECT_EQ(mesh.faceMap.size(), 3 * 3);

    EXPECT_EQ(mesh.faceMap.count({0, 1, 3}), 0);
    EXPECT_EQ(mesh.faceMap.count({1, 3, 0}), 0);
    EXPECT_EQ(mesh.faceMap.count({3, 0, 1}), 0);
}

TEST_F(WatertightTriangleMeshTest, FaceMapAfterBothHalfEdgeDeletion)
{
    mesh.deleteHalfEdge(mesh.findHalfEdge(1, 3), true);

    EXPECT_EQ(mesh.faces.size(), 4);
    EXPECT_FALSE(mesh.faces[1].isActive);
    EXPECT_FALSE(mesh.faces[3].isActive);
    EXPECT_EQ(mesh.freeFaceIndices.size(), 2);
    EXPECT_EQ(mesh.faceMap.size(), 2 * 3);

    EXPECT_EQ(mesh.faceMap.count({0, 1, 3}), 0);
    EXPECT_EQ(mesh.faceMap.count({1, 3, 0}), 0);
    EXPECT_EQ(mesh.faceMap.count({3, 0, 1}), 0);
    EXPECT_EQ(mesh.faceMap.count({1, 2, 3}), 0);
    EXPECT_EQ(mesh.faceMap.count({3, 1, 2}), 0);
    EXPECT_EQ(mesh.faceMap.count({2, 3, 1}), 0);
}

TEST_F(WatertightTriangleMeshTest, InsertVertex)
{

    auto origin_hf_size = mesh.halfEdges.size();
    auto origin_f_size = mesh.faces.size();

    Eigen::Vector3d v = 0.5 * verts[0] + 0.5 * verts[1];
    mesh.InsertVertexAtEdge(0.5 * verts[0] + 0.5 * verts[1], 0, 1);

    EXPECT_EQ(mesh.vertices.size(), 5);
    EXPECT_EQ(mesh.halfEdges.size(), origin_hf_size + 6);
    EXPECT_EQ(mesh.faces.size(), origin_f_size + 2);

    EXPECT_EQ(mesh.edgeMap.size(), mesh.halfEdges.size() / 2);
    EXPECT_EQ(mesh.faceMap.size(), mesh.faces.size() * 3);

    // auto f0 = mesh.faceMap.at({0,5,3})
    EXPECT_EQ(mesh.faceMap.count({0, 4, 3}), 1);
    EXPECT_EQ(mesh.faceMap.count({0, 2, 4}), 1);
    EXPECT_EQ(mesh.faceMap.count({4, 1, 3}), 1);
    EXPECT_EQ(mesh.faceMap.count({2, 1, 4}), 1);
    EXPECT_EQ(mesh.faceMap.count({0, 1, 3}), 0);
    EXPECT_EQ(mesh.faceMap.count({0, 2, 1}), 0);

    EXPECT_EQ(mesh.faceMap.at({0, 4, 3})->normal, (verts[3] - v).cross(verts[0] - v).normalized());
    EXPECT_EQ(mesh.faceMap.at({0, 4, 3})->normal, mesh.faceMap.at({4, 1, 3})->normal);
}

TEST_F(WatertightTriangleMeshTest, FlipEdge)
{
    auto it = mesh.edgeMap.find({0, 1});
    ASSERT_NE(it, mesh.edgeMap.end());

    mesh.flipEdge(0, 1);

    EXPECT_NE(mesh.edgeMap.find({2, 3}), mesh.edgeMap.end());

    it = mesh.edgeMap.find({2, 3});
    auto *he = it->second;
    EXPECT_EQ(he->face->index, 1);
    EXPECT_EQ(he->twin->face->index, 0);

    auto f = he->face;
    auto v0 = he->target->position, v1 = he->next->target->position, v2 = he->next->next->target->position;
    auto norm = (v1 - v0).cross(v2 - v0).normalized();
    EXPECT_TRUE(f->normal.isApprox(norm));
}

TEST_F(WatertightTriangleMeshTest, QueryVertexAdjacentTriangles)
{
    EXPECT_EQ(mesh.GetAdjacentTriangles(0).size(), 3);
    EXPECT_EQ(mesh.GetAdjacentTriangles(1).size(), 3);
    EXPECT_EQ(mesh.GetAdjacentTriangles(2).size(), 3);
    EXPECT_EQ(mesh.GetAdjacentTriangles(3).size(), 3);
}

int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}