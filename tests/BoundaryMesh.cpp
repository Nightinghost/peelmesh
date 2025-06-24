#include <gtest/gtest.h>
#include <peelmesh/geometry.hpp>

#include <unordered_set>

using namespace peelmesh;

class PlanarTriangleMeshTest : public ::testing::Test
{
    //  3 ---- 4 --- 5
    //  | \    |    /|
    //  |  \   |   / |
    //  |   \  |  /  |
    //  |    \ | /   |
    //  |     \|/    |
    //  0 ---- 1 --- 2
protected:
    void SetUp() override
    {
        verts = {{0, 0, 0}, {1, 0, 0}, {2, 0, 0}, {1, 0, 0}, {1, 1, 0}, {2, 1, 0}};
        tris = {{0, 1, 3}, {1, 4, 3}, {1, 5, 4}, {1, 2, 5}};
        mesh.InitMesh(verts, tris);
    }

    std::vector<Eigen::Vector3d> verts;
    std::vector<Eigen::Vector3i> tris;
    TriangleMesh mesh;
};

TEST_F(PlanarTriangleMeshTest, MeshInitialization)
{
    EXPECT_EQ(mesh.vertices.size(), 6);
    EXPECT_EQ(mesh.faces.size(), 4);
    EXPECT_EQ(mesh.halfEdges.size(), 12);

    // 验证第一条边的twin关系
    auto *he = mesh.faces[0].halfedge;
    EXPECT_EQ(he->twin, nullptr);
    EXPECT_EQ(he->next->next->next, he);

    auto v0 = verts[tris[0][0]], v1 = verts[tris[0][1]], v2 = verts[tris[0][2]];
    auto norm = (v1 - v0).cross(v2 - v0).normalized();
    EXPECT_TRUE(he->face->normal.isApprox(norm));
}

// 测试顶点添加
TEST_F(PlanarTriangleMeshTest, AddVertex)
{
    auto *v = mesh.addVertex({5, 5, 5});
    EXPECT_EQ(v->index, 6); // 初始4顶点
    EXPECT_EQ(mesh.vertices.size(), 7);
    EXPECT_TRUE(v->isActive);
    EXPECT_EQ(v->position, Eigen::Vector3d(5, 5, 5));
}

// 测试面删除
TEST_F(PlanarTriangleMeshTest, DeleteFace)
{
    mesh.deleteFace(0);
    EXPECT_FALSE(mesh.faces[0].isActive);
    EXPECT_EQ(mesh.freeFaceIndices.size(), 1);

    // 关联的半边应被标记为非活跃
    auto *he = mesh.faces[0].halfedge;
    EXPECT_FALSE(he->isActive);
    EXPECT_FALSE(he->next->isActive);
    EXPECT_FALSE(he->prev->isActive);
}

// 测试单半边删除
TEST_F(PlanarTriangleMeshTest, DeleteSingleHalfEdge)
{
    auto edge = mesh.findHalfEdge(1, 3);
    mesh.deleteHalfEdge(edge);

    // 验证边映射被移除
    EXPECT_EQ(mesh.edgeMap.count({1, 3}), 1);
    EXPECT_EQ(mesh.edgeMap.at({1, 3}), edge->twin);

    // 关联面应被删除
    EXPECT_TRUE(mesh.faces[1].isActive);
    EXPECT_FALSE(mesh.faces[0].isActive);
}

// 测试双半边删除
TEST_F(PlanarTriangleMeshTest, DeleteBothHalfEdge)
{
    auto edge = mesh.findHalfEdge(1, 3);
    mesh.deleteHalfEdge(edge, true);

    // 验证边映射被移除
    EXPECT_EQ(mesh.edgeMap.count({1, 3}), 0);

    // 关联面应被删除
    EXPECT_FALSE(mesh.faces[0].isActive);
    EXPECT_FALSE(mesh.faces[1].isActive);
}

// 验证数据一致性
TEST_F(PlanarTriangleMeshTest, MeshDataConsistency)
{
    auto [verts, tris] = mesh.getMeshData();

    EXPECT_EQ(verts.size(), 6);
    EXPECT_EQ(tris.size(), 4);

    // 验证顶点坐标正确性
    EXPECT_EQ(verts[0], Eigen::Vector3d(0, 0, 0));
}

TEST_F(PlanarTriangleMeshTest, FindExistingHalfEdge)
{
    // 验证边在映射表中存在
    EXPECT_NE(mesh.edgeMap.find({1, 3}), mesh.edgeMap.end());

    // 查找 0->1 的边
    auto he = mesh.findHalfEdge(1, 3);
    ASSERT_NE(he, nullptr);

    // 验证目标顶点和反向边
    EXPECT_EQ(he->target->index, 3);
    ASSERT_NE(he->twin, nullptr);
    EXPECT_EQ(he->twin->target->index, 1);

    EXPECT_NE(mesh.edgeMap.find({0, 1}), mesh.edgeMap.end());

    he = mesh.findHalfEdge(0, 1);
    ASSERT_NE(he, nullptr);
    EXPECT_EQ(he->twin, nullptr);
}

// 测试查找反向边（1->0）
TEST_F(PlanarTriangleMeshTest, FindReverseHalfEdge)
{
    auto he = mesh.findHalfEdge(1, 0);

    ASSERT_EQ(he, nullptr);

    he = mesh.findHalfEdge(3, 1);
    ASSERT_NE(he, nullptr);
    EXPECT_EQ(he->target->index, 1);
    EXPECT_EQ(he->twin->target->index, 3);
}

// 测试查找不存在的边
TEST_F(PlanarTriangleMeshTest, FindNonExistentEdge)
{
    // 添加孤立顶点但不创建边
    mesh.addVertex({5, 5, 5});
    EXPECT_EQ(mesh.findHalfEdge(0, 4), nullptr); // 0->4 不存在
}

// 测试删除边后查找
TEST_F(PlanarTriangleMeshTest, FindAfterEdgeDeletion)
{
    // 删除边 0->1
    mesh.deleteEdge(0, 1);
    EXPECT_EQ(mesh.findHalfEdge(0, 1), nullptr);
    EXPECT_EQ(mesh.findHalfEdge(1, 0), nullptr);
    EXPECT_EQ(mesh.findHalfEdge(0, 3), nullptr);
    EXPECT_EQ(mesh.findHalfEdge(3, 0), nullptr);

    EXPECT_EQ(mesh.findHalfEdge(1, 3), nullptr);
    EXPECT_EQ(mesh.findHalfEdge(3, 1)->target->index, 1);
}

// 测试无效顶点索引
TEST_F(PlanarTriangleMeshTest, InvalidVertexIndices)
{
    // 负索引
    EXPECT_EQ(mesh.findHalfEdge(-1, 0), nullptr);

    // 超出范围索引
    EXPECT_EQ(mesh.findHalfEdge(0, 10), nullptr);
}

// 测试删除顶点后查找相关边
TEST_F(PlanarTriangleMeshTest, FindAfterVertexDeletion)
{
    // 删除顶点1，除其他顶点外的全部元素被删除
    mesh.deleteVertex(1);

    EXPECT_FALSE(mesh.vertices[1].isActive);

    for (int i = 0; i < mesh.halfEdges.size(); i++)
    {
        EXPECT_FALSE(mesh.halfEdges[i].isActive);
    }

    for (int i = 0; i < mesh.faces.size(); i++)
    {
        EXPECT_FALSE(mesh.faces[i].isActive);
    }

    EXPECT_EQ(mesh.edgeMap.size(), 0);
    EXPECT_EQ(mesh.faceMap.size(), 0);
}

TEST_F(PlanarTriangleMeshTest, VertexIncidentEdgesInitialization)
{
    // 验证每个顶点的入边集合
    for (const auto &v : mesh.vertices)
    {
        EXPECT_FALSE(v.incident_halfedges.empty());

        for (TriangleMesh::HalfEdge *he : v.incident_halfedges)
        {
            ASSERT_NE(he, nullptr);
            EXPECT_EQ(he->target->index, v.index);
        }
    }

    EXPECT_EQ(mesh.vertices[0].incident_halfedges.size(), 1);
    EXPECT_EQ(mesh.vertices[1].incident_halfedges.size(), 4);
    EXPECT_EQ(mesh.vertices[2].incident_halfedges.size(), 1);
    EXPECT_EQ(mesh.vertices[3].incident_halfedges.size(), 2);
    EXPECT_EQ(mesh.vertices[4].incident_halfedges.size(), 2);
    EXPECT_EQ(mesh.vertices[5].incident_halfedges.size(), 2);

    // 验证边无重复记录
    TriangleMesh::Vertex &v0 = mesh.vertices[0];
    std::unordered_set<TriangleMesh::HalfEdge *> unique_edges(v0.incident_halfedges.begin(),
                                                              v0.incident_halfedges.end());
    EXPECT_EQ(unique_edges.size(), v0.incident_halfedges.size());
}

TEST_F(PlanarTriangleMeshTest, EdgeFlipIncidentUpdate)
{
    // 初始边 0->1
    TriangleMesh::HalfEdge *he = mesh.findHalfEdge(1, 3);
    TriangleMesh::Vertex &v0 = mesh.vertices[1];
    TriangleMesh::Vertex &v1 = mesh.vertices[3];

    // 记录原始出边数量
    const size_t v0_initial = v0.incident_halfedges.size();
    const size_t v1_initial = v1.incident_halfedges.size();

    // 执行边翻转
    mesh.flipEdge(1, 3);

    // 验证顶点出边集合更新
    EXPECT_EQ(v0.incident_halfedges.size(), v0_initial - 1); // 减少一条出边
    EXPECT_EQ(v1.incident_halfedges.size(), v1_initial - 1);

    // 新顶点应获得入边
    TriangleMesh::Vertex &v2 = mesh.vertices[0];
    TriangleMesh::Vertex &v3 = mesh.vertices[4];
    EXPECT_EQ(v2.incident_halfedges.size(), 1 + 1); // 新增一条
    EXPECT_EQ(v3.incident_halfedges.size(), 2 + 1);
}

TEST_F(PlanarTriangleMeshTest, VertexOutgoingEdgesInitialization)
{
    // 验证每个顶点的出边集合
    for (const auto &v : mesh.vertices)
    {
        EXPECT_FALSE(v.outgoing_halfedges.empty());

        for (TriangleMesh::HalfEdge *he : v.outgoing_halfedges)
        {
            ASSERT_NE(he, nullptr);
            EXPECT_EQ(he->next->next->target->index, v.index);
        }
    }

    EXPECT_EQ(mesh.vertices[0].outgoing_halfedges.size(), 1);
    EXPECT_EQ(mesh.vertices[1].outgoing_halfedges.size(), 4);
    EXPECT_EQ(mesh.vertices[2].outgoing_halfedges.size(), 1);
    EXPECT_EQ(mesh.vertices[3].outgoing_halfedges.size(), 2);
    EXPECT_EQ(mesh.vertices[4].outgoing_halfedges.size(), 2);
    EXPECT_EQ(mesh.vertices[5].outgoing_halfedges.size(), 2);

    // 验证边无重复记录
    TriangleMesh::Vertex &v0 = mesh.vertices[0];
    std::unordered_set<TriangleMesh::HalfEdge *> unique_edges(v0.outgoing_halfedges.begin(),
                                                              v0.outgoing_halfedges.end());
    EXPECT_EQ(unique_edges.size(), v0.outgoing_halfedges.size());
}

TEST_F(PlanarTriangleMeshTest, EdgeFlipOutgoingUpdate)
{
    // 初始边 0->1
    TriangleMesh::HalfEdge *he = mesh.findHalfEdge(1, 3);
    TriangleMesh::Vertex &v0 = mesh.vertices[1];
    TriangleMesh::Vertex &v1 = mesh.vertices[3];
    TriangleMesh::Vertex &v2 = mesh.vertices[0];
    TriangleMesh::Vertex &v3 = mesh.vertices[4];

    // 记录原始出边数量
    const size_t v0_initial = v0.outgoing_halfedges.size();
    const size_t v1_initial = v1.outgoing_halfedges.size();
    const size_t v2_initial = v2.outgoing_halfedges.size();
    const size_t v3_initial = v3.outgoing_halfedges.size();

    // 执行边翻转
    mesh.flipEdge(1, 3);

    // 验证顶点出边集合更新
    EXPECT_EQ(v0.outgoing_halfedges.size(), v0_initial - 1); // 减少一条出边
    EXPECT_EQ(v1.outgoing_halfedges.size(), v1_initial - 1);

    // 新顶点应获得出边

    EXPECT_EQ(v2.outgoing_halfedges.size(), v2_initial + 1); // 新增一条
    EXPECT_EQ(v3.outgoing_halfedges.size(), v3_initial + 1);
}

TEST_F(PlanarTriangleMeshTest, OutgoingEdgesConsistency)
{
    // 验证所有出边的双向一致性
    for (const auto &v : mesh.vertices)
    {
        for (TriangleMesh::HalfEdge *he : v.outgoing_halfedges)
        {
            // 出边的twin边应指向当前顶点
            EXPECT_EQ(he->prev->target->index, v.index)
                << "Edge " << he->index << " twin mismatch at vertex " << v.index;

            // 出边应属于活跃边
            EXPECT_TRUE(he->isActive)
                << "Inactive edge in outgoing list of vertex " << v.index;
        }
    }
}

TEST_F(PlanarTriangleMeshTest, FaceMapInitialization)
{
    EXPECT_EQ(mesh.faces.size(), 4);
    EXPECT_EQ(mesh.faceMap.size(), 4 * 3);

    EXPECT_EQ(mesh.faceMap.count({0, 1, 3}), 1);
    EXPECT_EQ(mesh.faceMap.count({1, 3, 0}), 1);
    EXPECT_EQ(mesh.faceMap.count({3, 0, 1}), 1);

    EXPECT_EQ(mesh.faceMap.at({0, 1, 3}), &mesh.faces[0]);
    EXPECT_EQ(mesh.faceMap.at({1, 3, 0}), &mesh.faces[0]);
    EXPECT_EQ(mesh.faceMap.at({3, 0, 1}), &mesh.faces[0]);
}

TEST_F(PlanarTriangleMeshTest, FaceMapAfterFaceDeletion)
{
    mesh.deleteFace(0);

    EXPECT_EQ(mesh.faces.size(), 4);
    EXPECT_FALSE(mesh.faces[0].isActive);
    EXPECT_EQ(mesh.freeFaceIndices.size(), 1);
    EXPECT_EQ(mesh.faceMap.size(), 3 * 3);

    EXPECT_EQ(mesh.faceMap.count({0, 1, 3}), 0);
    EXPECT_EQ(mesh.faceMap.count({1, 3, 0}), 0);
    EXPECT_EQ(mesh.faceMap.count({3, 0, 1}), 0);

    mesh.deleteFace(2);
    EXPECT_EQ(mesh.faces.size(), 4);
    EXPECT_FALSE(mesh.faces[2].isActive);
    EXPECT_EQ(mesh.freeFaceIndices.size(), 2);
    EXPECT_EQ(mesh.faceMap.size(), 2 * 3);
}

TEST_F(PlanarTriangleMeshTest, FaceMapAfterSingleHalfEdgeDeletion)
{
    mesh.deleteHalfEdge(mesh.findHalfEdge(1, 3));

    EXPECT_EQ(mesh.faces.size(), 4);
    EXPECT_FALSE(mesh.faces[0].isActive);
    EXPECT_EQ(mesh.freeFaceIndices.size(), 1);
    EXPECT_EQ(mesh.faceMap.size(), 3 * 3);

    EXPECT_EQ(mesh.faceMap.count({0, 1, 3}), 0);
    EXPECT_EQ(mesh.faceMap.count({1, 3, 0}), 0);
    EXPECT_EQ(mesh.faceMap.count({3, 0, 1}), 0);
}

TEST_F(PlanarTriangleMeshTest, FaceMapAfterBothHalfEdgeDeletion)
{
    mesh.deleteHalfEdge(mesh.findHalfEdge(1, 3), true);

    EXPECT_EQ(mesh.faces.size(), 4);
    EXPECT_FALSE(mesh.faces[0].isActive);
    EXPECT_FALSE(mesh.faces[1].isActive);
    EXPECT_EQ(mesh.freeFaceIndices.size(), 2);
    EXPECT_EQ(mesh.faceMap.size(), 2 * 3);

    EXPECT_EQ(mesh.faceMap.count({0, 1, 3}), 0);
    EXPECT_EQ(mesh.faceMap.count({1, 3, 0}), 0);
    EXPECT_EQ(mesh.faceMap.count({3, 0, 1}), 0);
    EXPECT_EQ(mesh.faceMap.count({1, 4, 3}), 0);
    EXPECT_EQ(mesh.faceMap.count({4, 3, 1}), 0);
    EXPECT_EQ(mesh.faceMap.count({3, 1, 4}), 0);
}

// 测试边插入顶点
TEST_F(PlanarTriangleMeshTest, InsertVertex)
{

    auto origin_hf_size = mesh.halfEdges.size();
    auto origin_f_size = mesh.faces.size();

    Eigen::Vector3d v = 0.5 * verts[1] + 0.5 * verts[3];
    mesh.InsertVertexAtEdge(0.5 * verts[1] + 0.5 * verts[3], 1, 3);

    EXPECT_EQ(mesh.vertices.size(), 7);
    EXPECT_EQ(mesh.halfEdges.size(), origin_hf_size + 6);
    EXPECT_EQ(mesh.faces.size(), origin_f_size + 2);

    EXPECT_EQ(mesh.edgeMap.size(), 12);
    EXPECT_EQ(mesh.faceMap.size(), mesh.faces.size() * 3);

    // auto f0 = mesh.faceMap.at({0,5,3})
    EXPECT_EQ(mesh.faceMap.count({0, 1, 6}), 1);
    EXPECT_EQ(mesh.faceMap.count({0, 6, 3}), 1);
    EXPECT_EQ(mesh.faceMap.count({6, 1, 4}), 1);
    EXPECT_EQ(mesh.faceMap.count({6, 4, 3}), 1);
    EXPECT_EQ(mesh.faceMap.count({0, 1, 3}), 0);
    EXPECT_EQ(mesh.faceMap.count({1, 4, 3}), 0);

    EXPECT_EQ(mesh.faceMap.at({0, 1, 6})->normal, (verts[0] - v).cross(verts[1] - v).normalized());
    EXPECT_EQ(mesh.faceMap.at({0, 1, 6})->normal, mesh.faceMap.at({3, 0, 6})->normal);
}

// 测试边翻转
TEST_F(PlanarTriangleMeshTest, FlipEdge)
{
    // 初始边连接顶点0-1
    auto it = mesh.edgeMap.find({1, 3});
    ASSERT_NE(it, mesh.edgeMap.end());

    mesh.flipEdge(1, 3);

    // 验证新边连接顶点0-4
    EXPECT_NE(mesh.edgeMap.find({0, 4}), mesh.edgeMap.end());

    // 验证面结构
    it = mesh.edgeMap.find({0, 4});
    auto *he = it->second;

    auto f = he->face;
    auto v0 = he->target->position, v1 = he->next->target->position, v2 = he->next->next->target->position;
    auto norm = (v1 - v0).cross(v2 - v0).normalized();
    EXPECT_TRUE(f->normal.isApprox(norm));
}

TEST_F(PlanarTriangleMeshTest, QueryVertexAdjacentTriangles)
{
    EXPECT_EQ(mesh.GetAdjacentTriangles(0).size(), 1);
    EXPECT_EQ(mesh.GetAdjacentTriangles(1).size(), 4);
    EXPECT_EQ(mesh.GetAdjacentTriangles(2).size(), 1);
    EXPECT_EQ(mesh.GetAdjacentTriangles(3).size(), 2);
    EXPECT_EQ(mesh.GetAdjacentTriangles(4).size(), 2);
    EXPECT_EQ(mesh.GetAdjacentTriangles(5).size(), 2);
}

int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}