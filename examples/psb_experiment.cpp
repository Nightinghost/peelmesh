#include <iostream>
#include <peelmesh/geometry.hpp>
#include <peelmesh/meshio.hpp>
#include <peelmesh/pipeline.hpp>
#include <peelmesh/utility.hpp>

#include <open3d/Open3D.h>

#include <fstream>
#include <iterator>
using namespace peelmesh;

std::vector<int> read_landmark(std::string filename, std::shared_ptr<open3d::geometry::TriangleMesh> mesh)
{
    open3d::geometry::KDTreeFlann kdtree(*mesh);

    std::ifstream file(filename);
    std::vector<int> landmarks;

    std::string line;
    while (std::getline(file, line))
    {
        std::stringstream ss(line);
        std::string c;
        double x, y, z;
        ss >> c >> x >> y >> z;

        std::vector<int> indices;
        std::vector<double> dists2;
        int k = kdtree.SearchKNN(Eigen::Vector3d{x, y, z}, 1, indices, dists2);
        landmarks.push_back(indices[0]);
    }

    return landmarks;
}

bool check_manifold(std::shared_ptr<open3d::geometry::TriangleMesh> mesh)
{
    return mesh->IsVertexManifold() && mesh->IsEdgeManifold();
}

void visualize_mesh_and_landmarks(std::shared_ptr<open3d::geometry::TriangleMesh> mesh, std::vector<int> landmarks)
{
    auto pcd = std::make_shared<open3d::geometry::PointCloud>();
    for (auto i : landmarks)
    {
        pcd->points_.push_back(mesh->vertices_[i]);
    }
    pcd->PaintUniformColor({1, 0, 0});

    open3d::visualization::DrawGeometries({mesh, pcd});
}

struct BSDMeshInfo
{
    std::string filename;

    std::vector<int> landmarks;
    std::vector<std::array<int, 2>> connections;

    std::vector<Eigen::Vector3d> vertices;
    std::vector<Eigen::Vector3i> triangles;

    BSDMeshInfo(std::string filename, std::vector<int> landmarks, std::vector<std::array<int, 2>> connections)
        : filename(filename), landmarks(landmarks), connections(connections)
    {
        static std::filesystem::path path = "./models/PSB";

        auto geo = open3d::io::CreateMeshFromFile((path / filename).generic_string());
        geo->RemoveDuplicatedTriangles();
        geo->RemoveDuplicatedVertices();
        geo->Scale(20, geo->GetCenter());
        vertices = geo->vertices_;
        triangles = geo->triangles_;
    }
};

void PSB_Dataset_Test()
{
    std::vector<BSDMeshInfo> dataset;
    // Human
    dataset.push_back({"2.off",
                       {228, 1434, 2832, 7256, 8390, 8010, 6008, 2094, 2154, 2785, 5423, 4896, 184, 4965, 4358, 1523, 9656, 5172, 1834, 3250, 2971, 10003, 2024, 1787, 754, 518, 7890, 1753, 6898, 2252, 6216, 5054, 5990, 5960, 4977, 5882, 1163, 1177, 4604, 2163, 4806, 5057, 2693, 9640, 4979},
                       {{0, 1}, {1, 2}, {2, 0}, {3, 4}, {4, 5}, {5, 3}, {6, 7}, {7, 8}, {8, 6}, {9, 10}, {10, 11}, {11, 9}, {12, 13}, {13, 14}, {14, 12}, {15, 16}, {16, 17}, {17, 15}, {18, 19}, {19, 20}, {20, 18}, {21, 22}, {22, 23}, {23, 21}, {24, 25}, {25, 26}, {26, 24}, {27, 28}, {28, 29}, {29, 27}, {30, 31}, {31, 32}, {32, 30}, {33, 34}, {34, 35}, {35, 33}, {36, 37}, {37, 38}, {38, 36}, {39, 40}, {40, 41}, {41, 39}, {42, 43}, {43, 44}, {44, 42}}});
    // Cup
    dataset.push_back({"30.off",
                       {5764, 8799, 7569, 6109, 8852, 7880, 4528, 8226, 8906},
                       {{0, 1}, {1, 2}, {2, 0}, {3, 4}, {4, 5}, {5, 3}, {6, 7}, {7, 8}, {8, 6}}});

    // Glass
    dataset.push_back({"41.off",
                       {2328, 1436, 3177, 1258, 2738, 1254, 2173, 1840, 3813, 930, 1259, 2734, 1872, 1870, 3171},
                       {{0, 1}, {1, 2}, {2, 0}, {3, 4}, {4, 5}, {5, 3}, {6, 7}, {7, 8}, {8, 6}, {9, 10}, {10, 11}, {11, 9}, {12, 13}, {13, 14}, {14, 12}}});

    // Airplane
    dataset.push_back({"61.off",
                       {2223, 5005, 4976, 1829, 1376, 3554, 3784, 1576, 3260, 3612, 4200, 4436, 3742, 4121, 4532, 3237, 3228, 3174},
                       {{0, 1}, {1, 2}, {2, 0}, {3, 4}, {4, 5}, {5, 3}, {6, 7}, {7, 8}, {8, 6}, {9, 10}, {10, 11}, {11, 9}, {12, 13}, {13, 14}, {14, 12}, {15, 16}, {16, 17}, {17, 15}}});
    // Ant
    dataset.push_back({"81.off",
                       {1616, 3033, 3045, 2596, 2610, 3772, 635, 2192, 1578, 2182, 2954, 2993, 2164, 2476, 3193, 1068, 1055, 2133, 2177, 2985, 3209, 1548, 2892, 2810, 521, 1534, 1793, 795, 1285, 2423},
                       {{0, 1}, {1, 2}, {2, 0}, {3, 4}, {4, 5}, {5, 3}, {6, 7}, {7, 8}, {8, 6}, {9, 10}, {10, 11}, {11, 9}, {12, 13}, {13, 14}, {14, 12}, {15, 16}, {16, 17}, {17, 15}, {18, 19}, {19, 20}, {20, 18}, {21, 22}, {22, 23}, {23, 21}, {24, 25}, {25, 26}, {26, 24}, {27, 28}, {28, 29}, {29, 27}}});
    // Chair
    dataset.push_back({"101.off",
                       {7751, 6535, 7750, 7474, 6586, 7829, 6968, 8331, 6971, 6376, 7917, 7520, 239, 1724, 2027, 1328, 244, 2294, 413, 1947, 1283, 943, 680, 1562},
                       {{0, 1}, {1, 2}, {2, 0}, {3, 4}, {4, 5}, {5, 3}, {6, 7}, {7, 8}, {8, 6}, {9, 10}, {10, 11}, {11, 9}, {12, 13}, {13, 14}, {14, 12}, {15, 16}, {16, 17}, {17, 15}, {18, 19}, {19, 20}, {20, 18}, {21, 22}, {22, 23}, {23, 21}}});
    // Octopus
    dataset.push_back({"122.off",
                       {5292, 5096, 2596, 4651, 5466, 5046, 3225, 5442, 5456, 5468, 5448, 3923, 5299, 2560, 5485, 4245, 5514, 4714, 5344, 4751, 2149, 5523, 4744, 4244},
                       {{0, 1}, {1, 2}, {2, 0}, {3, 4}, {4, 5}, {5, 3}, {6, 7}, {7, 8}, {8, 6}, {9, 10}, {10, 11}, {11, 9}, {12, 13}, {13, 14}, {14, 12}, {15, 16}, {16, 17}, {17, 15}, {18, 19}, {19, 20}, {20, 18}, {21, 22}, {22, 23}, {23, 21}}});

    // Table
    dataset.push_back({"141.off",
                       {3932, 3982, 4296, 2954, 7091, 6657, 2892, 4265, 3904, 2935, 2978, 6728},
                       {{0, 1}, {1, 2}, {2, 0}, {3, 4}, {4, 5}, {5, 3}, {6, 7}, {7, 8}, {8, 6}, {9, 10}, {10, 11}, {11, 9}}});

    // Teddy
    dataset.push_back({"161.off",
                       {5800, 8300, 9477, 6281, 8352, 8314, 5120, 5876, 10932, 5216, 7641, 10301, 9004, 5888, 5566, 9171, 5003, 5635, 10804, 7752, 4580},
                       {{0, 1}, {1, 2}, {2, 0}, {3, 4}, {4, 5}, {5, 3}, {6, 7}, {7, 8}, {8, 6}, {9, 10}, {10, 11}, {11, 9}, {12, 13}, {13, 14}, {14, 12}, {15, 16}, {16, 17}, {17, 15}, {18, 19}, {19, 20}, {20, 18}}});

    // Hand
    dataset.push_back({"183.off",
                       {8675, 4710, 1274, 7190, 8005, 10260, 7624, 7625, 10792, 6735, 6314, 9506, 4832, 4360, 7596},
                       {{0, 1}, {1, 2}, {2, 0}, {3, 4}, {4, 5}, {5, 3}, {6, 7}, {7, 8}, {8, 6}, {9, 10}, {10, 11}, {11, 9}, {12, 13}, {13, 14}, {14, 12}}});

    // Plier
    dataset.push_back({"201.off",
                       {3477, 3498, 3563, 3301, 3284, 3438, 2536, 2905, 2787, 2664, 2508, 2619},
                       {{0, 1}, {1, 2}, {2, 0}, {3, 4}, {4, 5}, {5, 3}, {6, 7}, {7, 8}, {8, 6}, {9, 10}, {10, 11}, {11, 9}}});

    // Fish
    dataset.push_back({"221.off",
                       {4462, 2536, 5790, 5129, 5116, 4011, 4807, 5314, 5307, 520, 492, 456, 6170, 6240, 6550},
                       {{0, 1}, {1, 2}, {2, 0}, {3, 4}, {4, 5}, {5, 3}, {6, 7}, {7, 8}, {8, 6}, {9, 10}, {10, 11}, {11, 9}, {12, 13}, {13, 14}, {14, 12}}});

    // Bird
    dataset.push_back({"241.off",
                       {2359, 1165, 1862, 873, 977, 860, 2630, 2643, 2590, 2078, 1706, 1181},
                       {{0, 1}, {1, 2}, {2, 0}, {3, 4}, {4, 5}, {5, 3}, {6, 7}, {7, 8}, {8, 6}, {9, 10}, {10, 11}, {11, 9}}});

    // Armadillo
    dataset.push_back({"281.off",
                       {19637, 20847, 8943, 22053, 9063, 23074, 3472, 15921, 14973, 7640, 23356, 24499, 5290, 18762, 24203, 11473, 7216, 13383},
                       {{0, 1}, {1, 2}, {2, 0}, {3, 4}, {4, 5}, {5, 3}, {6, 7}, {7, 8}, {8, 6}, {9, 10}, {10, 11}, {11, 9}, {12, 13}, {13, 14}, {14, 12}, {15, 16}, {16, 17}, {17, 15}}});

    // Bust
    dataset.push_back({"303.off",
                       {14593, 8865, 14747, 3140, 15365, 3121, 2086, 12906, 5450, 5428, 14637, 12906, 4771, 3121, 13687, 14196, 679, 1648, 13358, 12964, 639, 13412, 14636, 12415},
                       {{0, 1}, {1, 2}, {2, 3}, {3, 4}, {4, 5}, {5, 6}, {6, 7}, {7, 8}, {8, 9}, {9, 10}, {10, 0}, {11, 12}, {12, 13}, {13, 11}, {14, 15}, {15, 16}, {16, 14}, {17, 18}, {18, 19}, {19, 17}, {20, 21}, {21, 22}, {22, 23}, {23, 20}}});
    // Mech
    dataset.push_back({"324.off",
                       {11345, 5490, 10425, 5248, 8810, 197},
                       {{0, 1}, {1, 2}, {2, 0}, {3, 4}, {4, 5}, {5, 3}}});

    // Bearing
    dataset.push_back({"341.off",
                       {80, 77, 98, 95, 92, 89, 86, 83, 46, 30, 39, 259, 268, 253, 202, 208, 217, 169, 153, 160},
                       {{0, 1}, {1, 2}, {2, 3}, {3, 4}, {4, 5}, {5, 6}, {6, 7}, {7, 0}, {8, 9}, {9, 10}, {10, 8}, {11, 12}, {12, 13}, {13, 11}, {14, 15}, {15, 16}, {16, 14}, {17, 18}, {18, 19}, {19, 17}}});

    // Vase
    dataset.push_back({"373.off",
                       {6427, 10444, 8053, 7932, 7866, 7814, 8666, 8922, 7271, 6147, 9929, 10266, 9077, 8839, 7205, 5859, 4164, 2558, 1305},
                       {{0, 1}, {1, 2}, {2, 0}, {3, 4}, {4, 5}, {5, 3}, {6, 7}, {7, 8}, {8, 6}, {9, 10}, {10, 11}, {11, 12}, {12, 13}, {13, 14}, {14, 15}, {15, 9}, {16, 17}, {17, 18}, {18, 16}}});

    // FourLeg
    dataset.push_back({"384.off",
                       {458, 939, 393, 2482, 2924, 122, 1935, 948, 694, 195, 4022, 828, 4322, 4021, 4100, 286, 3337, 883, 2764, 3269, 3800, 1622, 527, 599},
                       {{0, 1}, {1, 2}, {2, 0}, {3, 4}, {4, 5}, {5, 3}, {6, 7}, {7, 8}, {8, 6}, {9, 10}, {10, 11}, {11, 9}, {12, 13}, {13, 14}, {14, 12}, {15, 16}, {16, 17}, {17, 15}, {18, 19}, {19, 20}, {20, 18}, {21, 22}, {22, 23}, {23, 21}}});

    for (auto &data : dataset)
    {
        std::cout << "------------------------------------------------------\n";
        std::cout << "Vertices: " << data.vertices.size() << std::endl;
        std::cout << "Triangles: " << data.triangles.size() << std::endl;
        std::cout << "Landmarks: " << data.landmarks.size() << std::endl;
        std::cout << "Geodesics: " << data.connections.size() << std::endl;

        auto mesh = std::make_shared<TriangleMesh>(data.vertices, data.triangles);
        peelmesh::PeelMeshPipeline pipe(mesh);

        auto start_time = std::chrono::system_clock::now();
        for (auto &pair : data.connections)
        {
            pipe.AddGeodesicPath(data.landmarks[pair[0]], data.landmarks[pair[1]]);
            // pipe.AddShortestPath(data.landmarks[pair[0]], data.landmarks[pair[1]]);
        }
        auto end_time = std::chrono::system_clock::now();
        auto elapsed_seconds = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count();
        std::cout << "Total Geodesic Insertion Time: " << elapsed_seconds << "ms" << std::endl;
        std::cout << "Average Geodesic Insertion Time: " << (double)elapsed_seconds / data.connections.size() << "ms" << std::endl;

        auto total_time = elapsed_seconds;

        start_time = std::chrono::system_clock::now();
        auto segments = pipe.AutoSegmentation();
        end_time = std::chrono::system_clock::now();
        elapsed_seconds = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count();
        std::cout << "Total Auto-Segmentation Time: " << elapsed_seconds << "ms" << std::endl;
        std::cout << "Average Auto-Segmentation Time: " << (double)elapsed_seconds / segments.size() << "ms" << std::endl;

        std::cout << "Total Pipeline Time: " << total_time + elapsed_seconds << "ms" << std::endl;

        visualize_segments(segments);

        std::cout << "=================\n";

        auto mesh2 = std::make_shared<TriangleMesh>(data.vertices, data.triangles);
        peelmesh::PeelMeshPipeline pipe2(mesh2);

        start_time = std::chrono::system_clock::now();
        for (auto &pair : data.connections)
        {
            pipe2.AddShortestPath(data.landmarks[pair[0]], data.landmarks[pair[1]]);
        }
        end_time = std::chrono::system_clock::now();
        elapsed_seconds = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count();
        std::cout << "Total Shortest Insertion Time: " << elapsed_seconds << "ms" << std::endl;
        std::cout << "Average Shortest Insertion Time: " << (double)elapsed_seconds / data.connections.size() << "ms" << std::endl;

        total_time = elapsed_seconds;

        start_time = std::chrono::system_clock::now();
        segments = pipe2.AutoSegmentation();
        end_time = std::chrono::system_clock::now();
        elapsed_seconds = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count();
        std::cout << "Total Shortest Auto-Segmentation Time: " << elapsed_seconds << "ms" << std::endl;
        std::cout << "Average Shortest Auto-Segmentation Time: " << (double)elapsed_seconds / segments.size() << "ms" << std::endl;

        std::cout << "Total Shortest Pipeline Time: " << total_time + elapsed_seconds << "ms" << std::endl;
        visualize_segments(segments);
    }
}

int main()
{
    PSB_Dataset_Test();

    return 0;
}