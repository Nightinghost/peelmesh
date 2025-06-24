#include <peelmesh/meshio.hpp>

#include <fstream>
#include <iostream>

namespace peelmesh
{
    std::tuple<std::vector<Eigen::Vector3d>, std::vector<Eigen::Vector3i>> ReadObj(const std::string &filename)
    {
        std::vector<Eigen::Vector3d> vertices;
        std::vector<Eigen::Vector3i> triangles;

        std::ifstream file(filename);
        if (!file.is_open())
        {
            std::cerr << "Failed to open file: " << filename << std::endl;
            return {vertices, triangles};
        }

        std::string line;
        while (std::getline(file, line))
        {
            if (line.empty())
            {
                continue;
            }

            std::istringstream iss(line);
            std::string type;
            iss >> type;

            if (type == "v")
            {
                Eigen::Vector3d v;
                iss >> v[0] >> v[1] >> v[2];
                vertices.push_back(v);
            }
            else if (type == "f")
            {
                Eigen::Vector3i f;
                std::string str;
                for (int i = 0; i < 3; i++)
                {
                    iss >> str;
                    if (str.find('/') != std::string::npos)
                        f[i] = std::stoi(str.substr(0, str.find('/'))) - 1;
                    else
                        f[i] = std::stoi(str) - 1;
                }
                triangles.push_back(f);
            }
        }

        return {vertices, triangles};
    }
} // namespace peelmesh