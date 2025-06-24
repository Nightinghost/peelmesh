# PeelMesh: Efficient Interactive Segmentation via Geodesic-Driven Dynamic Topological Updates

[![License](https://img.shields.io/badge/License-MIT-blue.svg)](https://opensource.org/licenses/MIT)
![MSVC](https://byob.yarr.is/Nightinghost/peelmesh/windows-msvc)
![Clang](https://byob.yarr.is/Nightinghost/peelmesh/windows-clang)
![GCC](https://byob.yarr.is/Nightinghost/peelmesh/ubuntu-gcc)

PeelMesh is a C++ library with optional Python bindings for flexible and interactive 3D mesh segmentation. It supports manually specified landmarks, dynamic geodesic path embedding, and mesh topology editing. This repository accompanies the [CGI 2025](https://www.cgsociety.org/) paper: "PeelMesh: Efficient Interactive Segmentation via Geodesic-Driven Dynamic Topological Updates"

## 🧩 Features

- Custom landmark-based segmentation
- Halfedge-based triangle mesh data structure with dynamic KD-tree integration
- Geometric attributes query (e.g. curvatures, surface area, geodesic distance)
- Optional Python bindings via pybind11

## 📂 Project Structure

```bash
peelmesh
├── include/peelmesh/     # Public C++ headers
├── src/peelmesh/         # C++ source files
├── src/bindings/         # pybind11 bindings
├── tests/                # GoogleTest test suite
├── examples/             # Example C++ demos and PSD experiment
├── models/               # Some meshes
├── cmake/                # CMake config & export files
├── external/             # External dependencies via FetchContent
└── CMakeLists.txt        # CMake project
```

## 🛠️ Build and Install Instructions (C++)

### 🔧 Prerequisites
- CMake $\ge$ 3.21
- C++ compiler with C++20 support (e.g. MSVC 2022, Clang for MSVC 2022, GCC 13)
- Python 3.x (optional, for bindings)

```bash
# Clone repository
git clone https://github.com/Nightinghost/peelmesh
cd peelmesh

# Configure
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=<path-to-install>

# Build
cmake --build . --target install
```

## 🧪 Python Bindings
To build the Python bindings, you need to have a working Python installation (conda, venv, etc.)

```bash
conda activate myenv  # activate your Python environment

mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release -DPEELMESH_BUILD_PYTHON_BINDINGS=ON

cmake --build .
```
The resulting Python module will be named `peelmesh` and located in the build/lib/Release (or build/lib/) folder, e.g. `build/lib/Release/peelmesh.cp313-win_amd64.pyd` on Windows.


## 📚 Usage (C++)
To use the library in your project, you can add the following to your `CMakeLists.txt` file:

```cmake
find_package(peelmesh REQUIRED)
message(STATUS "PeelMesh version: ${peelmesh_VERSION}")

if(MSVC)
    add_compile_definitions(_USE_MATH_DEFINES)
endif()

target_link_libraries(your_target PRIVATE peelmesh::peelmesh)
```

### Example Usage
```cpp
#include <peelmesh/pipeline.hpp>

int main()
{
    // Mesh data
    std::vector<Eigen::Vector3d> vertices = {{0, 0, 0}, {1, 0, 0}, {0, 1, 0}, {1, 1, 0}};
    std::vector<Eigen::Vector3i> triangles = {{0, 1, 2}, {1, 3, 2}};

    auto mesh = std::make_shared<peelmesh::TriangleMesh>(vertices, triangles);

    // Halfedge information of the mesh
    mesh->PrintMesh();

    // Create a pipeline object
    peelmesh::PeelMeshPipeline pipe(mesh);

    // Compute geodesic paths from vertices[start] to vertices[end]
    pipe.AddGeodesicPath(0, 1);
    pipe.AddGeodesicPath(1, 3);
    pipe.AddGeodesicPath(3, 0);

    // Extract the segment bounded by the geodesics based on the given vertices sequence
    auto segment = pipe.PeelOffMesh({0, 1, 3, 0});

    // Get the vertices and triangles of the segmented new mesh
    const auto &[verts, tris] = segment->getMeshData();

    return 0;
}
```

## Usage (Python)
To use the library in Python, you should copy the `peelmesh.*.pyd` file (e.g. peelmesh.cp310-win_amd64.pyd) to the Python project directory. Then, you can import the `peelmesh` module and use the `TriangleMesh` and `PeelMeshPipeline` classes as shown in the example below:

```python
import numpy as np
import peelmesh as pm

# Mesh data
vertices = np.array([[0, 0, 0], [1, 0, 0], [0, 1, 0], [1, 1, 0]])
triangles = np.asarray([[0, 1, 2], [1, 3, 2]])

# Create a pipeline object
pipe = pm.PeelMeshPipeline(vertices, triangles)

# Compute geodesic paths from vertices[start] to vertices[end]
pipe.add_geodesic_path(0, 1)
pipe.add_geodesic_path(1, 3)
pipe.add_geodesic_path(3, 0)

# Extract the segment bounded by the geodesics based on the given vertices sequence
segment = pipe.peeloff_mesh([0, 1, 3, 0])

# Get the vertices and triangles of the segmented new mesh
verts, tris = segment.get_mesh_data()
```

For other information, try:
```python
import peelmesh as pm
help(pm)
```

## 🖱️ Interactive Segmentation Demo (C++)

This project includes an interactive GUI demo for mesh segmentation. The tool allows you to manually define landmarks and extract a sub-mesh from a closed boundary on the surface.

![](./imgs/interactive_demo.gif)

### ✨ Features
- Load and visualize mesh models (File > Open)

- Export all segmented sub-meshes (Export > Export Segments)

- Interactive landmark selection

### 🕹️ Controls
| Action                      | Shortcut                |
| --------------------------- | ----------------------- |
| **Select feature point**    | `Ctrl + Left Click`     |
| **Undo last feature point** | `Ctrl + Right Click`    |
| **Close boundary**          | `Right Click` on canvas |
| **Execute segmentation**    | `Enter`                 |
| **Reset camera**            | `R` key                 |

After pressing <kbd>Enter</kbd>, the tool computes a segmentation based on the geodesic path between selected landmarks, and extracts the corresponding sub-meshes for further processing or export.





## Citation
If you use this code in your research, please cite our paper:

```java
@inproceedings{PeelMesh2025,
  title={PeelMesh: Efficient Interactive Segmentation via Geodesic-Driven Dynamic Topological Updates},
  author={Junjie Yin, Hao Zhou, Zixi Huang, Meie Fang, Ping Li, and Weiyin Ma},
  booktitle={Computer Graphics International (CGI)},
  year={2025}
}
```
