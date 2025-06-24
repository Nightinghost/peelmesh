include(GoogleTest)

add_executable(watertight_mesh_test WatertightMesh.cpp)
target_link_libraries(watertight_mesh_test PRIVATE peelmesh gtest_main geometry-central)

gtest_discover_tests(watertight_mesh_test)

add_executable(boundary_mesh_test BoundaryMesh.cpp)
target_link_libraries(boundary_mesh_test PRIVATE peelmesh gtest_main geometry-central)

gtest_discover_tests(boundary_mesh_test)

add_executable(kdtree_test kdtree.cpp)
target_link_libraries(kdtree_test PRIVATE peelmesh gtest_main geometry-central)

gtest_discover_tests(kdtree_test)