find_package(Open3D REQUIRED)

add_library(visualization)
target_sources(visualization
    PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/peelmesh/utility.hpp
    ${CMAKE_CURRENT_SOURCE_DIR}/peelmesh/utility.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/peelmesh/visualization.hpp
    ${CMAKE_CURRENT_SOURCE_DIR}/peelmesh/visualization.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/peelmesh/window.hpp
    ${CMAKE_CURRENT_SOURCE_DIR}/peelmesh/portable-file-dialogs.h
)

target_link_libraries(visualization PUBLIC peelmesh Open3D::Open3D geometry-central)
target_include_directories(visualization PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

add_executable(psb_experiment psb_experiment.cpp)
target_link_libraries(psb_experiment PRIVATE visualization)

add_executable(demo demo.cpp)
target_link_libraries(demo PRIVATE visualization)

add_custom_command(TARGET demo POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_directory_if_different
    ${Open3D_DIR}/../bin/resources
    $<TARGET_FILE_DIR:demo>/resources

    COMMAND ${CMAKE_COMMAND} -E copy_directory_if_different
    ${CMAKE_SOURCE_DIR}/models
    $<TARGET_FILE_DIR:demo>/models

    COMMAND ${CMAKE_COMMAND} -E copy_if_different
    ${Open3D_DIR}/../bin/Open3D.dll
    $<TARGET_FILE_DIR:demo>/Open3D.dll
)

if(${Open3D_VERSION} VERSION_GREATER_EQUAL "0.19.0")
    add_custom_command(TARGET demo POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
        ${Open3D_DIR}/../bin/tbb12_debug.dll
        $<TARGET_FILE_DIR:demo>/tbb12_debug.dll
    )
endif()