if(USE_PREBUILD_OPEN3D)
    if(NOT DEFINED OPEN3D_ROOT_DIR)
        message(FATAL_ERROR "No pre-build open3d is provided!")
    else()
        if(MSVC)
            message(STATUS "CMAKE_CONFIGURATION_TYPES: ${CMAKE_CONFIGURATION_TYPES}")

            if(CMAKE_BUILD_TYPE STREQUAL "Debug")
                set(Open3D_DIR "${OPEN3D_ROOT_DIR}/Debug")
                list(APPEND CMAKE_PREFIX_PATH "${Open3D_DIR}")
                message(STATUS "Open3D in: ${OPEN3D_ROOT_DIR}/Debug")
            else()
                set(Open3D_DIR "${OPEN3D_ROOT_DIR}/Release")
                list(APPEND CMAKE_PREFIX_PATH "{Open3D_DIR}")
                message(STATUS "Open3D in: ${Open3D_DIR}")
            endif()
        else()
            if(CMAKE_BUILD_TYPE STREQUAL "Debug")
                list(APPEND CMAKE_PREFIX_PATH "${OPEN3D_ROOT_DIR}/Debug")
                message(STATUS "Open3D in: ${OPEN3D_ROOT_DIR}/Debug")
            else()
                list(APPEND CMAKE_PREFIX_PATH "{OPEN3D_ROOT_DIR}/Release")
                message(STATUS "Open3D in: ${OPEN3D_ROOT_DIR}/Release")
            endif()
        endif()

        find_package(Open3D REQUIRED PATHS ${Open3D_DIR})

        set_target_properties(Open3D::Open3D PROPERTIES
            MAP_IMPORTED_CONFIG_RELWITHDEBINFO "Release"
            MAP_IMPORTED_CONFIG_MINSIZEREL "Release"
            MAP_IMPORTED_CONFIG_RELEASE "Release"
            MAP_IMPORTED_CONFIG_DEBUG "Debug"
        )
    endif()
endif()

find_package(Open3D CONFIG QUIET)

if(Open3D_FOUND)
    message(STATUS "Found Open3D v${Open3D_VERSION}.")
else()
    if(TARGET Open3D::Open3D)
        message(STATUS "Target Open3D::Open3D found.")
    else()
        option(GLIBCXX_USE_CXX11_ABI "Set -D_GLIBCXX_USE_CXX11_ABI=1" OFF)
        option(STATIC_WINDOWS_RUNTIME "Use static (MT/MTd) Windows runtime" ON)

        include(ExternalProject)
        ExternalProject_Add(

            external_open3d

            # PREFIX ${CMAKE_CURRENT_SOURCE_DIR}/../../.deps/open3d
            GIT_REPOSITORY https://github.com/intel-isl/Open3D.git
            GIT_TAG v0.18.0
            GIT_SHALLOW ON
            UPDATE_COMMAND ""

            # Check out https://github.com/intel-isl/Open3D/blob/master/CMakeLists.txt
            # For the full list of available options.
            CMAKE_ARGS
            -DCMAKE_CXX_STANDARD=${EXTERNAL_CXX_STANDARD}
            -DCMAKE_INSTALL_PREFIX=<INSTALL_DIR>
            -DCMAKE_BUILD_TYPE=${CMAKE_BUILD_TYPE}
            -DCMAKE_C_COMPILER=${CMAKE_C_COMPILER}
            -DCMAKE_CXX_COMPILER=${CMAKE_CXX_COMPILER}
            -DGLIBCXX_USE_CXX11_ABI=${GLIBCXX_USE_CXX11_ABI}
            -DSTATIC_WINDOWS_RUNTIME=${STATIC_WINDOWS_RUNTIME}
            -DBUILD_WEBRTC=OFF
            -DBUILD_SHARED_LIBS=ON
            -DBUILD_PYTHON_MODULE=OFF
            -DBUILD_EXAMPLES=OFF
            -DUSE_SYSTEM_EIGEN=ON
        )

        # Simulate importing Open3D::Open3D
        ExternalProject_Get_Property(external_open3d INSTALL_DIR)
        add_library(Open3DHelper INTERFACE)
        add_dependencies(Open3DHelper external_open3d)
        target_compile_features(Open3DHelper INTERFACE cxx_std_14)
        target_compile_definitions(Open3DHelper INTERFACE _GLIBCXX_USE_CXX11_ABI=$<BOOL:${GLIBCXX_USE_CXX11_ABI}>)
        target_include_directories(Open3DHelper INTERFACE "${INSTALL_DIR}/include" "${INSTALL_DIR}/include/open3d/3rdparty")
        target_link_directories(Open3DHelper INTERFACE "${INSTALL_DIR}/lib")
        target_link_libraries(Open3DHelper INTERFACE Open3D)
        add_library(Open3D::Open3D ALIAS Open3DHelper)

        set(Open3D_DIR ${INSTALL_DIR} CACHE PATH "Open3D installation directory" FORCE)
    endif()
endif()
