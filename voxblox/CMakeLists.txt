cmake_minimum_required(VERSION 3.5)
project(voxblox)

set(CMAKE_CXX_STANDARD 14)
add_definitions(-Wall -Wextra)

set(AMENT_ENABLE_TESTING 0)
set(CMAKE_MACOSX_RPATH 0)

# Load necessary ROS 2 packages
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_auto REQUIRED)
find_package(Protobuf REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(glog REQUIRED)
find_package(voxblox_proto REQUIRED)

ament_auto_find_build_dependencies()


####################
# HEADER FILES #
####################

include_directories(
  include
  ${CMAKE_CURRENT_BINARY_DIR} # Generate proto headers are here
  ${EIGEN3_INCLUDE_DIRS}
  ${VOXBLOX_PROTO_INCLUDE_DIRS}
)

####################
# SET SOURCE FILES #
####################

set(${PROJECT_NAME}_SRCS
  src/alignment/icp.cc
  src/core/block.cc
  src/core/esdf_map.cc
  src/core/tsdf_map.cc
  src/integrator/esdf_integrator.cc
  src/integrator/esdf_occ_integrator.cc
  src/integrator/integrator_utils.cc
  src/integrator/intensity_integrator.cc
  src/integrator/tsdf_integrator.cc
  src/io/mesh_ply.cc
  src/io/sdf_ply.cc
  src/mesh/marching_cubes.cc
  src/simulation/objects.cc
  src/simulation/simulation_world.cc
  src/utils/camera_model.cc
  src/utils/evaluation_utils.cc
  src/utils/layer_utils.cc
  src/utils/neighbor_tools.cc
  src/utils/protobuf_utils.cc
  src/utils/timing.cc
  src/utils/voxel_utils.cc
)

#############
# LIBRARIES #
#############

ament_auto_add_library(${PROJECT_NAME}
  ${${PROJECT_NAME}_SRCS}
)

target_link_libraries(${PROJECT_NAME}
  ${voxblox_proto_LIBRARIES}
  glog::glog
)

############
# BINARIES #
############

ament_auto_add_executable(tsdf_to_esdf
  test/tsdf_to_esdf.cc
)
target_link_libraries(tsdf_to_esdf ${PROJECT_NAME})

ament_auto_add_executable(test_load_esdf
  test/test_load_esdf.cc
)
target_link_libraries(test_load_esdf ${PROJECT_NAME})

#########
# TESTS #
#########

if(BUILD_TESTING)
    find_package(ament_cmake_gtest REQUIRED)
    # see https://github.com/tu-darmstadt-ros-pkg/eigen_checks
    find_package(eigen_checks REQUIRED)
    # Replicate the test_data target (same commands as before)
    add_custom_target(test_data)
    add_custom_command(TARGET test_data
            COMMAND rm -rf test_data
            COMMAND mkdir -p test_data
            COMMAND cp -r ${CMAKE_SOURCE_DIR}/test/test_data/* test_data/ || :
    )

    ament_add_gtest(test_approx_hash_array
            test/test_approx_hash_array.cc
    )

    if(TARGET test_approx_hash_array)
        target_link_libraries(test_approx_hash_array ${PROJECT_NAME})
    endif()

    ament_add_gtest(test_tsdf_map
            test/test_tsdf_map.cc
    )
    if(TARGET test_tsdf_map)
        target_link_libraries(test_tsdf_map ${PROJECT_NAME})
        ament_target_dependencies(test_tsdf_map eigen_checks)
    endif()

    ament_add_gtest(test_protobuf
            test/test_protobuf.cc
    )
    if(TARGET test_protobuf)
        target_link_libraries(test_protobuf ${PROJECT_NAME})
    endif()

    ament_add_gtest(test_tsdf_interpolator
            test/test_tsdf_interpolator.cc
    )
    if(TARGET test_tsdf_interpolator)
        target_link_libraries(test_tsdf_interpolator ${PROJECT_NAME})
        ament_target_dependencies(test_tsdf_interpolator eigen_checks)
    endif()

    ament_add_gtest(test_layer
            test/test_layer.cc
    )
    if(TARGET test_layer)
        target_link_libraries(test_layer ${PROJECT_NAME})
        ament_target_dependencies(test_layer eigen_checks)
    endif()

    ament_add_gtest(test_merge_integration
            test/test_merge_integration.cc
    )
    if(TARGET test_merge_integration)
        target_link_libraries(test_merge_integration ${PROJECT_NAME})
        ament_target_dependencies(test_merge_integration eigen_checks)
    endif()

    ament_add_gtest(test_layer_utils
            test/test_layer_utils.cc
    )
    if(TARGET test_layer_utils)
        target_link_libraries(test_layer_utils ${PROJECT_NAME})
        ament_target_dependencies(test_layer_utils eigen_checks)
    endif()

    ament_add_gtest(test_sdf_integrators
            test/test_sdf_integrators.cc
    )
    if(TARGET test_sdf_integrators)
        target_link_libraries(test_sdf_integrators ${PROJECT_NAME})
        ament_target_dependencies(test_sdf_integrators eigen_checks)
    endif()

    ament_add_gtest(test_bucket_queue
            test/test_bucket_queue.cc
    )
    if(TARGET test_bucket_queue)
        target_link_libraries(test_bucket_queue ${PROJECT_NAME} )
    endif()

    ament_add_gtest(test_clear_spheres
            test/test_clear_spheres.cc
    )
    if(TARGET test_clear_spheres)
        target_link_libraries(test_clear_spheres ${PROJECT_NAME} )
    endif()

endif()  # BUILD_TESTING


##########
# EXPORT #
##########

ament_auto_package()
