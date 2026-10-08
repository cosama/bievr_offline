cmake_minimum_required(VERSION 3.22)
project(bievr_offline LANGUAGES C CXX)

# Upstream requires C++20 (concepts).
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
add_compile_options(-g)

find_package(Eigen3 REQUIRED)
find_package(Ceres REQUIRED)
find_package(TBB REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(GTSAM CONFIG REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io filters registration)
set(PYBIND11_FINDPYTHON ON)
find_package(pybind11 CONFIG REQUIRED)

# Keep upstream pristine.
get_filename_component(BIEVR_FRAMEWORK_DIR "${CMAKE_CURRENT_SOURCE_DIR}/.." ABSOLUTE)
set(UPSTREAM_BIEVR "${BIEVR_FRAMEWORK_DIR}/upstream/BIEVR-LIO-SLAM")
if(NOT EXISTS "${UPSTREAM_BIEVR}/BIEVR/src/pipeline.cpp")
  message(FATAL_ERROR
    "Pinned upstream BIEVR-LIO-SLAM source is missing at ${UPSTREAM_BIEVR}. "
    "Run: git submodule update --init --recursive upstream/BIEVR-LIO-SLAM")
endif()

# Patches apply to a staged copy; the upstream checkout is never modified.
set(BIEVR_STAGE "${CMAKE_CURRENT_BINARY_DIR}/upstream_staged")
file(REMOVE_RECURSE "${BIEVR_STAGE}")
file(MAKE_DIRECTORY "${BIEVR_STAGE}")
file(COPY "${UPSTREAM_BIEVR}/BIEVR/include" "${UPSTREAM_BIEVR}/BIEVR/src"
     DESTINATION "${BIEVR_STAGE}")
file(COPY "${UPSTREAM_BIEVR}/modules"
     DESTINATION "${BIEVR_STAGE}")

find_program(PATCH_EXECUTABLE patch REQUIRED)
file(GLOB BIEVR_INTEGRATION_PATCHES
     "${CMAKE_CURRENT_SOURCE_DIR}/patches/integration/*.patch")
list(SORT BIEVR_INTEGRATION_PATCHES)
list(LENGTH BIEVR_INTEGRATION_PATCHES BIEVR_PATCH_COUNT)
foreach(patch_file IN LISTS BIEVR_INTEGRATION_PATCHES)
  execute_process(
    COMMAND "${PATCH_EXECUTABLE}" -F0 --silent --no-backup-if-mismatch -p0 -i "${patch_file}"
    WORKING_DIRECTORY "${BIEVR_STAGE}"
    RESULT_VARIABLE patch_result
    ERROR_VARIABLE patch_error)
  if(NOT patch_result EQUAL 0)
    message(FATAL_ERROR
      "Patch did not apply to pinned upstream: ${patch_file}\n${patch_error}")
  endif()
  set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS "${patch_file}")
endforeach()
message(STATUS "BIEVR: applied ${BIEVR_PATCH_COUNT} patches to staged upstream")

# Upstream's estimator library and SLAM backend; the ROS interfaces are
# replaced by cpp/bindings.cpp.
pybind11_add_module(_core NO_EXTRAS
  cpp/bindings.cpp
  "${BIEVR_STAGE}/src/bievr_map.cpp"
  "${BIEVR_STAGE}/src/common.cpp"
  "${BIEVR_STAGE}/src/pipeline.cpp"
  "${BIEVR_STAGE}/src/imu_integrator.cpp"
  "${BIEVR_STAGE}/src/inertial_factor.cpp"
  "${BIEVR_STAGE}/src/ls_optimizer.cpp"
  "${BIEVR_STAGE}/src/map_accumulator.cpp"
  "${BIEVR_STAGE}/src/preprocess.cpp"
  "${BIEVR_STAGE}/src/prior_factor.cpp"
  "${BIEVR_STAGE}/src/synchronizer.cpp"
  "${BIEVR_STAGE}/src/timing.cpp"
  "${BIEVR_STAGE}/src/undistort.cpp"
  "${BIEVR_STAGE}/src/utils.cpp"
  "${BIEVR_STAGE}/modules/scancontext/src/scan_context.cpp"
  "${BIEVR_STAGE}/modules/map_io/src/cloud_utils.cpp"
  "${BIEVR_STAGE}/modules/map_io/src/prior_map.cpp"
  "${BIEVR_STAGE}/modules/map_io/src/tile_map.cpp"
  "${BIEVR_STAGE}/modules/pgo/src/loop_closer.cpp"
)

target_include_directories(_core PRIVATE
  "${BIEVR_STAGE}/include"
  "${BIEVR_STAGE}/modules/scancontext/include"
  "${BIEVR_STAGE}/modules/map_io/include"
  "${BIEVR_STAGE}/modules/pgo/include"
  "${GTSAM_INCLUDE_DIR}"
  ${PCL_INCLUDE_DIRS}
)
# Same compile mode as upstream's library target (log++ in LPP mode, -O3).
target_compile_definitions(_core PRIVATE MODE_LPP)
target_compile_options(_core PRIVATE -O3 -fconcepts)
target_link_libraries(_core PRIVATE
  Eigen3::Eigen
  Ceres::ceres
  TBB::tbb
  yaml-cpp::yaml-cpp
  gtsam
  ${PCL_LIBRARIES}
)

install(TARGETS _core LIBRARY DESTINATION bievr)
