file(GLOB pose_estimation_srcs src/pose_estimation.cc)
file(GLOB visualization_srcs src/visualization.cc)
set(srcs ${visualization_srcs} ${pose_estimation_srcs})

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)