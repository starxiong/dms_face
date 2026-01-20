# build Exe
message(STATUS "face_poseNode find dir:${PROJECT_SOURCE_DIR}/src")
add_executable(${PROJECT_NAME} src/main.cc ${srcs})
# target_link_libraries(${PROJECT_NAME}Node ${PROJECT_NAME}_${GNSSPARSE_VERSION} )
target_link_libraries(${PROJECT_NAME} ${face_pose_link_libraries} ${face_pose_third_libraries} yaml-cpp ceres)