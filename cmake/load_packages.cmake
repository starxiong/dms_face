# INCLUDE(${CMAKE_CURRENT_LIST_DIR}/../extern/cmake/load_packages.cmake)
# INCLUDE(${CMAKE_CURRENT_LIST_DIR}/../extern/cmake/load_packages.cmake)

set(face_pose_third_libraries "")
set(face_pose_link_libraries "")
set(face_pose_include_directories "")

# 添加 find package 依赖
# find_package(CUDA REQUIRED)
find_package(PCL REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)

# 添加glogs 依赖
set(face_pose_third_libraries ${face_pose_include_directories} gflags glog)
set(EIGEN3_INCLUDE_DIRS ${CMAKE_CURRENT_LIST_DIR}/../dependency/eigen3)

# 添加 Opencv 依赖
find_package(OpenCV REQUIRED COMPONENTS)
set(face_pose_include_directories ${face_pose_include_directories} ${OpenCV_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS})
set(face_pose_third_libraries ${face_pose_third_libraries} ${OpenCV_LIBS} yaml-cpp libpthread.so)

# 添加 ceres 依赖
add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/../dependency/ceres)
set(CERES_INCLUDE_DIRS ${CMAKE_CURRENT_LIST_DIR}/../dependency/ceres/include)
set(face_pose_include_directories ${face_pose_include_directories} ${CERES_INCLUDE_DIRS})
set(face_pose_link_libraries ${face_pose_link_libraries} ${CMAKE_CURRENT_LIST_DIR}/../dependency/ceres ${CERES_LIBRARIES})

# 添加 PCL 依赖
find_package(PCL EXACT REQUIRED)
set(face_pose_include_directories ${face_pose_include_directories} ${PCL_INCLUDE_DIRS})
set(face_pose_link_libraries ${face_pose_link_libraries} ${PCL_LIBRARIES})

# catkin_package()
if(ROS)
    message(STATUS "ROS is found")
    list(APPEND CMAKE_PREFIX_PATH "/opt/ros/noetic")
    find_package(catkin REQUIRED COMPONENTS
    roscpp 
    roslib 
    std_msgs 
    sensor_msgs 
    pcl_conversions
    pcl_ros 
    cv_bridge 
    image_transport
    )
    set(face_pose_include_directories ${face_pose_include_directories}  ${catkin_INCLUDE_DIRS})
    set(face_pose_third_libraries ${face_pose_third_libraries}  ${catkin_LIBRARIES})
endif()

set(face_pose_include_directories ${face_pose_include_directories} ${catkin_INCLUDE_DIRS} )
set(face_pose_link_libraries ${face_pose_link_libraries} ${catkin_LIBRARIES}  yaml-cpp)
set(face_pose_link_libraries ${face_pose_link_libraries} /usr/local/lib/)
set(face_pose_link_libraries ${face_pose_link_libraries} ceres)

# 添加头文件路径
include_directories(${face_pose_include_directories}) 
# 添加库路径
link_directories(${face_pose_link_libraries} yaml-cpp)