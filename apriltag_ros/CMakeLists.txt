cmake_minimum_required(VERSION 3.5)
project(apriltag_ros)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  #add_compile_options(-Wall -Wextra -Wpedantic)
endif()


set (dependencies ament_cmake
                  rclcpp
                  std_msgs
                  std_srvs
                  brain_box_msgs
                  cv_bridge
                  Eigen3
                  OpenCV
                  apriltag
                  tf2
                  tf2_ros
                  image_transport
                  am_utils
                  sensor_msgs)

find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(apriltag REQUIRED)

foreach(Dependency IN ITEMS ${dependencies})
  find_package(${Dependency} REQUIRED)
endforeach()

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
# We default to 'Release' if none was specified
IF(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  MESSAGE(STATUS "Setting build type to 'Release' as none was specified.")
  SET(CMAKE_BUILD_TYPE Release CACHE STRING "Choose the build type" FORCE)
  SET_PROPERTY(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "Coverage" "Debug" "Release" "MinSizeRel" "RelWithDebInfo")
ENDIF()

add_compile_options("-O3" "-funsafe-loop-optimizations" "-fsee" "-funroll-loops" "-fno-math-errno" "-funsafe-math-optimizations" "-ffinite-math-only" "-fno-signed-zeros")


# Extract the include directories and libraries from apriltag::apriltag as catkin_package() does not support modern cmake.
get_target_property(apriltag_INCLUDE_DIRS apriltag::apriltag INTERFACE_INCLUDE_DIRECTORIES)
get_target_property(apriltag_LIBRARIES apriltag::apriltag INTERFACE_LINK_LIBRARIES)


###########
## Build ##
###########

include_directories(include
  ${EIGEN3_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}_common src/common_functions.cpp)
target_link_libraries(${PROJECT_NAME}_common ${OpenCV_LIBRARIES} apriltag::apriltag)
ament_target_dependencies(${PROJECT_NAME}_common ${dependencies})

add_library(${PROJECT_NAME}_continuous_detector src/continuous_detector.cpp)
target_link_libraries(${PROJECT_NAME}_continuous_detector ${PROJECT_NAME}_common ${catkin_LIBRARIES})
ament_target_dependencies(${PROJECT_NAME}_continuous_detector ${dependencies})

add_library(${PROJECT_NAME}_single_image_detector src/single_image_detector.cpp)
target_link_libraries(${PROJECT_NAME}_single_image_detector ${PROJECT_NAME}_common ${catkin_LIBRARIES})
ament_target_dependencies(${PROJECT_NAME}_single_image_detector ${dependencies})


add_executable(${PROJECT_NAME}_continuous_node src/${PROJECT_NAME}_continuous_node.cpp)
target_link_libraries(${PROJECT_NAME}_continuous_node ${PROJECT_NAME}_continuous_detector)
ament_target_dependencies(${PROJECT_NAME}_continuous_node ${dependencies})

add_executable(${PROJECT_NAME}_single_image_server_node src/${PROJECT_NAME}_single_image_server_node.cpp)
target_link_libraries(${PROJECT_NAME}_single_image_server_node ${PROJECT_NAME}_single_image_detector )
ament_target_dependencies(${PROJECT_NAME}_single_image_server_node ${dependencies})

add_executable(${PROJECT_NAME}_single_image_client_node src/${PROJECT_NAME}_single_image_client_node.cpp)
target_link_libraries(${PROJECT_NAME}_single_image_client_node ${PROJECT_NAME}_common)
ament_target_dependencies(${PROJECT_NAME}_single_image_client_node ${dependencies})


#############
## Install ##
#############


install(TARGETS
  ${PROJECT_NAME}_continuous_node
  ${PROJECT_NAME}_single_image_server_node
  ${PROJECT_NAME}_single_image_client_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/
        DESTINATION include/
    )
    
install(DIRECTORY launch/
        DESTINATION share/${PROJECT_NAME}
    )
install(
  TARGETS ${PROJECT_NAME}_common
  EXPORT ${PROJECT_NAME}_commonTargets
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

install(
  TARGETS ${PROJECT_NAME}_continuous_detector
  EXPORT ${PROJECT_NAME}_continuous_detectorTargets
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

install(
  TARGETS ${PROJECT_NAME}_single_image_detector
  EXPORT ${PROJECT_NAME}_single_image_detectorTargets
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

ament_export_include_directories(include)
ament_export_libraries(apriltag_ros)
ament_export_dependencies(${dependencies})
ament_package()