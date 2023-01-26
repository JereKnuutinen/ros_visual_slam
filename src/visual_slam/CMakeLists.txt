cmake_minimum_required(VERSION 3.5)
project(visual_slam)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
# Include open cv
find_package( OpenCV REQUIRED)
message(STATUS "OpenCV directory is ${OpenCV_INCLUDE_DIRS}")
include_directories( ${OpenCV_INCLUDE_DIRS} )


# Find Eigen3
find_package(Eigen3 REQUIRED NO_MODULE)
# set(Easy3D_DIR "/usr/local/Cellar/easy3d/2.5.2/lib/CMake/Easy3D")

#include_directories(${CERES_INCLUDE_DIRS})

#set(G2O_USE_VENDORED_CERES ON CACHE BOOL "Use vendored Ceres from g2o/EXTERNAL/ceres")
# SET (G2O_LIB_TYPE SHARED)
LIST( APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules )
set(G2O_INCLUDE_DIRS
/usr/local/include)
set(G2O_LIB_DIRS
/usr/local/lib)
include_directories(${G2O_INCLUDE_DIRS})
link_directories(${G2O_LIB_DIRS})
FIND_PACKAGE( G2O )
SET(G2O_LIBS g2o_cli g2o_ext_freeglut_minimal g2o_simulator g2o_solver_slam2d_linear g2o_types_icp g2o_types_slam2d g2o_core g2o_interface g2o_solver_csparse g2o_solver_structure_only g2o_types_sba g2o_types_slam3d g2o_csparse_extension g2o_opengl_helper g2o_solver_dense g2o_stuff g2o_types_sclam2d g2o_parser g2o_solver_pcg g2o_types_data g2o_types_sim3 )


# Find YAML-CPP

find_package(YAML-CPP REQUIRED)
include_directories(${YAML_CPP_INCLUDE_DIR})

find_package(Boost REQUIRED filesystem)
INCLUDE_DIRECTORIES( ${Boost_INCLUDE_DIR} )

add_executable(run_slam_node src/main.cpp)
ament_target_dependencies(run_slam_node
    "rclcpp"
    "sensor_msgs"
    "std_msgs"
    "geometry_msgs"
)
target_link_libraries(run_slam_node Eigen3::Eigen)

target_link_libraries(run_slam_node ${OpenCV_LIBS} )
target_link_libraries(run_slam_node ${G2O_LIBS})
target_link_libraries(run_slam_node Boost::filesystem)
target_link_libraries(run_slam_node ${PCL_LIBRARIES})
target_link_libraries(run_slam_node ${YAML_CPP_LIBRARIES})
target_sources(run_slam_node PUBLIC ${SOURCES})

target_include_directories(run_slam_node PUBLIC .)
target_include_directories(run_slam_node PUBLIC src)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
#target_link_libraries(gazebo_my_robot ${robot_description_LIBRARIES})

install(
  TARGETS
  run_slam_node
  DESTINATION lib/${PROJECT_NAME}
)



# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
