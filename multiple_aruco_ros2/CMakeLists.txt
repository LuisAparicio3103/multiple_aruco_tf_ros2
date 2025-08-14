cmake_minimum_required(VERSION 3.8)
project(multiple_aruco_ros2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
set(CMAKE_CXX_STANDARD 11) # C++11...
set(CMAKE_CXX_STANDARD_REQUIRED ON) #...is required...
set(CMAKE_CXX_EXTENSIONS ON) #...with compiler extensions like gnu++11

set(THIS_PACKAGE_INCLUDE_DEPENDS
    OpenCV
    cv_bridge
    geometry_msgs
    image_transport
    rclcpp
    rclpy
    tf2
    tf2_ros
    tf2_geometry_msgs
    aruco
    aruco_msgs
    sensor_msgs
    visualization_msgs
    turtlesim
)
find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

add_executable(multiple_aruco_tf_ros2 src/multiple_aruco_tf_ros2.cpp)
target_include_directories(multiple_aruco_tf_ros2
  PUBLIC
  include)

target_include_directories(multiple_aruco_tf_ros2
  SYSTEM PUBLIC
  ${OpenCV_INCLUDE_DIRS}
)
ament_target_dependencies(multiple_aruco_tf_ros2 ${THIS_PACKAGE_INCLUDE_DEPENDS})
target_link_libraries(multiple_aruco_tf_ros2 ${OpenCV_LIBRARIES})

add_executable(relative_pose src/relative_pose.cpp)
target_include_directories(relative_pose
  PUBLIC
  include)

target_include_directories(relative_pose
  SYSTEM PUBLIC
  ${OpenCV_INCLUDE_DIRS}
)
ament_target_dependencies(relative_pose ${THIS_PACKAGE_INCLUDE_DEPENDS})
target_link_libraries(relative_pose ${OpenCV_LIBRARIES})

add_executable(obtener_imagen src/obtener_imagen.cpp)
target_include_directories(obtener_imagen
  PUBLIC
  include)

target_include_directories(obtener_imagen
  SYSTEM PUBLIC
  ${OpenCV_INCLUDE_DIRS}
)
ament_target_dependencies(obtener_imagen ${THIS_PACKAGE_INCLUDE_DEPENDS})
target_link_libraries(obtener_imagen ${OpenCV_LIBRARIES})

#add_executable(listener_tf src/listener_tf.cpp)
#target_include_directories(listener_tf
#  PUBLIC
#  include)

#target_include_directories(listener_tf
#  SYSTEM PUBLIC
#  ${OpenCV_INCLUDE_DIRS}
#)
#ament_target_dependencies(listener_tf ${THIS_PACKAGE_INCLUDE_DEPENDS})
#target_link_libraries(listener_tf ${OpenCV_LIBRARIES})


# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()


install(TARGETS
  multiple_aruco_tf_ros2
  relative_pose
  obtener_imagen
#  listener_tf
  DESTINATION lib/${PROJECT_NAME})

	
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}/)  


ament_package()
