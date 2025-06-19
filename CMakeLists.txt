cmake_minimum_required(VERSION 3.8)
project(ps5_publisher)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(image_transport REQUIRED)


# Adicionar executável ps5_pub
add_executable(ps5_pub src/ps5_pub.cpp)
ament_target_dependencies(ps5_pub rclcpp sensor_msgs cv_bridge OpenCV yaml-cpp image_transport)

# Adicionar executável disparity_to_rviz


# Diretórios de inclusão
target_include_directories(ps5_pub PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(ps5_pub PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

# Definir o caminho para o arquivo stereo_calib.yaml
target_compile_definitions(ps5_pub PUBLIC LEFT_CALIB_PATH="${CMAKE_CURRENT_SOURCE_DIR}/include/left.yaml")
target_compile_definitions(ps5_pub PUBLIC RIGHT_CALIB_PATH="${CMAKE_CURRENT_SOURCE_DIR}/include/right.yaml")

target_link_libraries(ps5_pub ${OpenCV_LIBRARIES} yaml-cpp)


install(TARGETS ps5_pub
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

install(DIRECTORY include/
  DESTINATION share/${PROJECT_NAME}/include
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
