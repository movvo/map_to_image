cmake_minimum_required(VERSION 3.5)
project(map_to_image)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -lgtest -lgmock)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)

# Include directories
include_directories(
  include
  test/include
)

# Add cpp libraries
add_library(${PROJECT_NAME}_lib
  "src/map_to_image.cpp"
)

ament_target_dependencies(${PROJECT_NAME}_lib 
                          rclcpp 
                          std_msgs
                          sensor_msgs
                          nav_msgs
                          cv_bridge
                          image_transport
                        )

# Add executable
add_executable(${PROJECT_NAME}
                src/main.cpp
              )
target_link_libraries(${PROJECT_NAME} ${PROJECT_NAME}_lib)
ament_target_dependencies(${PROJECT_NAME}
                          rclcpp
                          std_msgs
                          sensor_msgs
                          nav_msgs
                          cv_bridge
                          image_transport
                        )

# Installs
install(TARGETS
        ${PROJECT_NAME}
        DESTINATION lib/${PROJECT_NAME}
      )

install(DIRECTORY
        config
        DESTINATION share/${PROJECT_NAME}
      )

install(DIRECTORY
        launch
        DESTINATION share/${PROJECT_NAME}
      )

ament_package()