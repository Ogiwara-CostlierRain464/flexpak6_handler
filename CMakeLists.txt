cmake_minimum_required(VERSION 2.8.3)
project(flexpak6_handler)

find_package(catkin REQUIRED COMPONENTS
  roscpp
)


include_directories(
  ${catkin_INCLUDE_DIRS}
)


file(GLOB_RECURSE PROJECT_SOURCES src/*.cpp)
file(GLOB_RECURSE PROJECT_HEADERS include/*.h)

add_executable(${PROJECT_NAME}_node
        ${PROJECT_SOURCES}
        ${PROJECT_HEADERS}
        )

target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
)
