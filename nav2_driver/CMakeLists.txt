cmake_minimum_required(VERSION 2.8.3)
project(nav2_driver)

find_package(catkin REQUIRED COMPONENTS geometry_msgs nav_msgs roscpp std_msgs tf)
find_package(Boost REQUIRED COMPONENTS system)

catkin_package(
  CATKIN_DEPENDS geometry_msgs nav_msgs roscpp
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

add_library(nav2remote src/nav2remote.cpp)

add_executable(nav2_driver src/nav2_driver.cpp)
target_link_libraries(nav2_driver nav2remote ${catkin_LIBRARIES})

install(TARGETS nav2remote
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})

install(TARGETS nav2_driver
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        PATTERN ".svn" EXCLUDE)

install(DIRECTORY launch/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
        PATTERN ".svn" EXCLUDE)
