cmake_minimum_required(VERSION 2.8.3)
project(ls01d)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  message_generation
)

add_message_files(
  FILES
  LaserRanges.msg)

add_service_files(
  FILES
  CMD.srv)


generate_messages(DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS
  roscpp
  rospy
  std_msgs
  message_runtime
  )



include_directories(
  include ${catkin_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}/src
)



add_executable(ls01d src/main.cpp src/uart_driver.cpp)
add_dependencies(ls01d ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(ls01d ${catkin_LIBRARIES})


add_executable(ls01d2
  src/ls01d.cpp
  src/uart_driver.cpp
)
add_dependencies(ls01d2 ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(ls01d2
  ${catkin_LIBRARIES}
)


install(TARGETS ls01d
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
