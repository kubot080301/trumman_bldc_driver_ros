cmake_minimum_required(VERSION 2.8.3)
project(motor_driver)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  serial
  std_msgs

)

include_directories(include ${catkin_INCLUDE_DIRS})

catkin_package(
  CATKIN_DEPENDS 
  INCLUDE_DIRS include
)

add_executable(motor_driver src/motor_driver.cpp)
target_link_libraries(motor_driver ${catkin_LIBRARIES})
add_dependencies(motor_driver motor_driver_gencpp)

#############
## Install ##
#############
# Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

install(TARGETS
    motor_driver
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  )



