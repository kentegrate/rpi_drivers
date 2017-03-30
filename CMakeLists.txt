cmake_minimum_required(VERSION 2.8.3)
project(rpi_drivers)
set(CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS}")

option(HAS_BCM2835 "has bcm2835" OFF)

if(HAS_BCM2835)
  add_definitions(-DHAS_BCM2835)
endif()

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
)


catkin_package(
  INCLUDE_DIRS include
  LIBRARIES rpi_drivers
  )


include_directories(include ${catkin_INCLUDE_DIRS})

### Build libraries
if(HAS_BCM2835)
  add_library(mpu9250_driver src/mpu9250_i2c.cpp)
  target_link_libraries(mpu9250_driver bcm2835)

#  add_library(ads1115_driver src/ads1115.cpp)
#  target_link_libraries(ads1115_driver bcm2835)

  add_library(pca9685_driver src/pca9685.cpp)
  target_link_libraries(pca9685_driver bcm2835)

  add_library(ada_pca9685_driver src/ada_pca9685.cpp)
  target_link_libraries(ada_pca9685_driver bcm2835)

  add_library(ms5611_driver src/ms5611.cpp)
  target_link_libraries(ms5611_driver bcm2835)

  add_library(tb6552_driver src/tb6552.cpp)
  target_link_libraries(tb6552_driver bcm2835)

else()
  set(GTEST_ROOT ~/Downloads/googletest-release-1.8.0/)
  include_directories(${GTEST_ROOT}/include/)
  link_directories(${GTEST_ROOT}/build/)
endif()


### Build executables
add_executable(mpu9250_node src/mpu9250_node.cpp)
target_link_libraries(mpu9250_node ${catkin_LIBRARIES})
add_dependencies(mpu9250_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

#add_executable(ads1115_node src/ads1115_node.cpp)
#target_link_libraries(ads1115_node ${catkin_LIBRARIES})
#add_dependencies(ads1115_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

#add_executable(ms5611_node src/ms5611_node.cpp)
#target_link_libraries(ms5611_node ${catkin_LIBRARIES})
#add_dependencies(ms5611_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(pca9685_node src/pca9685_node.cpp)
target_link_libraries(pca9685_node ${catkin_LIBRARIES})
add_dependencies(pca9685_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(ada_pca9685_node src/ada_pca9685_node.cpp)
target_link_libraries(ada_pca9685_node ${catkin_LIBRARIES})
add_dependencies(ada_pca9685_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(tb6552_node src/tb6552_node.cpp)
target_link_libraries(tb6552_node ${catkin_LIBRARIES})
add_dependencies(tb6552_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

### Linking 
if(HAS_BCM2835)
  target_link_libraries(mpu9250_node mpu9250_driver)
#  target_link_libraries(ads1115_node ads1115_driver)  
#  target_link_libraries(ms5611_node ms5611_driver)
  target_link_libraries(pca9685_node pca9685_driver)
  target_link_libraries(ada_pca9685_node ada_pca9685_driver)
  target_link_libraries(tb6552_node tb6552_driver)
else(HAS_BCM2835)
  target_link_libraries(mpu9250_node gmock pthread)
#  target_link_libraries(ads1115_node gmock pthread) 
#  target_link_libraries(ms5611_node gmock pthread)
  target_link_libraries(pca9685_node gmock pthread)
  target_link_libraries(tb6552_node gmock pthread)
endif(HAS_BCM2835)

