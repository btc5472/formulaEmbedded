cmake_minimum_required(VERSION 2.8.3)

project(fsae_electric_vehicle)

## Compile as C++11, supported in ROS Kinetic and newer
## add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  roslaunch
  std_msgs
  genmsg
  message_generation
)

roslaunch_add_file_check(src/launch)

add_message_files(FILES
  gps.msg
  serial.msg
  speedometer.msg
  brake_pressure.msg
  coolant.msg
  drive_train.msg
  suspension.msg
  data_struct.msg
)

generate_messages(DEPENDENCIES std_msgs)

catkin_package(
	INCLUDE_DIRS socket.io-client-cpp
	CATKIN_DEPENDS message_runtime)

include_directories(
  socket.io-client-cpp
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME} socket.io-client-cpp/src/sio_client.cpp socket.io-client-cpp/src/sio_socket.cpp socket.io-client-cpp/src/internal/sio_client_impl.cpp socket.io-client-cpp/src/internal/sio_packet.cpp)

target_include_directories(${PROJECT_NAME} PUBLIC 
    socket.io-client-cpp/src 
    socket.io-client-cpp/lib/websocketpp 
    socket.io-client-cpp/lib/rapidjson/include
)



add_executable(DriveTrain src/cpp/drivetrain/drive_train.cpp src/cpp/drivetrain/CANController.cpp)
target_link_libraries(DriveTrain ${catkin_LIBRARIES})
target_compile_features(DriveTrain PUBLIC cxx_std_17)
add_dependencies(DriveTrain fsae_electric_vehicle_generate_messages_cpp)

add_executable(CoolantTemp src/cpp/Coolant_temp/main.cpp  src/cpp/Coolant_temp/CANController.cpp)
target_link_libraries(CoolantTemp ${catkin_LIBRARIES})
target_compile_features(CoolantTemp PUBLIC cxx_std_17)
add_dependencies(CoolantTemp fsae_electric_vehicle_generate_messages_cpp)

add_executable(Brakes src/cpp/brakes/brake_pressure.cpp src/cpp/brakes/CANController.cpp)
target_link_libraries(Brakes ${catkin_LIBRARIES})
target_compile_features(Brakes PUBLIC cxx_std_17)
add_dependencies(Brakes fsae_electric_vehicle_generate_messages_cpp)

add_executable(SocketIOSender src/cpp/SocketIOSender/main.cpp)
target_link_libraries(SocketIOSender ${catkin_LIBRARIES} ${PROJECT_NAME} )
target_compile_features(SocketIOSender PUBLIC cxx_std_17)
add_dependencies(SocketIOSender fsae_electric_vehicle_generate_messages_cpp)

add_executable(SPEEDOMETER src/cpp/speedometer/main.cpp)
target_link_libraries(SPEEDOMETER ${catkin_LIBRARIES})
target_compile_features(SPEEDOMETER PUBLIC cxx_std_17)
add_dependencies(SPEEDOMETER fsae_electric_vehicle_generate_messages_cpp)

add_executable(gps_lap_timer src/cpp/gps_lap_timer/gps_lap_timer.cpp src/cpp/gps_lap_timer/GPSUtilities.cpp src/cpp/gps_lap_timer/CANController.cpp)
target_link_libraries(gps_lap_timer ${catkin_LIBRARIES})
target_compile_features(gps_lap_timer PUBLIC cxx_std_17)
add_dependencies(gps_lap_timer fsae_electric_vehicle_generate_messages_cpp)

########################## These are old python files ############################
#
# This reader.py is for reading all data being published (except gps data) and outputting it
#
#catkin_install_python(
# PROGRAMS
#  src/python/reader/reader.py
#  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#)
#
# This gps.py is for reading GPS data being published and outputting it
#
#catkin_install_python(
#  PROGRAMS
#  src/python/gps/gps.py
#  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#)
##################################################################################



install(TARGETS SocketIOSender
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS SPEEDOMETER
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS Brakes
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS CoolantTemp
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS DriveTrain
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS gps_lap_timer
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

message("Bin Dir is ${CATKIN_PACKAGE_BIN_DESTINATION}")

install(PROGRAMS
   src/js/reader.js
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install (FILES
  src/launch/main.launch
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

