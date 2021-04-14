#include "ros/ros.h"
#include <ros/callback_queue.h>
#include "fsae_electric_vehicle/serial.h"
#include "fsae_electric_vehicle/speedometer.h"
#include "fsae_electric_vehicle/brake_pressure.h"
#include "fsae_electric_vehicle/coolant.h"
#include "fsae_electric_vehicle/drive_train.h"
#include "fsae_electric_vehicle/gps.h"
#//include "/home/faizan/catkin_ws/src/fsae_electric_vehicle/socket.io-client-cpp/src/sio_client.h" //Change to directory where the header file is at or it won't compile
#include <sio_client.h>
#include <string>
#include <mutex>
//#include <socket.io-client-cpp/src/sio_client.h>
//#include "fsae_electric_vehicle/sio_client.h"

  //std::string speedVal;
  float speedVal;
  float driveTrain;
  float coolantTemp;
  float brakePressure;
  
  std::mutex dataMutex;

  void speedCallback(const fsae_electric_vehicle::speedometer::ConstPtr& msg){
 	std::lock_guard<std::mutex> lock{dataMutex};
 	memcpy(&speedVal, &msg->speed, sizeof(speedVal)+1);
 }
  
  void gpsCallback(const fsae_electric_vehicle::gps::ConstPtr& msg) {
	//std::lock_guard<std::mutex> lock{dataMutex};
 }

 void brakeCallback(const fsae_electric_vehicle::brake_pressure::ConstPtr& msg) {
	std::lock_guard<std::mutex> lock{dataMutex};
 	memcpy(&brakePressure, &msg->pressure, sizeof(speedVal)+1);
 }

 void coolantCallback(const fsae_electric_vehicle::coolant::ConstPtr& msg) {
	std::lock_guard<std::mutex> lock{dataMutex};
 	memcpy(&coolantTemp, &msg->temperature, sizeof(speedVal)+1);
 }

 void batteryCallback(const fsae_electric_vehicle::drive_train::ConstPtr& msg) {
	std::lock_guard<std::mutex> lock{dataMutex};
 	memcpy(&driveTrain, &msg->voltage, sizeof(speedVal)+1);
 }

int main(int argc, char **argv) {

  sio::client h;
  h.connect("http://64.227.48.74:2222");
  
  //sio::message::list speedData(speedVal);
  //sio::message::list speedData(std::to_string(speedVal));


  ros::init(argc, argv, "SocketIOSender");
  ros::NodeHandle n;
  

  ros::Subscriber speed = n.subscribe<fsae_electric_vehicle::speedometer>("speedometer", 1000, speedCallback);
  ros::Subscriber gps = n.subscribe<fsae_electric_vehicle::gps>("GPS", 1000, gpsCallback);
  ros::Subscriber bat = n.subscribe<fsae_electric_vehicle::drive_train>("drivetrain_voltage", 1000, batteryCallback);
  ros::Subscriber cool = n.subscribe<fsae_electric_vehicle::coolant>("coolant_temperature", 1000, coolantCallback);
  ros::Subscriber brake = n.subscribe<fsae_electric_vehicle::brake_pressure>("brake_pressure", 1000, brakeCallback);
  

  
  ros::Rate loop_rate{30};



  while (ros::ok()) {
    ros::spinOnce();
    sio::message::list speedData(std::to_string(speedVal));
    sio::message::list brakeData(std::to_string(brakePressure));
    sio::message::list coolantData(std::to_string(coolantTemp));
    sio::message::list batteryData(std::to_string(driveTrain));
    
    h.socket()->emit("speedometer", speedData);
    h.socket()->emit("driveTrain", batteryData);
    h.socket()->emit("coolantTemp", coolantData);
    h.socket()->emit("brakePressure", brakeData);
    
    
    
    loop_rate.sleep();
  }
}
