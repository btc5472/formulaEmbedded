This repo is a collection of C++ programs that run along side Robot Operating System (ROS) on a Jetson TX2 computer running a special version of Ubuntu Linux. We call this our Data Aquisition System (DAS). Its purpose its to collect data from various sensors around the car, typically from potentiometers, process that data and then send it off to a remote server to be stored and presented in real time on a website. This is unfinished code because its an ongoing project at my university but this repo is a fork of where the project was when I left.

I (Brandon Cobb) wrote the majority of the code in this repo. The one I worked most on was the node (program) that tracks the racecars position and number of laps around a track. This directory for this is formulaEmbedded/src/fsae_electric_vehicle/src/cpp/gps_lap_timer/gps_lap_timer.cpp

Embedded Team 2021 Code Update:

After dowloading the entire repo, download the Socket.io libaries in the link below and add to'src/fsae_electric_vehicle/socket.io-client-cpp':
https://drive.google.com/file/d/1SGfRvsVMYFg4fRMS2RImpZ6Gbzr-G704/view?usp=sharing
