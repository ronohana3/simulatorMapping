#ifndef DRONE_CONTROLLER_H
#define DRONE_CONTROLLER_H

#include "NavigationController.hpp"
#include "CameraController.hpp"

#include "simulator/simulator.h"

class DroneController 
{
public:
    DroneController(Simulator *sim, CamParam par);
    void navigateToBox(const cv::Rect &box);
    void getFrame(cv::Mat &dst);
private:
    NavigationController navigationController;
    CameraController camera;
    Simulator* simulator;

};

#endif