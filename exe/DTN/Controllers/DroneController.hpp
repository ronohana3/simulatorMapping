#ifndef DRONE_CONTROLLER_H
#define DRONE_CONTROLLER_H

#include "NavigationController.hpp"
#include "CameraController.hpp"

#include "simulator/simulator.h"

class DroneController 
{
public:
    DroneController(Simulator *sim);
    void navigateToBox(const cv::Mat &frame, const cv::RotatedRect &box);
    void getFrame(cv::Mat &dst);
private:
    NavigationController navigationController;
    CameraController camera;
    Simulator* simulator;

};

#endif