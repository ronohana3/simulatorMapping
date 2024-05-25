#ifndef CAMERA_CONTROLLER_H
#define CAMERA_CONTROLLER_H

#include <opencv2/opencv.hpp>
#include <string>

#include "simulator/simulator.h"

class CameraController 
{
public:
    CameraController(Simulator* sim);
    void getFrame(cv::Mat &dst);
private:
    Simulator* simulator;
};

#endif