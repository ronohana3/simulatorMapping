#ifndef CAMERA_CONTROLLER_H
#define CAMERA_CONTROLLER_H

#include <opencv2/opencv.hpp>
#include <string>

#include "simulator/simulator.h"

typedef struct sCamParam
{
    uint width;
    uint height;
    float fx;
    float fy;
    float cx;
    float cy;
    float f() { return (fx + fy)/2; };
} CamParam;

class CameraController 
{
public:
    CameraController(Simulator* sim, CamParam &par);
    void getFrame(cv::Mat &dst);
    CamParam param;
private:
    Simulator* simulator;
};

#endif