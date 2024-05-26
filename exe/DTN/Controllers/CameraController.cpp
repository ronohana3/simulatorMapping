#include "CameraController.hpp"



CameraController::CameraController(Simulator *sim, CamParam &par) {
    simulator = sim;
    param = par;
}

void CameraController::getFrame(cv::Mat &dst)
{
    simulator->getFrame(dst);
}