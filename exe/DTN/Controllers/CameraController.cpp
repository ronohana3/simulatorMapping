#include "CameraController.hpp"



CameraController::CameraController(Simulator *sim) {
    simulator = sim;
}

void CameraController::getFrame(cv::Mat &dst)
{
    simulator->getFrame(dst);
}