#include "DroneController.hpp"

DroneController::DroneController(Simulator *sim, CamParam par) : 
simulator(sim), camera(sim, par), navigationController(sim, par) {}

void DroneController::getFrame(cv::Mat &dst)
{
    camera.getFrame(dst);
}

void DroneController::navigateToBox(const cv::Rect box)
{ 
    navigationController.moveAlongBoxDirection(box, 0.2, 1);
}
    
