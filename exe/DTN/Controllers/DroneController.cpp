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

void DroneController::scan(const cv::Rect box)
{ 
    bool isCw = true;
    if (!box.empty())
        isCw = (box.x + box.width/2) > camera.param.width/2;
    navigationController.rotateInPlace(isCw, 5);
}
    
