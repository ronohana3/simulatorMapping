#include "DroneController.hpp"

DroneController::DroneController(Simulator *sim, CamParam par) : 
simulator(sim), camera(sim, par), navigationController(sim, par) {}

void DroneController::getFrame(cv::Mat &dst)
{
    camera.getFrame(dst);
}

void DroneController::navigateToBox(const cv::Rect &box)
{ 
    navigationController.moveAlongBoxDirection(box, 0.2, 1);
    // std::string c;
    // float distance = 1.0 - (float)box.boundingRect().area()/frame.size().area();
    // std::cout << distance << std::endl;
    // int dx = 5, dy = 5;
    
    // const float moveStep = 0.05*distance;
    // const float angleStep = 2*(1 - distance);
    // cv::Point2i frameCenter(frame.size().width / 2, frame.size().height / 2);
    // if (abs(box.center.x - frameCenter.x) > dx)
    // {
    //     c = box.center.x > frameCenter.x ? "cw " : "ccw ";
    //     c += std::to_string(angleStep);
    //     simulator->command(c);
    // }

    // if (abs(box.center.y - frameCenter.y) > dy)
    // {
    //     c = box.center.y > frameCenter.y ? "down " : "up ";
    //     c += std::to_string(moveStep);
    //     simulator->command(c);
    // }

    // if (distance > 0.65)
    // {
    //     c = "forward ";
    //     c += std::to_string(moveStep);
    //     simulator->command(c);
    // }
    // else if (abs(box.center.y - frameCenter.y) < dy && abs(box.center.x - frameCenter.x) < dx)
    // {   
    //     // Attack
    //     c = "forward 0.4";
    //     simulator->command(c);
    // }
}
    
