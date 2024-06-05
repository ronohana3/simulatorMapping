#ifndef NAVIGATION_CONTROLLER_H
#define NAVIGATION_CONTROLLER_H

#include <opencv2/opencv.hpp>
#include "simulator/simulator.h"
#include "CameraController.hpp"

class NavigationController 
{
public:
    NavigationController(Simulator *sim, CamParam &par) : simulator(sim), camParam(par) {};
    void moveAlongBoxDirection(const cv::Rect &box, double distance, double velocity);
private:
    Simulator* simulator;
    CamParam camParam;
    
    void moveUp(double distance, double velocity);
    void moveDown(double distance, double velocity);
    void moveForward(double distance, double velocity);
    void moveLeft(double distance, double velocity);
    void moveRight(double distance, double velocity);
    void rotateCw(double angle, double velocity);
    void rotateCcw(double angle, double velocity);
    void moveAlongDirection(const cv::Point3f &direction, double distance, double velocity);
    cv::Point3f pixelToDirection(const cv::Point2i &pixel);
};

#endif