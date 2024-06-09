#include "NavigationController.hpp"

void NavigationController::moveAlongBoxDirection(const cv::Rect &box, double distance, double velocity)
{
    cv::Point2i boxCenterPixel = cv::Point2i(box.x + box.width/2, box.y + box.height/2);

    
    float distanceFromDrone = 1.0 - (float)box.area()/(camParam.width * camParam.height);
    cv::Point2i frameCenterPixel = cv::Point2i((int)camParam.cx, (int)camParam.cy);

    // if (abs(boxCenterPixel.y - frameCenterPixel.y) < 5 && abs(boxCenterPixel.x - frameCenterPixel.x) < 5 && distanceFromDrone < 0.7)
    // {   
    //     // Attack
    //     moveForward(0.6, velocity);
    // }
    // else
    // {
    //     cv::Point3f direction = pixelToDirection(boxCenterPixel);
    //     moveAlongDirection(direction, distance, velocity);
    // }
    float distanceToMaintain = 0.95;
    float MinDistanceFromDrone = 0.7;
    int commandIntervalUsleepDelta = 0;

    if (distanceFromDrone > distanceToMaintain)
        commandIntervalUsleep -= commandIntervalUsleepDelta;
    else
        commandIntervalUsleep += commandIntervalUsleepDelta;


    if (distanceFromDrone > MinDistanceFromDrone)
    {
        cv::Point3f direction = pixelToDirection(boxCenterPixel);
        moveAlongDirection(direction, distance, velocity);
    }

    
}

void NavigationController::moveUp(double distance, double velocity) 
{
    std::string c = "up " + std::to_string(distance);
    simulator->command(c, commandIntervalUsleep, commandFps, totalCommandTimeInSecond);
}

void NavigationController::moveDown(double distance, double velocity)
{
    std::string c = "down " + std::to_string(distance);
    simulator->command(c, commandIntervalUsleep, commandFps, totalCommandTimeInSecond);
}

void NavigationController::moveForward(double distance, double velocity)
{
    std::string c = "forward " + std::to_string(distance);
    simulator->command(c, commandIntervalUsleep, commandFps, totalCommandTimeInSecond);
}

void NavigationController::moveLeft(double distance, double velocity)
{
    std::string c = "left " + std::to_string(distance);
    simulator->command(c, commandIntervalUsleep, commandFps, totalCommandTimeInSecond);
}

void NavigationController::moveRight(double distance, double velocity)
{
    std::string c = "right " + std::to_string(distance);
    simulator->command(c, commandIntervalUsleep, commandFps, totalCommandTimeInSecond);
}

void NavigationController::rotateCw(double angle, double velocity)
{
    std::string c = "cw " + std::to_string(angle);
    simulator->command(c, commandIntervalUsleep, commandFps, totalCommandTimeInSecond);
}

void NavigationController::rotateCcw(double angle, double velocity)
{
    std::string c = "ccw " + std::to_string(angle);
    simulator->command(c, commandIntervalUsleep, commandFps, totalCommandTimeInSecond);
}

cv::Point3f NavigationController::pixelToDirection(const cv::Point2i &pixel)
{
    double f = (double)camParam.f();
    

    double nx = (pixel.x - camParam.cx) / f;
    double ny = (pixel.y - camParam.cy) / f;
    cv::Point3f n = cv::Point3f(nx, ny, 1) / cv::norm(cv::Point3f(nx, ny, 1));
    // std::cout << "n=" << n << " boxCenter=" << pixel << std::endl;
    return n;
    // cv::Point2f frameCenter(camParam.cx, camParam.cy);
    // double distance = cv::norm(frameCenter - cv::Point2f(pixel.x, pixel.y));

    // std::cout << "f=" << f << " distance=" << distance  << " frameCenter=" << frameCenter << " boxCenter=" << pixel << std::endl;
    // float cosTheta = (float)(f / sqrt(f*f + distance*distance));
    // float sinTheta = (float)(distance / sqrt(f*f + distance*distance));
    // float cosPhi = (float)((pixel.x - camParam.cx) / distance);
    // float sinPhi = (float)((pixel.y - camParam.cy) / distance);
    // return cv::Point3f(cosPhi*sinTheta, sinPhi*sinTheta, cosTheta);

}

void NavigationController::moveAlongDirection(const cv::Point3f &direction, double distance, double velocity)
{
    // if (direction.x > 0)
    //     moveRight(distance*abs(direction.x), velocity);
    // else
    //     moveLeft(distance*abs(direction.x), velocity);
    float angle = std::acos(direction.z/(std::sqrt(direction.z*direction.z + direction.x*direction.x)));
    angle *= 180.0/M_PI;
    if (direction.x > 0)
        rotateCw(angle, velocity);
    else
        rotateCcw(angle, velocity);
    
    if (direction.y > 0)
        moveDown(distance*abs(direction.y), velocity);
    else
        moveUp(distance*abs(direction.y), velocity);
    
    moveForward(distance*abs(direction.z), velocity);
}