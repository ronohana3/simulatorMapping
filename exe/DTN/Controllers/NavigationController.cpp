#include "NavigationController.hpp"

void NavigationController::moveAlongBoxDirection(const cv::Rect &box, double distance, double velocity)
{
    cv::Point2i boxCenterPixel = cv::Point2i(box.x + box.width/2, box.y + box.height/2); 
    cv::Point3f direction = pixelToDirection(boxCenterPixel);
    moveAlongDirection(direction, distance, velocity);
}

void NavigationController::moveUp(double distance, double velocity) 
{
    std::string c = "up " + std::to_string(distance);
    simulator->command(c);
}

void NavigationController::moveDown(double distance, double velocity)
{
    std::string c = "down " + std::to_string(distance);
    simulator->command(c);
}

void NavigationController::moveForward(double distance, double velocity)
{
    std::string c = "forward " + std::to_string(distance);
    simulator->command(c);
}

void NavigationController::moveLeft(double distance, double velocity)
{
    std::string c = "left " + std::to_string(distance);
    simulator->command(c);
}

void NavigationController::moveRight(double distance, double velocity)
{
    std::string c = "right " + std::to_string(distance);
    simulator->command(c);
}

void NavigationController::rotateCw(double angle, double velocity)
{
    std::string c = "cw " + std::to_string(angle);
    simulator->command(c);
}

void NavigationController::rotateCcw(double angle, double velocity)
{
    std::string c = "ccw " + std::to_string(angle);
    simulator->command(c);
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
    // std::cout << "direction=" << direction << " direction norm=" << cv::norm(direction) << std::endl;
    if (direction.x > 0)
        moveRight(distance*abs(direction.x), velocity);
    else
        moveLeft(distance*abs(direction.x), velocity);
    
    if (direction.y > 0)
        moveDown(distance*abs(direction.y), velocity);
    else
        moveUp(distance*abs(direction.y), velocity);
    
    moveForward(distance*abs(direction.z), velocity);
}