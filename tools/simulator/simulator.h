//
// Created by tzuk on 6/4/23.
//

#ifndef ORB_SLAM2_SIMULATOR_H
#define ORB_SLAM2_SIMULATOR_H

#include <memory>
#include <pangolin/pangolin.h>
#include <pangolin/geometry/geometry.h>
#include <pangolin/gl/glsl.h>
#include <pangolin/gl/glvbo.h>
#include <functional>
#include <vector>

#include <matplotlibcpp.h>

#include <pangolin/utils/file_utils.h>
#include <pangolin/geometry/glgeometry.h>
#include "System.h"
#include <Eigen/SVD>
#include <filesystem>
#include "include/run_model/TextureShader.h"
/**
 *  @class Simulator
 *  @brief This class provides a simulation environment for virtual robotic navigation and mapping.
 *
*/
class Simulator {
public:
    pangolin::OpenGlRenderState s_cam;

   Simulator();

   ~Simulator();

/**
 *Starts the 3D model viewer (pangolin)
 * */
    std::thread run();
/**
 * sample the simulator state, this changes from false to true once the model is loaded
 * @return is the simulator is ready
 * */
    bool isReady(){return ready;}


   void getFrame(cv::Mat &dst);

    /**
 * @brief Fetches the current location matrix from ORBSLAM2.
 *
 * @return A 4x4 location matrix where:
 * - The first 3x3 sub-matrix represents the rotation matrix.
 * - The last column represents the translation vector.
 * - The y-axis indicates the height in reverse (i.e., negative values correspond to upward direction).
 */
    cv::Mat getCurrentLocation();
/**
 * @brief Executes a specific command for controlling the virtual robot in the simulation, NOTICE: the available commands are in the commandMap object .
 *
 * @param command A string specifying the command to be executed.
 * @param intervalUsleep Optional parameter setting the sleep interval between command execution in microseconds. Default is 50000 microseconds.
 * @param fps Optional parameter defining the frames per second rate for visualization. Default is 30.0.
 * @param totalCommandTimeInSeconds Optional parameter setting the total duration for the command execution in seconds. Default is 1 second.
 *
 * This method enables the users to navigate the virtual robot in the simulation by executing specific commands.
 */
    void command(std::string &command, int intervalUsleep = 50000,
                 double fps = 30.0,
                 int totalCommandTimeInSeconds = 1);
/**
 * @brief kills the run thread
 *
 */
    void stop() { stopFlag = true; }

private:
    /**
 * @brief A map for controlling the virtual robot's actions.
 *
 * This property is an unordered map where:
 * - The key is a string representing a command for the virtual robot.
 * - The value is a boolean indicating whether the command is executable (true) or not (false).
 */
    std::unordered_map<std::string, bool> commandMap = {
            {"cw",      true},
            {"ccw",     true},
            {"forward", true},
            {"back",    true},
            {"right",   true},
            {"up",      true},
            {"down",    true},
            {"left",    true},
            {"flip",    false},
            {"rc",      false}};

    Eigen::Matrix3d K;
    bool stopFlag;
    bool ready;

    double movementFactor{};
    std::string modelPath;
    std::string secondModelPath;
    std::string modelTextureNameToAlignTo;
    std::vector<Eigen::Vector3d> Picks_w;
    bool isSaveMap;
    bool trackImages;
    bool cull_backfaces;
    pangolin::GlSlProgram program;
    pangolin::GlGeometry geomToRender;
    pangolin::GlGeometry secondGeomToRender;
    Eigen::Vector2i viewportDesiredSize;
    cv::Mat Tcw;
    std::mutex locationLock;
    std::mutex frameLock;
    cv::Mat m_frame;

    void simulatorRunThread();

    void extractSurface(const pangolin::Geometry &modelGeometry, std::string modelTextureNameToAlignTo,
                        Eigen::MatrixXf &surface);

    void alignModelViewPointToSurface(const pangolin::Geometry &modelGeometry, std::string modelTextureNameToAlignTo);

    void intervalOverCommand(const std::function<void(pangolin::OpenGlRenderState &, double &)> &func,
                             double value, int intervalUsleep,
                             double fps,
                             int totalCommandTimeInSeconds);

    void applyCommand(std::string &command, double value,
                 int intervalUsleep,
                 double fps,
                 int totalCommandTimeInSeconds);

    void static applyForwardToModelCam(pangolin::OpenGlRenderState &cam, double value);

    void static applyRightToModelCam(pangolin::OpenGlRenderState &cam, double value);

    void static applyYawRotationToModelCam(pangolin::OpenGlRenderState &cam, double value);

    void static applyUpModelCam(pangolin::OpenGlRenderState &cam, double value);

    void static applyPitchRotationToModelCam(pangolin::OpenGlRenderState &cam, double value);

};


#endif
