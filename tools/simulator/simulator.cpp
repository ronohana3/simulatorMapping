//
// Created by tzuk on 6/4/23.
//

#include "simulator.h"
#include "secondModelPath.h"
#include <nlohmann/json.hpp>

cv::Mat Simulator::getCurrentLocation() {
    locationLock.lock();
    cv::Mat locationCopy = Tcw.clone();
    locationLock.unlock();
    return locationCopy;
}

void Simulator::getFrame(cv::Mat &dst) {
    frameLock.lock();
    if (!m_frame.empty())
        m_frame.copyTo(dst);
    frameLock.unlock();
}

Simulator::Simulator() : stopFlag(false), ready(false),cull_backfaces(false), viewportDesiredSize(640, 480) {

    std::ifstream fSettings(this->GetSettingsPath());
    nlohmann::json data = nlohmann::json::parse(fSettings);
    fSettings.close();
    float fx = data["Camera"]["fx"];
    float fy = data["Camera"]["fy"];
    float cx = data["Camera"]["cx"];
    float cy = data["Camera"]["cy"];
    K << fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0;
    
    modelPath = data["modelPath"];
    modelTextureNameToAlignTo = data["modelTextureNameToAlignTo"];
    secondModelPath = data["secondModelPath"];
    movementFactor = data["movementFactor"];
    
    secondModelMovementEnabled = data["secondModelMovementEnabled"];
    secondModelPositionTimeInterval = data["secondModelPositionTimeInterval"];
    secondModelInitialForwardDirection = Eigen::Vector2f(data["secondModelInitialForwardDirection"]["x"], data["secondModelInitialForwardDirection"]["z"]).normalized();
    secondModelYawAxis = Eigen::Vector3f(data["secondModelYawAxis"]["x"], data["secondModelYawAxis"]["y"], data["secondModelYawAxis"]["z"]).normalized();
    secondModelPitchAxis = Eigen::Vector3f(data["secondModelPitchAxis"]["x"], data["secondModelPitchAxis"]["y"], data["secondModelPitchAxis"]["z"]).normalized();
    secondModelpitchAngle = data["secondModelpitchAngle"];
}

Simulator::~Simulator() {
    m_frame.deallocate();
    program.ClearShaders();
}

void Simulator::command(std::string &command, int intervalUsleep, double fps, int totalCommandTimeInSeconds) {
    std::istringstream iss(command);
    std::string c;
    double value;
    iss >> c;
    if (commandMap.count(c) && commandMap[c]) {

        std::string stringValue;
        iss >> stringValue;
        value = std::stod(stringValue);
        applyCommand(c, value, intervalUsleep, fps, totalCommandTimeInSeconds);

    } else {
        std::cout << "the command " << c << " is not supported and will be skipped" << std::endl;
    }
}

void Simulator::simulatorRunThread() {
    pangolin::CreateWindowAndBind("Main", viewportDesiredSize[0], viewportDesiredSize[1]);
    glEnable(GL_DEPTH_TEST);
    s_cam = pangolin::OpenGlRenderState(
            pangolin::ProjectionMatrix(viewportDesiredSize(0), viewportDesiredSize(1), K(0, 0), K(1, 1), K(0, 2),
                                       K(1, 2), 0.1, 20),
            pangolin::ModelViewLookAt(0.1, -0.1, 0.3, 0, 0, 0, 0.0, -1.0,
                                      pangolin::AxisY)); // the first 3 value are meaningless because we change them later

    bool show_bounds = false;
    bool show_axis = false;
    bool show_x0 = false;
    bool show_y0 = false;
    bool show_z0 = false;

    pangolin::RegisterKeyPressCallback('b', [&]() { show_bounds = !show_bounds; });
    pangolin::RegisterKeyPressCallback('0', [&]() { cull_backfaces = !cull_backfaces; });
    pangolin::RegisterKeyPressCallback('a', [&]() { show_axis = !show_axis; });
    pangolin::RegisterKeyPressCallback('k', [&]() { stopFlag = !stopFlag; });
    pangolin::RegisterKeyPressCallback('x', [&]() { show_x0 = !show_x0; });
    pangolin::RegisterKeyPressCallback('y', [&]() { show_y0 = !show_y0; });
    pangolin::RegisterKeyPressCallback('z', [&]() { show_z0 = !show_z0; });
    pangolin::RegisterKeyPressCallback('w', [&]() { applyForwardToModelCam(s_cam, movementFactor); });
    pangolin::RegisterKeyPressCallback('a', [&]() { applyRightToModelCam(s_cam, movementFactor); });
    pangolin::RegisterKeyPressCallback('s', [&]() { applyForwardToModelCam(s_cam, -movementFactor); });
    pangolin::RegisterKeyPressCallback('d', [&]() { applyRightToModelCam(s_cam, -movementFactor); });
    pangolin::RegisterKeyPressCallback('e', [&]() { applyYawRotationToModelCam(s_cam, 1); });
    pangolin::RegisterKeyPressCallback('q', [&]() { applyYawRotationToModelCam(s_cam, -1); });
    pangolin::RegisterKeyPressCallback('2', [&]() { applyForwardToModelCam(s_cam, movementFactor); applyYawRotationToModelCam(s_cam, 1); });
    pangolin::RegisterKeyPressCallback('1', [&]() { applyForwardToModelCam(s_cam, movementFactor); applyYawRotationToModelCam(s_cam, -1); });
    pangolin::RegisterKeyPressCallback('r', [&]() { applyUpModelCam(s_cam, -movementFactor); });// ORBSLAM y axis is reversed
    pangolin::RegisterKeyPressCallback('f', [&]() { applyUpModelCam(s_cam, movementFactor); });
    
    
    const pangolin::Geometry modelGeometry = pangolin::LoadGeometry(modelPath);  
    alignModelViewPointToSurface(modelGeometry, modelTextureNameToAlignTo);
    geomToRender = pangolin::ToGlGeometry(modelGeometry);
    for (auto &buffer: geomToRender.buffers) {
        buffer.second.attributes.erase("normal");
    }

    const pangolin::Geometry secondModelGeometry = pangolin::LoadGeometry(secondModelPath);
    secondGeomToRender = pangolin::ToGlGeometry(secondModelGeometry);
    for (auto &buffer: secondGeomToRender.buffers) {
        buffer.second.attributes.erase("normal");
    }

    auto LoadProgram = [&]() {
        program.ClearShaders();
        program.AddShader(pangolin::GlSlAnnotatedShader, pangolin::shader);
        program.Link();
    };
    LoadProgram();
    pangolin::Handler3D handler(s_cam);
    pangolin::View &d_cam = pangolin::CreateDisplay()
            .SetBounds(0.0, 1.0, 0.0, 1.0, ((float) -viewportDesiredSize[0] / (float) viewportDesiredSize[1]))
            .SetHandler(&handler);

    // Second geomtry movement parameters
    int secondModelPositionIndex = 0;
    int secondModelYawAngleIndex = 0;    
    auto secondModelUpdatedTimestamp = chrono::system_clock::now();

    while (!pangolin::ShouldQuit() && !stopFlag) {
        
        // ready = true;
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        if (d_cam.IsShown()) {
            d_cam.Activate();

            if (cull_backfaces) {
                glEnable(GL_CULL_FACE);
                glCullFace(GL_BACK);
            }
            program.Bind();

            program.SetUniform("KT_cw", s_cam.GetProjectionMatrix() * s_cam.GetModelViewMatrix());
            pangolin::GlDraw(program, geomToRender, nullptr);


            if (secondModelMovementEnabled)
            {
                auto msSinceSecondModelPositionUpdated = chrono::duration_cast<chrono::milliseconds>(chrono::system_clock::now() - secondModelUpdatedTimestamp).count();
        
                if(msSinceSecondModelPositionUpdated > secondModelPositionTimeInterval)
                {
                    secondModelPositionIndex = (secondModelPositionIndex + 3) % secondModelPositionsCount;
                    secondModelYawAngleIndex = (secondModelYawAngleIndex + 1) % secondModelYawAngleRotationsCount;
                    secondModelUpdatedTimestamp = chrono::system_clock::now();
                }
            }

            Eigen::Vector3f secondModelPosition = Eigen::Vector3f(secondModelPositions[secondModelPositionIndex],
                                                                secondModelPositions[secondModelPositionIndex + 1], 
                                                                secondModelPositions[secondModelPositionIndex + 2]);
            
            // calculate yaw rotation matrix
            Eigen::Affine3f yawRotation = Eigen::Affine3f(Eigen::AngleAxisf(secondModelYawAngleRotations[secondModelYawAngleIndex], secondModelYawAxis.normalized()));
            // calculate pitch rotation matrix
            Eigen::Affine3f pitchRotation = Eigen::Affine3f(Eigen::AngleAxisf(secondModelpitchAngle, secondModelPitchAxis.normalized()));       
            
            // create the matrix transformation that combines both translation and rotations
            Eigen::Affine3f secondModelTransformation = Eigen::Affine3f::Identity();
            secondModelTransformation.translate(secondModelPosition);
            secondModelTransformation = secondModelTransformation * yawRotation;
            secondModelTransformation = secondModelTransformation * pitchRotation;
            

            program.SetUniform("KT_cw", s_cam.GetProjectionMatrix() * s_cam.GetModelViewMatrix() * secondModelTransformation.matrix());
            pangolin::GlDraw(program, secondGeomToRender, nullptr);

            program.Unbind();

            int viewport_size[4];
            glGetIntegerv(GL_VIEWPORT, viewport_size);

            pangolin::Image<unsigned char> buffer;
            pangolin::VideoPixelFormat fmt = pangolin::VideoFormatFromString("RGBA32");
            buffer.Alloc(viewport_size[2], viewport_size[3], viewport_size[2] * fmt.bpp / 8);
            glReadBuffer(GL_BACK);
            glPixelStorei(GL_PACK_ALIGNMENT, 1);
            glReadPixels(0, 0, viewport_size[2], viewport_size[3], GL_RGBA, GL_UNSIGNED_BYTE, buffer.ptr);
            
            cv::Mat imgBuffer = cv::Mat(viewport_size[3], viewport_size[2], CV_8UC4, buffer.ptr);
            
            frameLock.lock();
            cv::flip(imgBuffer, m_frame, 0);
            cv::cvtColor(m_frame, m_frame, cv::COLOR_RGB2BGR);
            frameLock.unlock();

            ready = true;

            s_cam.Apply();

            glDisable(GL_CULL_FACE);

            buffer.Dealloc();
        }

        pangolin::FinishFrame();
    }

    pangolin::DestroyWindow("Main");
}

std::thread Simulator::run() {
    std::thread thread(&Simulator::simulatorRunThread, this);
    return thread;
}

void Simulator::extractSurface(const pangolin::Geometry &modelGeometry, std::string modelTextureNameToAlignTo,
                               Eigen::MatrixXf &surface) {
    std::vector<Eigen::Vector3<unsigned int>> surfaceIndices;
    for (auto &o: modelGeometry.objects) {
        if (o.first == modelTextureNameToAlignTo) {
            const auto &it_vert = o.second.attributes.find("vertex_indices");
            if (it_vert != o.second.attributes.end()) {
                const auto &vs = std::get<pangolin::Image<unsigned int>>(it_vert->second);
                for (size_t i = 0; i < vs.h; ++i) {
                    const Eigen::Map<const Eigen::Vector3<unsigned int>> v(vs.RowPtr(i));
                    surfaceIndices.emplace_back(v);
                }
            }
        }
    }
    surface = Eigen::MatrixXf(surfaceIndices.size() * 3, 3);
    int currentIndex = 0;
    for (const auto &b: modelGeometry.buffers) {
        const auto &it_vert = b.second.attributes.find("vertex");
        if (it_vert != b.second.attributes.end()) {
            const auto &vs = std::get<pangolin::Image<float>>(it_vert->second);
            for (auto &row: surfaceIndices) {
                for (auto &i: row) {
                    const Eigen::Map<const Eigen::Vector3f> v(vs.RowPtr(i));
                    surface.row(currentIndex++) = v;
                }
            }
        }
    }
}

void Simulator::applyPitchRotationToModelCam(pangolin::OpenGlRenderState &cam, double value) {
    double rand = double(value) * (M_PI / 180);
    double c = std::cos(rand);
    double s = std::sin(rand);

    Eigen::Matrix3d R;
    R << 1, 0, 0,
         0, c, -s,
         0, s, c;

    Eigen::Matrix4d pangolinR = Eigen::Matrix4d::Identity();
    pangolinR.block<3, 3>(0, 0) = R;

    auto camMatrix = pangolin::ToEigen<double>(cam.GetModelViewMatrix());

    // Left-multiply the rotation
    camMatrix = pangolinR * camMatrix;

    // Convert back to pangolin matrix and set
    pangolin::OpenGlMatrix newModelView;
    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j) {
            newModelView.m[j * 4 + i] = camMatrix(i, j);
        }
    }

    cam.SetModelViewMatrix(newModelView);
}

void Simulator::intervalOverCommand(
        const std::function<void(pangolin::OpenGlRenderState &, double &)> &func, double value,
        int intervalUsleep, double fps, int totalCommandTimeInSeconds) {
    double intervalValue = value / (fps * totalCommandTimeInSeconds);
    int intervalIndex = 0;
    while (intervalIndex <= fps * totalCommandTimeInSeconds) {
        usleep(intervalUsleep);
        func(s_cam, intervalValue);
        intervalIndex += 1;
    }
}

void Simulator::applyCommand(std::string &command, double value, int intervalUsleep, double fps,
                             int totalCommandTimeInSeconds) {
    if (command == "cw") {
        intervalOverCommand(Simulator::applyYawRotationToModelCam, value, intervalUsleep, fps,
                            totalCommandTimeInSeconds);
    } else if (command == "ccw") {
        intervalOverCommand(Simulator::applyYawRotationToModelCam, -1 * value,
                            intervalUsleep, fps,
                            totalCommandTimeInSeconds);
    } else if (command == "forward") {
        intervalOverCommand(Simulator::applyForwardToModelCam, value, intervalUsleep, fps,
                            totalCommandTimeInSeconds);
    } else if (command == "back") {
        intervalOverCommand(Simulator::applyForwardToModelCam, -1 * value, intervalUsleep,
                            fps, totalCommandTimeInSeconds);
    } else if (command == "right") {
        intervalOverCommand(Simulator::applyRightToModelCam, -1 * value, intervalUsleep,
                            fps, totalCommandTimeInSeconds);
    } else if (command == "left") {
        intervalOverCommand(Simulator::applyRightToModelCam, value, intervalUsleep, fps,
                            totalCommandTimeInSeconds);
    } else if (command == "up") {
        intervalOverCommand(Simulator::applyUpModelCam, -1 * value, intervalUsleep, fps,
                            totalCommandTimeInSeconds);
    } else if (command == "down") {
        intervalOverCommand(Simulator::applyUpModelCam, value, intervalUsleep, fps,
                            totalCommandTimeInSeconds);
    }
}

void Simulator::applyUpModelCam(pangolin::OpenGlRenderState &cam, double value) {
    auto camMatrix = pangolin::ToEigen<double>(cam.GetModelViewMatrix());
    camMatrix(1, 3) += value;
    cam.SetModelViewMatrix(camMatrix);
}

void Simulator::applyForwardToModelCam(pangolin::OpenGlRenderState &cam, double value) {
    auto camMatrix = pangolin::ToEigen<double>(cam.GetModelViewMatrix());
    camMatrix(2, 3) += value;
    cam.SetModelViewMatrix(camMatrix);
}

void Simulator::applyRightToModelCam(pangolin::OpenGlRenderState &cam, double value) {
    auto camMatrix = pangolin::ToEigen<double>(cam.GetModelViewMatrix());
    camMatrix(0, 3) += value;
    cam.SetModelViewMatrix(camMatrix);
}

void Simulator::applyYawRotationToModelCam(pangolin::OpenGlRenderState &cam, double value) {
    double rand = double(value) * (M_PI / 180);
    double c = std::cos(rand);
    double s = std::sin(rand);

    Eigen::Matrix3d R;
    R << c, 0, s,
            0, 1, 0,
            -s, 0, c;

    Eigen::Matrix4d pangolinR = Eigen::Matrix4d::Identity();
    pangolinR.block<3, 3>(0, 0) = R;

    auto camMatrix = pangolin::ToEigen<double>(cam.GetModelViewMatrix());

    // Left-multiply the rotation
    camMatrix = pangolinR * camMatrix;

    // Convert back to pangolin matrix and set
    pangolin::OpenGlMatrix newModelView;
    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j) {
            newModelView.m[j * 4 + i] = camMatrix(i, j);
        }
    }

    cam.SetModelViewMatrix(newModelView); 
}

void Simulator::alignModelViewPointToSurface(const pangolin::Geometry &modelGeometry,
                                        std::string modelTextureNameToAlignTo) {
    Eigen::MatrixXf surface;
    extractSurface(modelGeometry, modelTextureNameToAlignTo, surface); 
    Eigen::JacobiSVD<Eigen::MatrixXf> svd(surface, Eigen::ComputeThinU | Eigen::ComputeThinV);
    svd.computeV();
    Eigen::Vector3f v = svd.matrixV().col(2);
    const auto mvm = pangolin::ModelViewLookAt(v.x(), v.y(), v.z(), 0, 0, 0, 0.0,
                                               -1.0,
                                               pangolin::AxisY);
    const auto proj = pangolin::ProjectionMatrix(viewportDesiredSize(0), viewportDesiredSize(1), K(0, 0), K(1, 1),
                                                 K(0, 2), K(1, 2), 0.1, 20);
    s_cam.SetModelViewMatrix(mvm);
    s_cam.SetProjectionMatrix(proj);
    applyPitchRotationToModelCam(s_cam, -90);
}