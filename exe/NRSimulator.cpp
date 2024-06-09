//
// Created by tzuk on 6/4/23.
//
#include <matplotlibcpp.h>
#include "simulator/simulator.h"
#include "navigation/RoomExit.h"
#include "include/Auxiliary.h"


// Function to extract camera position and look-at direction
void ExtractCameraParams(const pangolin::OpenGlRenderState& s_cam, Eigen::Vector3f& position, Eigen::Vector3f& lookat_direction) {
    // Retrieve ModelViewMatrix
    pangolin::OpenGlMatrix mv = s_cam.GetModelViewMatrix();
    
    // Convert OpenGlMatrix to Eigen::Matrix4f
    Eigen::Matrix4f model_view;
    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j) {
            model_view(i, j) = mv.m[j*4 + i];
        }
    }

    // Compute the camera position by transforming (0,0,0) using the inverse ModelViewMatrix
    Eigen::Affine3f mv_inverse = Eigen::Affine3f(model_view).inverse();
    position = mv_inverse * Eigen::Vector3f::Zero();

    // Extract the look-at direction (negative Z-axis of the camera in world coordinates)
    lookat_direction = -model_view.block<3, 1>(0, 2);
}

// Function to calculate the signed angle between two vectors
float SignedAngleBetweenVectors(const Eigen::Vector2f& vec1, const Eigen::Vector2f& vec2) {
    // Normalize the input vectors
    Eigen::Vector2f v1 = vec1.normalized();
    Eigen::Vector2f v2 = vec2.normalized();
    
    // Compute dot product (cosine of the angle)
    float dot_product = v1.dot(v2);
    
    // Compute the cross product in 2D (gives a scalar value representing the z-component in 3D)
    float cross_product = v1.x() * v2.y() - v1.y() * v2.x();
    
    // Compute the angle using atan2 (y, x)
    float angle = atan2(cross_product, dot_product);

    // Convert angle to range [0, 2π]
    if (angle < 0) {
        angle += 2 * M_PI;
    }
    
    return angle;
}



int main(int argc, char **argv) {    
    
    Simulator *simulator = new Simulator();

    auto simulatorThread = simulator->run();
    while (!simulator->isReady()) { // wait for the 3D model to load
        usleep(1000);
    }

    pangolin::OpenGlMatrix reset_mvm = simulator->s_cam.GetModelViewMatrix();
    bool generatePosition = false;
    bool printPosition = false;


    while(true)
    {
        Eigen::Vector3f camera_position, camera_lookat_direction;
        ExtractCameraParams(simulator->s_cam, camera_position, camera_lookat_direction);

        // std::cout << "Camera Position: " << camera_position.transpose() << std::endl;
        // std::cout << "Camera Look-At Direction: " << camera_lookat_direction.transpose() << std::endl;

        Eigen::Vector2f secondModelInitialForwardDirection = Eigen::Vector2f(-0.357791, 0.933791).normalized();
        Eigen::Vector2f lookatDirection = Eigen::Vector2f(camera_lookat_direction.x(), camera_lookat_direction.z()).normalized();
        float yawAngle = SignedAngleBetweenVectors(secondModelInitialForwardDirection, lookatDirection);
        cout << camera_position[0] << "," << camera_position[1] << "," << camera_position[2] << "," << yawAngle << "," << endl;


        int key = cv::waitKey(20);
        if (key != -1)
        {
            break;
        }
    }
    while(true)
    {
        char c;
        c = getchar();
        if(c == 'g')
        {
            generatePosition = true;
        }
        else if (c == 's')
        {
            
            double ex, ey, ez, lx, ly, lz;
            std::cin >> ex >> ey >> ez >> lx >> ly >> lz;
            auto mvm = pangolin::ModelViewLookAt(ex, ey, ez, lx, ly, lz, 0.0,
                                               1.0,
                                               0);
            simulator->s_cam.SetModelViewMatrix(mvm);
        }
        else if (c == 'm')
        {
            Eigen::Matrix4d mat = simulator->s_cam.GetModelViewMatrix();
            cout << "Mat" << endl;    
            cout << mat << endl;
            cout << "Mat Inv" << endl; 
            cout << mat.inverse() << endl;
        }
        else if (c == 'r')
        {
            simulator->s_cam.SetModelViewMatrix(reset_mvm);
        }
        else if(c == 'p')
        {
            Eigen::Matrix4d mat = simulator->s_cam.GetModelViewMatrix();
            Eigen::Vector3d cam_pos = mat.inverse().topRightCorner<3,1>();
            cout << cam_pos[0] << "," << cam_pos[1] << "," << cam_pos[2] << endl;
        }
        else if(c == 'b')
        {
            break;
        }
        
        if(generatePosition)
        {   

            cout << "Navigate to camera position and press Enter." << endl;
        
            getchar();
            getchar();
            Eigen::Matrix4d mat = simulator->s_cam.GetModelViewMatrix();
            Eigen::Vector3d eyePos = mat.inverse().topRightCorner<3,1>();
            cout << "Camera postion recoreded." << endl;
            cout << "Navigate to the position camera should look at and press Enter." << endl;
            getchar();

            mat = simulator->s_cam.GetModelViewMatrix();
            Eigen::Vector3d lookPos = mat.inverse().topRightCorner<3,1>();
            cout << "(ex, ey, ez, lx, ly, lz) = (" << eyePos[0] << "," << eyePos[1] << "," << eyePos[2] << "," << lookPos[0] << "," << lookPos[1] << "," << lookPos[2] << ")" << endl;
            generatePosition = false;
        }

        sleep(0.1);
    }

    
    // This line stop the exceuation of this thread until simulatorThread will finish.
    simulatorThread.join();
    delete simulator;
}
