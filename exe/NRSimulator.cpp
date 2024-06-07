//
// Created by tzuk on 6/4/23.
//
#include <matplotlibcpp.h>
#include "simulator/simulator.h"
#include "navigation/RoomExit.h"
#include "include/Auxiliary.h"

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
        Eigen::Matrix4d mat = simulator->s_cam.GetModelViewMatrix();
        Eigen::Vector3d cam_pos = mat.inverse().topRightCorner<3,1>();
        cout << cam_pos[0] << "," << cam_pos[1] << "," << cam_pos[2] << "," << endl;

        int key = cv::waitKey(200);
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
