//
// Created by tzuk on 6/4/23.
//
#include <matplotlibcpp.h>
#include "simulator/simulator.h"
#include "navigation/RoomExit.h"
#include "include/Auxiliary.h"

int main(int argc, char **argv) {    
    std::string settingPath = Auxiliary::GetGeneralSettingsPath();
    std::ifstream programData(settingPath);
    nlohmann::json data;
    programData >> data;
    programData.close();
    std::string configPath = data["DroneYamlPathSlam"];
    std::string VocabularyPath = data["VocabularyPath"];
    std::string modelTextureNameToAlignTo = data["modelTextureNameToAlignTo"];
    std::string model_path = data["modelPath"];
    std::string map_input_dir = data["mapInputDir"];
    bool trackImages = data["trackImages"];
    double movementFactor = data["movementFactor"];
    Simulator simulator(configPath, model_path, modelTextureNameToAlignTo, trackImages, false, map_input_dir, false,
                        "", movementFactor,VocabularyPath);
    simulator.setTrack(false);
    auto simulatorThread = simulator.run();
    while (!simulator.isReady()) { // wait for the 3D model to load
        usleep(1000);
    }

    // simulator.setTrack(true);
    // int angle = 5;
    // int lastAngle = 360;
    // for (int i = 0; i < std::ceil(lastAngle / angle); i++) {
        
    //     std::string c = "left 0.2";
    //     simulator.command(c);
        
    //     c = "right 0.4";
    //     simulator.command(c);
        
    //     c = "left 0.2";
    //     simulator.command(c);
        
    //     c = "cw " + std::to_string(angle);
    //     simulator.command(c);
        
    //     sleep(1);
    // }


    // TEST RON
    pangolin::OpenGlMatrix reset_mvm = simulator.s_cam.GetModelViewMatrix();
    bool generatePosition = false;

    while(true)
    {
        char c;
        std::cin >> c;
        if(c == 'g')
        {
            generatePosition = true;
        }
        if (c == 's')
        {
            
            double ex, ey, ez, lx, ly, lz;
            std::cin >> ex >> ey >> ez >> lx >> ly >> lz;
            auto mvm = pangolin::ModelViewLookAt(ex, ey, ez, lx, ly, lz, 0.0,
                                               1.0,
                                               0);
            simulator.s_cam.SetModelViewMatrix(mvm);
        }
        else if (c == 'm')
        {
            Eigen::Matrix4d mat = simulator.s_cam.GetModelViewMatrix();
            cout << "Mat" << endl;    
            cout << mat << endl;
            cout << "Mat Inv" << endl; 
            cout << mat.inverse() << endl;
        }
        else if (c == 'm')
        {
            Eigen::Matrix4d mat = simulator.s_cam.GetModelViewMatrix();
            cout << "Mat" << endl;    
            cout << mat << endl;
            cout << "Mat Inv" << endl; 
            cout << mat.inverse() << endl;
        }
        else if (c == 'r')
        {
            simulator.s_cam.SetModelViewMatrix(reset_mvm);
        }
        else if(c == 'p')
        {
            Eigen::Matrix4d mat = simulator.s_cam.GetModelViewMatrix();
            Eigen::Vector3d cam_pos = mat.inverse().topRightCorner<3,1>();
            cout << "x=" << cam_pos[0] << " y=" << cam_pos[1] << " z=" << cam_pos[2] << endl;
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
            Eigen::Matrix4d mat = simulator.s_cam.GetModelViewMatrix();
            Eigen::Vector3d eyePos = mat.inverse().topRightCorner<3,1>();
            cout << "Camera postion recoreded." << endl;
            cout << "Navigate to the position camera should look at and press Enter." << endl;
            getchar();

            mat = simulator.s_cam.GetModelViewMatrix();
            Eigen::Vector3d lookPos = mat.inverse().topRightCorner<3,1>();
            cout << "(ex, ey, ez, lx, ly, lz) = (" << eyePos[0] << "," << eyePos[1] << "," << eyePos[2] << "," << lookPos[0] << "," << lookPos[1] << "," << lookPos[2] << ")" << endl;
            generatePosition = false;
        }
        sleep(0.1);
    }

    // simulator.setTrack(false);
    
    // sleep(2);
    // auto scanMap = simulator.getCurrentMap();
    // std::vector<Eigen::Vector3d> eigenData;
    // for (auto &mp: scanMap) {
    //     if (mp != nullptr && !mp->isBad()) {
    //         auto vector = ORB_SLAM2::Converter::toVector3d(mp->GetWorldPos());
    //         eigenData.emplace_back(vector);
    //     }
    // }

    // for (auto vec: eigenData) {
    //     std::cout << vec << std::endl;
    // }

    // This line stop the exceuation of this thread until simulatorThread will finish.
    simulatorThread.join();
}
