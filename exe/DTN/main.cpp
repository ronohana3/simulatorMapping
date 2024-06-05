#include <iostream>
#include <vector>
#include <filesystem>

#include <opencv2/opencv.hpp>

#include "Tracking/CMT.h"
#include "Detection/inference.h"
#include "Controllers/DroneController.hpp"

#include "simulator/simulator.h"
#include "include/Auxiliary.h"

using namespace cv;
using namespace std;
using namespace cmt;

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
    
    // To generate new starting position use NRSimulator.exe and press g. 
    auto mvm = pangolin::ModelViewLookAt(1.58662,0.557122,-0.751696,0.89928,0.555703,-0.812255, 0, 1.0, 0);
    // auto mvm = pangolin::ModelViewLookAt(-3.8966, 0.974479 ,3.55261 ,-3.566 ,0.98659, 1.03574, 0, 1.0, 0);
    simulator.s_cam.SetModelViewMatrix(mvm);


    // our simulator
    
    Inference inf("http://localhost:5000/inference", cv::Size(640, 640));
    Mat frame, grayFrame;
    Rect boundingBox;
    CMT tracker = CMT();
    bool isTracking = false;
    int i = 1;

    // CamParam cameraParameters = {
    //     .width = 640,
    //     .height = 480,
    //     .fx = 619.6508392029048,
    //     .fy = 618.5264705043031,
    //     .cx = 321.88905699582324,
    //     .cy = 243.8086797913814
    // };
    CamParam cameraParameters = {
        .width = 640,
        .height = 480,
        .fx = 140,
        .fy = 140,
        .cx = 320,
        .cy = 240
    };
    DroneController drone(&simulator, cameraParameters);

    namedWindow("Stream");

    // int fourcc = VideoWriter::fourcc('M', 'J', 'P', 'G');
    // string destVideoPath = "/home/ron/Project/simulatorVideos/sim_output.avi";
    // VideoWriter writer;
    // writer.open(destVideoPath, fourcc, 10, Size(640, 480));
    // string destFramePath = "/home/ron/Project/simulatorVideos/frame_";

    while (true) 
    {
        drone.getFrame(frame);

        if(frame.empty())
        {
            break;
        }

        cvtColor(frame, grayFrame, COLOR_BGR2GRAY);

        std::cout << "Start proccesing frame "<< frame.size() <<" #" << i << std::endl;
        std::cout << "Tracking: " << (isTracking ? "yes" : "no" ) << std::endl;

        // Detection
        if (!isTracking)
        {
            std::vector<Detection> output = inf.runInference(frame);
            int detections = output.size();
            std::cout << "Number of detections: " << detections << std::endl;
            if (detections > 0)
            {
                Detection detection = output[0];
                for (int j=1; j<detections; j++)
                    if(output[i].confidence > detection.confidence)
                        detection = output[j];
                if(detection.confidence > 0.55)
                {
                    boundingBox = detection.box;
                    tracker.initialize(grayFrame, boundingBox);
                    isTracking = true;
                    std::cout << "Found drone with confidence=" << detection.confidence << " in boundingBox=" << boundingBox << std::endl;
                }
                        
            }
        }
        else
        {
            tracker.processFrame(grayFrame);

            if(0.05*tracker.init_points_active_size < tracker.points_active.size() && tracker.points_active.size() > 10)
            {
                Point2f vertices[4];
                tracker.bb_rot.points(vertices);
                for(size_t i = 0; i < tracker.points_active.size(); i++)
                {
                    circle(frame, tracker.points_active[i], 2, Scalar(255,0,0));
                }
                for (int i = 0; i < 4; i++)
                {
                    line(frame, vertices[i], vertices[(i+1)%4], Scalar(255,0,0), 2);
                }

                std::cout << "Active points: " << tracker.points_active.size() << std::endl;
                
                drone.navigateToBox(tracker.bb_rot.boundingRect());
                
            }
            else
            {
                std::cout << "Lost tracking" << std::endl;
                isTracking = false;
                tracker = CMT();

            }
        }

        i++;
        cv::imshow("Stream", frame);

        // if (writer.isOpened())
        //     writer.write(frame);
        // imwrite(destFramePath + to_string(i) + ".jpg", frame);

        int key = cv::waitKey(1);
        
        if (key != -1)
        {
            break;
        }
    }

    destroyAllWindows();

    // writer.release();

    // This line stop the exceuation of this thread until simulatorThread will finish.
    simulatorThread.join();
}