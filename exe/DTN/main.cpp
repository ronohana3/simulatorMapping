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

// TODO: do it in a proper way with thread safe mechanism
void ValidateBoxInBackground(Inference *inf, Mat &frame, Rect box, bool *isBusy, bool *foundDrone)
{
    if ((*isBusy)) return;
    *isBusy = true;
    std::thread([&, inf, frame, box, isBusy, foundDrone] {
        std::vector<Detection> output = inf->runInference(frame);
        int detections = output.size();
        if (detections > 0)
        {
            cv::imshow("Test", frame);
            sleep(2);
            Detection detection = output[0];
            for (int j=1; j<detections; j++)
                if(output[j].confidence > detection.confidence)
                    detection = output[j];
            if(detection.confidence > 0.55 && detection.box.contains(Point2i(box.x + box.width/2, box.y + box.height/2)))
                *foundDrone = true;
            else
                *foundDrone = false;

            
        }
        else
            *foundDrone = false;
        
        *isBusy = false;
        
    }).detach();
}

int main(int argc, char **argv) {    
    
    Simulator *simulator = new Simulator();

    auto simulatorThread = simulator->run();
    while (!simulator->isReady()) { // wait for the 3D model to load
        usleep(1000);
    }
    
    // To generate new starting position use NRSimulator.exe and press g. 
    auto mvm = pangolin::ModelViewLookAt(-0.00131965,1.00848,-1.88549,-0.00162125,1.00623,-1.3855, 0, 1.0, 0);
    simulator->s_cam.SetModelViewMatrix(mvm);

    
    Inference inf("http://localhost:5000/inference", cv::Size(640, 640));
    Mat frame, grayFrame;
    Rect boundingBox;
    CMT tracker = CMT();
    bool isTracking = false;
    int frameCount = 1;

    CamParam cameraParameters = {
        .width = 640,
        .height = 480,
        .fx = 619.6508392029048,
        .fy = 618.5264705043031,
        .cx = 321.88905699582324,
        .cy = 243.8086797913814
    };

    DroneController drone(simulator, cameraParameters);

    namedWindow("Stream");
    // namedWindow("Test");

    bool validationInProcess = false;
    bool foundDrone = false;
    Mat validationFrame;


    int fourcc = VideoWriter::fourcc('M', 'J', 'P', 'G');
    string destVideoPath = "/home/rbdlab/Projects/IronDrone/simulatorVideos/sim_output.avi";
    VideoWriter writer;
    writer.open(destVideoPath, fourcc, 10, Size(640, 480));

    while (true) 
    {
        drone.getFrame(frame);

        if(frame.empty())
        {
            break;
        }

        cvtColor(frame, grayFrame, COLOR_BGR2GRAY);

        std::cout << "Start proccesing frame "<< frame.size() <<" #" << frameCount << std::endl;
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
                    if(output[j].confidence > detection.confidence)
                        detection = output[j];
                if(detection.confidence > 0.55)
                {
                    boundingBox = detection.box;
                    tracker.initialize(grayFrame, boundingBox);
                    foundDrone = true;
                    isTracking = true;
                    std::cout << "Found drone with confidence=" << detection.confidence << " in boundingBox=" << boundingBox << std::endl;
                }         
            }
        }
        else
        {
            tracker.processFrame(grayFrame);

            if(0.05*tracker.init_points_active_size < tracker.points_active.size() && tracker.points_active.size() > 10 && foundDrone)
            {
                
                // if(!validationInProcess)
                // {
                //     frame.copyTo(validationFrame);
                //     ValidateBoxInBackground(&inf, validationFrame, tracker.bb_rot.boundingRect(), &validationInProcess, &foundDrone);
                // }

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
                
                auto trackingBox = tracker.bb_rot.boundingRect();
                std::thread([&, trackingBox] {
                    drone.navigateToBox(trackingBox);
                }).join();
            }
            else
            {
                std::cout << "Lost tracking" << std::endl;
                isTracking = false;
                tracker = CMT();

            }
        }

        frameCount++;
        cv::imshow("Stream", frame);

        if (writer.isOpened())
            writer.write(frame);

        int key = cv::waitKey(50);
        
        if (key != -1)
        {
            break;
        }
    }

    destroyAllWindows();

    writer.release();

    frame.release();
    grayFrame.release();
    validationFrame.release();
    // This line stop the exceuation of this thread until simulatorThread will finish.
    simulatorThread.join();
    delete simulator;
}