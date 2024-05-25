#ifndef INFERENCE_H
#define INFERENCE_H

#include <opencv2/opencv.hpp>
#include <string>
#include <vector>

struct Detection
{
    int class_id{0};
    std::string className{};
    float confidence{0.0};
    cv::Rect box{};
};

class Inference
{
public:
    Inference(const std::string &serverURL, const cv::Size &modelInputShape = {640, 640});
    std::vector<Detection> runInference(const cv::Mat &input);

private:
    std::string serverURL;
    std::vector<std::string> classes{"drone"};
    cv::Size modelInputShape;
};

#endif // INFERENCE_H
