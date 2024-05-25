#include "inference.h"
#include <opencv2/opencv.hpp>
#include <iostream>
#include <curl/curl.h>
#include <nlohmann/json.hpp>

Inference::Inference(const std::string &serverURL, const cv::Size &modelInputShape)
    : serverURL(serverURL), modelInputShape(modelInputShape)
{
}

size_t WriteCallback(void* contents, size_t size, size_t nmemb, void* userp)
{
    ((std::string*)userp)->append((char*)contents, size * nmemb);
    return size * nmemb;
}

std::vector<Detection> Inference::runInference(const cv::Mat &input)
{
    std::vector<Detection> detections;

    std::vector<uchar> buf;
    cv::imencode(".jpg", input, buf);
    auto* enc_msg = reinterpret_cast<unsigned char*>(buf.data());
    
    CURL* curl;
    CURLcode res;
    curl_global_init(CURL_GLOBAL_ALL);
    curl = curl_easy_init();
    
    if(curl) {
        struct curl_slist *headers = NULL;
        headers = curl_slist_append(headers, "Content-Type: application/octet-stream");
        
        std::string readBuffer;
        
        curl_easy_setopt(curl, CURLOPT_URL, serverURL.c_str());
        curl_easy_setopt(curl, CURLOPT_POST, 1L);
        curl_easy_setopt(curl, CURLOPT_HTTPHEADER, headers);
        curl_easy_setopt(curl, CURLOPT_POSTFIELDS, enc_msg);
        curl_easy_setopt(curl, CURLOPT_POSTFIELDSIZE, buf.size());
        curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, WriteCallback);
        curl_easy_setopt(curl, CURLOPT_WRITEDATA, &readBuffer);
        
        res = curl_easy_perform(curl);
        
        if(res != CURLE_OK) {
            fprintf(stderr, "curl_easy_perform() failed: %s\n", curl_easy_strerror(res));
            std::cout << "curl_easy_perform() failed:\n" << curl_easy_strerror(res) << std::endl;
        } else {
            auto jsonData = nlohmann::json::parse(readBuffer);
            for (const auto &det : jsonData) {
                Detection detection;
                detection.class_id = det["class_id"].get<int>();
                detection.className = det["className"].get<std::string>();
                detection.confidence = det["confidence"].get<float>();
                detection.box = cv::Rect(cv::Point2i(det["box"][0].get<int>(), det["box"][1].get<int>()),
                                         cv::Point2i(det["box"][2].get<int>(), det["box"][3].get<int>()));
                
                detections.push_back(detection);
            }
        }
        
        curl_easy_cleanup(curl);
    }
    
    curl_global_cleanup();
    
    return detections;
}
