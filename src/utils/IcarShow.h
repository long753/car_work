#pragma once 
#include "../include/common.hpp"
#include "../include/detection.hpp"
#include "../utils/ThreadUtil.hpp"
#include "SingletonHolder.hpp"
#include <thread>
#include <vector>
#include <atomic>
#include <functional>
#include "../vision/mapping.h"

class IcarShow : public SingletonHolder<IcarShow>{
private:
    bool enable = false; // 显示窗口使能
    int sizeWindow = 1;  // 窗口数量
    cv::Mat imgShow;     // 窗口图像

public:
    std::atomic<bool> show_enable;
    std::thread debug_window_thread;
    ThreadSafeQueue<cv::Mat> debug_window1;
    ThreadSafeQueue<cv::Mat> debug_window2;
    std::mutex debug_image1_mutex;
    std::mutex debug_image2_mutex;
    cv::Mat debug_image1;
    cv::Mat debug_image2;
    std::string now_scene = "Track";


    IcarShow();
    
    ~IcarShow();
    void show_init();
    void ai_draw_result(cv::Mat &img, std::vector<PredictResult> results);
    void run();

private:
    cv::Scalar getCvcolor(int index);
    void display();
    void set_new_window(int index, cv::Mat img, std::string name);
};

inline static auto & icarShow = IcarShow::get_instance();
