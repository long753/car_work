#pragma once
#include "ImageProcess.h"
#include "../utils/IcarShow.h"
#include <filesystem>
#include <vector>
#include <future>
#include <memory>
#include <chrono>
#include <atomic>
#include "../driver/camera_driver.h"
#include "../utils/ThreadUtil.hpp"
#include "../utils/videorecord.hpp"
class ImageCapture{

private:
//以后搞无锁的 但是无锁效率不一定比有锁高


std::atomic<bool> capture_flag{true};
std::atomic<int> now_image_index{0};
int next_image_index{0};
public:
cv::VideoCapture capture;

cv::Mat rgb_image;
std::mutex image_capture_mutex;
std::atomic<bool> image_update_flag{false};
ImageProcess image_process;
std::jthread camera_thread;

//初始化相机
bool camera_init();
//启动相机采集(多线程)
void raw_image_catch();
void image_show();
void run();
~ImageCapture();

};