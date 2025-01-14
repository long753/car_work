#pragma once
#include "../include/detection.hpp"
#include "../utils/ThreadUtil.hpp"
#include "../utils/IcarShow.h"
#include <vector>
#include <thread>
#include <atomic>
#include <queue>
#include <mutex>
#include <condition_variable>

class AiProcess {
private:
std::atomic<bool> stop_flag = false;
std::condition_variable ai_cond;
int detection_num =2;

public:
int thread_free_nums = 1;
std::mutex ai_mutex;
std::mutex result_mutex;
std::vector<std::jthread> threads;
std::vector<PredictResult> results;
std::queue<cv::Mat> image_queue;


void ai_process_start();
AiProcess();
~AiProcess();

void commit_ai_task(cv::Mat img);



};