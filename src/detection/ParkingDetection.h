#pragma once 
#include "../include/common.hpp"
#include "../include/detection.hpp"
#include "../element/Findline.h"

class Parking
{
private:
    /**
     * @brief 场景状态
     *
     */
    enum Step
    {
        init = 0, // 初始化屏蔽
        det,      // AI标识检测
        enable,   // 场景使能
        stop      // 准备停车
    };
    Step step = Step::init; // 场景状态
    uint16_t countRec = 0;  // AI场景识别计数器
    uint16_t countSes = 0;  // 场次计数器

public:
    uint16_t countExit = 0; // 程序退出计数器
    StopWatch parking_timer;
    bool park = false;      // 停车标志
    /**
     * @brief 停车区AI识别与路径规划处理
     *
     * @param predict AI检测结果
     * @return true
     * @return false
     */
    bool process(vector<PredictResult> predict);
   
    /**
     * @brief 图像绘制禁行区识别结果
     *
     * @param img 需要叠加显示的图像
     */
    void drawImage(Mat &img);

    auto get_draw_task(){
        return [*this](cv::Mat img){
            if (step == Step::enable){
            putText(img, "[5] PARK - ENABLE", Point(COLSIMAGE / 2 - 30, 10), cv::FONT_HERSHEY_TRIPLEX, 0.3, cv::Scalar(0, 255, 0), 1, CV_AA);
            }
        };
    }
};