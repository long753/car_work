#pragma once 
#include "common.hpp"
#include "../element/Findline.h" 
#include "../include/detection.hpp"

class Bridge
{
public:
    bool process(Findline &track, vector<PredictResult> predict);

    /**
     * @brief 识别结果图像绘制
     *
     */
    void drawImage(Mat &image,Findline track);
    bool bridgeEnable = false;   // 桥区域使能标志


private:
    uint16_t counterSession = 0; // 图像场次计数器
    uint16_t counterRec = 0;     // 加油站标志检测计数器
    uint16_t counterRec_all=0;
};
