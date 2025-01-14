#pragma once
#include "../include/common.hpp"
#include "../element/Findline.h"
#include <cmath>
#include <array>
#include <memory>
using namespace cv;
using namespace std;
class Control
{
public:
    int speed_up=0;
    uint16_t servoPwm = PWMSERVOMID; //舵机PWM
    float motorSpeed = CarParams->speedHigh; // 电机初始速度 
    std::array<float,4> errorFiFo{0};
    std::array<float,4> errorDtFiFo{0};
    std::array<float,20> err_blur{0};
    float lastError = 0;
    float qianzhan = CarParams->qianzhan;
    float turnP;
    float turnD;
    float racing_speed=CarParams->racing_speed;

    void servo_control(float error,Findline &findline,Scene roadScence);
    void motor_control(Scene scene);
    float error_cal(Findline & findline);

    Control();
    ~Control() = default;

};


