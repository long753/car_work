#pragma once

#include "../utils/SingletonHolder.hpp"
#include "../include/json.hpp"
#include <string>
#include <memory>
#include <iostream>
#include <fstream>


struct Params {
    float speedLow = 1.0;       // 智能车最低速
    float speedHigh = 1.0;      // 智能车最高速
    float speedDown = 0.6;      // 特殊区域降速速度
    float ring_p = 1.4;
    float ring_d = 2.5;
    float danger_p= 1.0;
    int rescue_angle_diff = 60;
    int qianzhan=132;
    int danger_qianzhan = 200;
    int danger_anglesize = 20;
    bool accelerate = false;
    float rescue_Cruise_speed = 1;
    bool danger_col_debug = true;
    bool PWM_show = false;
    int bridge_qianzhan = 180;
    float danger_d = 0.4;
    int parking_count = 80; // 停车计数
    int parking_time = 10000; // 停车计时
    int imgh = 200;
    float turnP = 3.5;          // 一阶比例系数：转弯控制量
    float turnD = 3.5;          // 一阶微分系数：转弯控制量
    bool debug = false;         // 调试模式使能
    bool key_start_end = true;    //按键控制启动停止
    bool inference_time_view = false;//查看最占用时间部分的时间
    int rescue_endpoint_row = 0;//贝塞尔拉线终点行，越大拐角越大
    int rescue_qianzhan = 180;
    bool rescue_direction = true;//true是右，即evil/thief,false是左，即patient/tumble
    bool rescue_key= false;
    int rescue_conenum = 5;
    float bridge_score = 0.9;
    int bridge_count_exit = 40;
    int bridge_count_enter = 10;
    uint16_t rowCutUp = 10;     // 图像顶部切行
    uint16_t edgeCutup = 20;    //赛道边沿计算切行
    uint16_t rowCutBottom = 10; // 图像顶部切行
    bool bridge = true;                                // 坡道区使能
    bool danger = true;                                // 危险区使能
    bool rescue = true;                                // 救援区使能
    bool racing = true;                                // 追逐区使能
    bool parking = true;                               // 停车区使能
    bool ring = true;                                  // 环岛使能
    bool cross = true;                                 // 十字道路使能
    float score = 0.5;                                 // AI检测置信度
    int rescue_enter_count = 5;
    float danger_speed = 1.5;
    float accelerate_speed = 3.0;
    int rescue_enterstop_count = 10;
    int danger_slowcount = 10;
    int danger_blockcount = 5;
    int corner_up_update_count = 8;
    int corner_mid_update_count =12;
    int corner_down_update_count = 0;
    int corner_exit_update_count =30;
    bool ring_use_gyro = true;
    float ring_exit_angle = 300; // 退出角度
    bool rescue_wrong_assert = false;
    int rescue_labelframe = 2;
    float racing_speed=1.0;
    int racing_anglesize=30;
    bool racing_key=false;
    int prop1Type=2;        // prop1 对应的指定类型
    int prop2Type=3;        // prop2 对应的指定类型
    int prop3Type=1;        // prop3 对应的指定类型

    NLOHMANN_DEFINE_TYPE_INTRUSIVE(
        Params, speedLow, speedHigh, speedDown,danger_p,qianzhan,danger_qianzhan,ring_p,ring_d,
        danger_anglesize,PWM_show,parking_count,parking_time,imgh,turnP, turnD, debug, key_start_end,inference_time_view,rescue_endpoint_row,
        rescue_qianzhan,rescue_direction,rescue_key,rescue_conenum,rescue_angle_diff,accelerate,
        rowCutUp,edgeCutup, rowCutBottom, bridge, danger,rescue,danger_col_debug,rescue_Cruise_speed,
        racing, parking, ring, cross, score,danger_d,rescue_enter_count,bridge_count_enter,rescue_enterstop_count,
        danger_speed,bridge_qianzhan,danger_slowcount,danger_blockcount,rescue_labelframe,
        bridge_count_exit,bridge_score,corner_up_update_count,rescue_wrong_assert,accelerate_speed,
        corner_mid_update_count,corner_down_update_count,corner_exit_update_count,ring_exit_angle,
        ring_use_gyro,racing_speed,racing_anglesize,racing_key,prop1Type,prop2Type,prop3Type); // 添加构造函数
    
  };
struct RingParams{
  bool using_fix_angle = false;
  float left_enter_pwm = 1000;
  float right_enter_pwm= 690 ;
  float left_exit_pwm=1000;
  float right_exit_pwm=690;
  bool using_fix_angle_exit=true;
  int entering_place_left = 70;
  int entering_place_right= 70;
  float left_num45=0.4583;
  float left_num4=0.2;
  float left_num5=0.2;
  float right_num45=0.4583;
  float right_num4=0.3;
  float right_num5=0.3;
  int near_right11=10;
  int near_right12=35;
  int near_right2=20;
  int near_right3=30;
  int near_right4=10;
  int near_right5=15;
  int near_right6=7;
  int near_left11=10;
  int near_left12=35;
  int near_left2=15;
  int near_left3=30;
  int near_left4=10;
  int near_left5=15;
  int near_left6=7;
  bool show_debug = false;
  bool video_rec = true;
  int racing_exit_time = 500;
  int racing_danger_angle_left = 20;
  int racing_danger_angle_right = 20;
  float danger_pingbi_row = 0.2;
  bool danger_correct = true;
  float danger_block_pingbi = 0.2;
  int danger_block_angle = 50;
  float rescue_exit_speed = 1.2;
  NLOHMANN_DEFINE_TYPE_INTRUSIVE(
        RingParams,using_fix_angle,left_enter_pwm,right_enter_pwm,left_exit_pwm,right_exit_pwm,using_fix_angle_exit,
        danger_pingbi_row,danger_block_pingbi,danger_block_angle,rescue_exit_speed,
        entering_place_left,entering_place_right,left_num45,left_num4,left_num5,right_num45,right_num4,right_num5,near_right11,near_right12,
        near_right2,near_right3,near_right4, near_right5,near_right6,near_left11, near_left12,near_left2,near_left3,near_left4,near_left5,near_left6
        ,show_debug,video_rec,racing_exit_time,racing_danger_angle_left,racing_danger_angle_right); // 添加构造函数
    
  
};

struct AIParams{
  int racing_spy_exit_time = 10;
  float danger_pingbi_row = 0.25;
  bool danger_correct = false;
  float danger_block_pingbi= 0.2;
  int danger_block_angle = 50;
  float rescue_exit_speed =1.5;
  bool bridge_change_scheme = false;
  float bridge_block_row = 0.2;
  int bridge_count_enter_changed = 10;
  int racing_cruise_frame = 15;
  int danger_exit_frame = 50;
  NLOHMANN_DEFINE_TYPE_INTRUSIVE(
        AIParams,racing_spy_exit_time,danger_pingbi_row,
        danger_correct,danger_block_pingbi,danger_block_angle,rescue_exit_speed ,bridge_change_scheme,bridge_block_row,bridge_count_enter_changed,racing_cruise_frame,danger_exit_frame); // 添加构造函数
    
  
};

class CarConfig : public SingletonHolder<CarConfig>
{
public:
std::shared_ptr<Params> shared_params;
void load();
void print_params();
CarConfig();
~CarConfig();

};

class RingConfig: public SingletonHolder<RingConfig>
{
  public:
  std::shared_ptr<RingParams> ring_params;
  void ring_config_load();
  RingConfig();

};

class AIConfig : public SingletonHolder<AIConfig>{
 public:
std::shared_ptr<AIParams> ai_params;
 void ai_config_load();
 AIConfig();
  
};
//Findline 保存着巨大的类使用指针放置爆栈
inline static auto& CarData = CarConfig::get_instance();
inline static auto& CarParams = CarData.shared_params;
inline static auto& RingData = RingConfig::get_instance();
inline static auto& ring_params = RingData.ring_params;
inline static auto& AIData = AIConfig::get_instance();
inline static auto& ai_params = AIData.ai_params;





