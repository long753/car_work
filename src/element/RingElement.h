#pragma once
#include "../include/common.hpp"
#include "../include/detection.hpp"
#include "../include/stop_watch.hpp"
#include "../utils/algorithm.h"
#include "Findline.h"
#include <array>

#define NO_LINE_LEFT_X 4
#define NO_LINE_RIGHT_Y 315

enum RingType {
  RingNone = 0, // 未知类型
  RingRight, // 右入环岛
  RingLeft    // 左入环岛
  
};
enum class RingStep {
  None , // 
  PreEntering,//预入环
  Entering, // 入环
  Inside,   // 环中
  Exiting,  // 出环
  Finish    // 环任务结束
};

class Ring {
public:

  RingType ringType = RingType::RingNone; // 环岛类型
  RingStep ringStep = RingStep::None;     // 环岛处理阶段
  int repairline_straight = 30;//CarParams->rowCutBottom+20;    // 用于环补线的点（行号）
                    // 用于环补线的点（列号）


  std::array<int,30> ring_state_right; //保存找到右环的状态
  std::array<int,30> ring_state_near_right; //保存找到右环的状态
  std::array<int, 30> ring_state_left;
  std::array<int, 30> ring_state_near_left;

  int corner_up_index= 0;
  int corner_up_update_count = 0; //上拐点更新计数

  int corner_mid_index= 0;
  int corner_mid_update_count = 0; //中拐点更新计数

  int corner_down_index= 0;
  int corner_down_update_count = 0; //下拐点更新计数

  int corner_edge_index= 0;
  int corner_edge_update_count = 0; //上边沿点更新计数

  int corner_exit_index= 0;
  int corner_exit_update_count = 0; //出环拐点更新计数
  
  int exit_flag=0;
  int Entering_flag=0;

  int inside_flag=0;
  
  StopWatch Entering_time;
  StopWatch pre_entering_timer;
  StopWatch pre_entering_to_entering;
  StopWatch inside_to_exiting;
  StopWatch entering_to_inside;
  StopWatch reset_time;

  float start_angle;
  Ring();
  cv::Point corner_up_point {0,0};
  cv::Point corner_mid_point{0,0};
  cv::Point corner_down_point{0,0};
  cv::Point corner_edge_point {0,0};
  cv::Point corner_exit_point {0,0};

  StopWatch timer1;
  void reset();
  void circle_search(Findline & findline,float angle);
  int check_far_corner_right(Findline &findline);
  int check_near_corner_right(Findline & findline);
  int check_exit_corner_right(Findline & findline);
  int check_far_corner_left(Findline & findline);
  int check_near_corner_left(Findline & findline);
  int check_exit_corner_left(Findline & findline);
  int get_up_corner(Findline & findline);
  int get_mid_corner(Findline & findline);
  int get_down_corner(Findline & findline);
  int image_to_enter(Findline & findline);
  void repair_line_exit(Findline & findline);
  void repair_line_prev(Findline & fidnline);
  void repair_line(Findline & fidnline);
  int get_edge_corner(Findline & findline);
  bool straight_line_judge(std::vector<int> dir);

  auto get_draw_task(){
    return [*this](cv::Mat img){
      putText(img, to_string(static_cast<int>(ringStep)), Point(COLSIMAGE / 2 - 30, 10),
      cv::FONT_HERSHEY_TRIPLEX, 0.5, cv::Scalar(0, 0, 255), 0.5, CV_AA);
      circle(img,corner_up_point,3,Scalar(255, 255, 0), -1); //青色
      circle(img,corner_mid_point,3,Scalar(255, 0, 0), -1); // 蓝色
      circle(img,corner_down_point,3,Scalar(255, 0, 170), -1); // 紫色
      circle(img,corner_edge_point,3,Scalar(147, 20, 255), -1); //粉色
      circle(img,corner_exit_point,3,Scalar(170, 170, 255), -1); //浅粉色
    };
  }
};
