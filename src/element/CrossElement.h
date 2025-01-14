#pragma once
#include "../include/common.hpp"
#include "../include/detection.hpp"
#include "Findline.h"

enum class CrossroadType {
  None = 0,
  CrossroadLeft,     // 左斜入十字
  CrossroadRight,    // 右斜入十字
  CrossroadStraight, // 直入十字
};

class Cross {
public:
  cv::Point pointBreakLU;
  cv::Point pointBreakLD;
  cv::Point pointBreakRU;
  cv::Point pointBreakRD;
  int pointLU_index = 0;
  int pointLD_index = 0;
  int pointRU_index = 0;
  int pointRD_index = 0;
  uint16_t counterFild = 0;
  CrossroadType crossroadType = CrossroadType::None; // 十字道路类型
  void reset(void);
  bool cross_search(Findline &findline);  
  void repair_line(Findline & findline);
  void repair_line_tripoints(Findline & findline);

auto get_draw_task(){
    return [*this](cv::Mat img){
      putText(img, to_string(static_cast<int>(crossroadType)), Point(COLSIMAGE / 2 - 30, 10),
      cv::FONT_HERSHEY_TRIPLEX, 0.5, cv::Scalar(0, 0, 255), 0.5, CV_AA);
      circle(img,pointBreakLU,3,Scalar(255, 44, 20), -1);
      circle(img,pointBreakLD,3,Scalar(255, 170, 0), -1); //青色
      circle(img,pointBreakRU,3,Scalar(255, 0, 255), -1); // 粉色
      circle(img,pointBreakRD,3,Scalar(226, 43, 138), -1); //紫色
    };
  }

};
