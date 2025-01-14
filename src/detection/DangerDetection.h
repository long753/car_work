#pragma once
#include "../element/Findline.h"
#include "../include/common.hpp"
#include "../include/detection.hpp"

enum class DangerStep {
  None,   // AI检测
  Enable, // 使能（标志识别成功）
  Cruise    // 处理
};

class Danger {

public:
  /**
   * @brief 危险区AI识别与路径规划处理
   *
   * @param Findline 赛道识别结果
   * @param predict AI检测结果
   * @return true
   * @return false
   */
  enum class BarrierType{ //障碍物类型
    None,
    LeftCone,
    RightCone,
    LeftBlock,
    RightBlock
  };

  enum class BlockStep{
    NoBlock,
    BeforeBlock,
    AfterBlock
  };
  bool process(Findline &track, vector<PredictResult> predict);
  bool left_cone_process(Findline &track, vector<PredictResult> & predict);
  bool right_cone_process(Findline &track, vector<PredictResult> & predict);
  bool block_process(Findline &track, vector<PredictResult> &predict);

  BarrierType barriertype = BarrierType::None;
  DangerStep dangerstep = DangerStep::None;

  /**
   * @brief 图像绘制禁行区识别结果
   *
   * @param img 需要叠加显示的图像
   */
  void drawImage(Mat &img);

  auto get_draw_task() {
    return [*this](Mat img) mutable {
      if (enable) {
        putText(img, "[2] DANGER - ENABLE", Point(COLSIMAGE / 2 - 30, 20),
                cv::FONT_HERSHEY_TRIPLEX, 0.5, cv::Scalar(0,0, 255), 1, CV_AA);
        cv::Rect rect(nearest_object.x, nearest_object.y, nearest_object.width,
                      nearest_object.height);
        cv::rectangle(img, rect, cv::Scalar(0, 0, 255), 1);
        if (cone_left){
            putText(img,"left_cone",Point(COLSIMAGE/2-30,40),cv::FONT_HERSHEY_TRIPLEX, 0.5, cv::Scalar(0, 0, 255), 1, CV_AA);
        }else{
            putText(img,"right_cone",Point(COLSIMAGE/2-30,40),cv::FONT_HERSHEY_TRIPLEX, 0.5, cv::Scalar(0, 0, 255), 1, CV_AA);
        }
      }
    };
  }

private:
  bool enable = false;     // 场景检测使能标志
  bool next_obj_flag = true;

  PredictResult nearest_object; // 最近的物体
  PredictResult next_object; // 下一个最近的物体
  uint16_t counterRec = 0; 
  uint16_t counterSession = 0;  // 图像场次计数器
  uint16_t cone_counter = 0;
  uint16_t counterImmunity = 0;
  uint16_t block_count = 0; // Block场次计数
  uint16_t count_left_tendency = 0;
  uint16_t count_right_tendency = 0;
  bool cone_left = false;
  bool block_state= false; //Block是否被检测到
  BlockStep block_step; // 遇到障碍的状态
  bool first_cone = false; //第一个锥筒
  bool second_cone = false;
  uint16_t last_cone_row = 0;
  int16_t gap_cone_distance = 0;


  /**
   * @brief 缩减优化车道线（双车道→单车道）
   *
   * @param track
   * @param left
   */
  void curtailTracking(Findline &track, bool left);
};
