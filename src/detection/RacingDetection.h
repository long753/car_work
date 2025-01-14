#pragma once
#include "../element/Findline.h"
#include "../include/common.hpp"
#include "../include/detection.hpp"

class Racing {
public:
  bool carStoping = false; // 停车标志
  int race_step=0;
  bool racing_exit_status = false;
  StopWatch exit_timer;
  /**
   * @brief 检测与路径规划
   *
   * @param track 赛道识别结果
   * @param detection AI检测结果
   */
  bool process(Findline &track, vector<PredictResult> predicts);

  /**
   * @brief 图像绘制禁行区识别结果
   *
   * @param img 需要叠加显示的图像
   */
  void drawImage(Mat &img);

  auto get_draw_task() {
    return [*this](cv::Mat img) {
      if (typeRace == TypeRace::Spy) {
        switch (stepSpy) {
        case StepSpy::Det:
          putText(img, "[4] RACE - SPY - Det", Point(COLSIMAGE / 2 - 50, 10),
                  cv::FONT_HERSHEY_TRIPLEX, 0.3, cv::Scalar(0, 255, 0), 1,
                  CV_AA);
          break;
        case StepSpy::Bypass:
          putText(img, "[4] RACE - SPY - Bypass", Point(COLSIMAGE / 2 - 50, 10),
                  cv::FONT_HERSHEY_TRIPLEX, 0.3, cv::Scalar(0, 255, 0), 1,
                  CV_AA);
          break;
        case StepSpy::Inside:
          putText(img, "[4] RACE - SPY - Inside", Point(COLSIMAGE / 2 - 50, 10),
                  cv::FONT_HERSHEY_TRIPLEX, 0.3, cv::Scalar(0, 255, 0), 1,
                  CV_AA);
          break;
        case StepSpy::Resist:
          putText(img, "[4] RACE - SPY - Resist", Point(COLSIMAGE / 2 - 50, 10),
                  cv::FONT_HERSHEY_TRIPLEX, 0.3, cv::Scalar(0, 255, 0), 1,
                  CV_AA);
          break;
        default:
          break;
        }
      } else if (typeRace == TypeRace::Danger) {
        putText(img, "[4] RACE - DANGER", Point(COLSIMAGE / 2 - 30, 10),
                cv::FONT_HERSHEY_TRIPLEX, 0.3, cv::Scalar(0, 255, 0), 1, CV_AA);

      } else if (typeRace == TypeRace::Safe) {
        putText(img, "[4] RACE - Safe", Point(COLSIMAGE / 2 - 30, 10),
                cv::FONT_HERSHEY_TRIPLEX, 0.3, cv::Scalar(0, 255, 0), 1, CV_AA);
      }

      putText(img, to_string(_index), Point(COLSIMAGE / 2 - 10, 40),
              cv::FONT_HERSHEY_TRIPLEX, 0.3, cv::Scalar(0, 255, 0), 1, CV_AA);
    };
  }


  int counterSes[3] = {0, 0, 0}; // 图像场次计数器
  int counterRec[3] = {0, 0, 0}; // 标志检测计数器
  bool sideLeft = true;          // AI标识左右侧使能
  int _index = 0;
  /**
   * @brief 场景类型
   *
   */
  enum TypeRace {
    None = 0, // AI检测
    Safe,     // 普通车辆
    Spy,      // 嫌疑车辆
    Danger,    // 危险车辆
    Prop       //道具车
  };
  TypeRace typeRace = TypeRace::None; // 场景类型
  

  /**
   * @brief 嫌疑车辆逼停阶段
   *
   */
  enum StepSpy {
    Det = 0, // AI检测
    Bypass,  // 车辆绕行
    Inside,  // 变道
    Resist   // 阻挡
  };

  StepSpy stepSpy = StepSpy::Det; // 嫌疑车辆逼停阶段

  /**
   * @brief 检索AI场景类型
   *
   */
  void searchTypeRace(Findline &track,vector<PredictResult> predicts);
  /**
   * @brief 检索目标图像坐标
   *
   * @param predicts AI识别结果
   * @param index 检索序号
   * @return PredictResult
   */
  PredictResult searchSign(vector<PredictResult> predicts, int index);

  /**
   * @brief 缩减优化车道线（双车道→单车道）
   *
   * @param track
   * @param left
   */
  void curtailTracking(Findline &track, bool left);
  // 添加 mapPropType 
  TypeRace mapPropType(int propType);
};
