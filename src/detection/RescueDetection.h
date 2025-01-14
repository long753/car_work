#pragma once 
#include "../include/common.hpp"
#include "../element/Findline.h"
#include "../include/detection.hpp"
#include "../vision/mapping.h"
#include "../utils/algorithm.h"
class Rescue {
public:
  bool carStoping = false;  // 停车标志
  bool carExitting = false; // 出库标志
  int angle_ = 0;//实时角度
  int angle = 0;//上次记录角度
  enum Step {
    None = 0, // AI检测
    Enable,   // 使能（标志识别成功）
    Enter,    // 进站
    Cruise,   // 巡航
    Stop,     // 停车
    Exit      // 出站
  };

  Step step = Step::None;
  bool entryLeft = true; // 左入库使能标志
  /**
   * @brief 检测初始化
   *
   */
  void reset(void);

  /**
   * @brief 检测与路径规划
   *
   * @param track 赛道识别结果
   * @param detection AI检测结果
   */
  bool process(Findline &track, vector<PredictResult> predict);
  void assert_conenum(vector<PredictResult> predict);


  /**
   * @brief 识别结果图像绘制
   *
   */
void drawImage(Findline track, Mat &image);

auto get_draw_task(){
  return [*this](cv::Mat img){
    string state = "None";
    switch (step) {
    case Step::Enable:
      state = "Enable";
      break;
    case Step::Enter:
      state = "Enter";
      break;
    case Step::Cruise:
      state = "Cruise";
      break;
    case Step::Stop:
      state = "Stop";
      break;
    case Step::Exit:
      state = "Exit";
      break;
    }
    if (entryLeft) {
      // 绘制锥桶坐标
      for (size_t i = 0; i < pointConeLeft.size(); i++) {
        circle(img, Point(pointConeLeft[i].col, pointConeLeft[i].row), 2,
               Scalar(92, 92, 205), -1); // 锥桶坐标：红色
      }
      putText(img, "[3] RESCUE - LEFT", Point(COLSIMAGE / 2 - 30, 10),
              cv::FONT_HERSHEY_TRIPLEX, 0.3, cv::Scalar(0, 255, 0), 1, CV_AA);
    } else {
      // 绘制锥桶坐标
      for (size_t i = 0; i < pointConeRight.size(); i++) {
        circle(img, Point(pointConeRight[i].col, pointConeRight[i].row), 2,
               Scalar(92, 92, 205), -1); // 锥桶坐标：红色
      }
      putText(img, "[3] RESCUE - RIGHT", Point(COLSIMAGE / 2 - 30, 10),
              cv::FONT_HERSHEY_TRIPLEX, 0.3, cv::Scalar(0, 255, 0), 1, CV_AA);
    }

    putText(img, state, Point(COLSIMAGE / 2 - 10, 30),
            cv::FONT_HERSHEY_TRIPLEX, 0.3, cv::Scalar(0, 255, 0), 1, CV_AA);

    putText(img, to_string(_distance), Point(COLSIMAGE / 2 - 15, 40),
            cv::FONT_HERSHEY_TRIPLEX, 0.3, cv::Scalar(0, 255, 0), 1,
            CV_AA); // 显示锥桶距离
    if (_pointNearCone.row > 0)
      circle(img, Point(_pointNearCone.col, _pointNearCone.row), 5,
             Scalar(200, 200, 200), -1);
    if (levelCones > 0)
      line(img, Point(0, levelCones), Point(img.cols, levelCones),
           Scalar(255, 255, 255), 1);
    putText(img, to_string(indexDebug),
            Point(COLSIMAGE / 2 - 10, ROWSIMAGE - 20), cv::FONT_HERSHEY_TRIPLEX,
            0.3, cv::Scalar(0, 0, 255), 1, CV_AA);
  };
}
private:
  bool again = false; // 第二次进入救援区标志
  double _distance = 0;
  int levelCones = 0; // 锥桶的平均高度
  int enter_row = 0;
  bool retreat_dirct = true;
  int Rowmax_last = 0;
  bool enter_flag_Rowmax = false;
  mpoint _pointNearCone;
  mpoint pointHCone;
  vector<mpoint> pointConeLeft;      // AI元素检测边缘点集
  vector<mpoint> pointConeRight;     // AI元素检测边缘点集
  vector<mpoint> lastPointsEdgeLeft; // 记录上一场边缘点集（丢失边）
  vector<mpoint> lastPointsEdgeRight;

  vector<vector<mpoint>> pathsEdgeLeft; // 记录入库路径
  vector<vector<mpoint>> pathsEdgeRight;
  int indexDebug = 0;
  

  uint16_t counterSession = 0;  // 图像场次计数器
  uint16_t counterRec = 0;      // 标志检测计数器
  uint16_t counterExit = 0;     // 标志结束计数器
  uint16_t counterImmunity = 0; // 屏蔽计数器

  /**
   * @brief 从AI检测结果中检索锥桶坐标集合
   *
   * @param predict AI检测结果
   * @return vector<mpoint>
   */
  void searchCones(vector<PredictResult> predict);

  /**
   * @brief 搜索距离赛道左边缘最近的锥桶坐标
   *
   * @param pointsEdgeLeft 赛道边缘点集
   * @param predict AI检测结果
   * @return mpoint
   */
  mpoint getConeLeftDown(vector<mpoint> pointsEdgeLeft,
                        vector<mpoint> pointsCone);



  /**
   * @brief 搜索距离赛道右边缘最近的锥桶坐标
   *
   * @param pointsEdgeRight 赛道边缘点集
   * @param predict AI检测结果
   * @return mpoint
   */
  mpoint getConeRightDown(vector<mpoint> pointsEdgeRight,
                         vector<mpoint> pointsCone);

  /**
   * @brief 在俯视域由左边缘预测右边缘
   *
   * @param pointsEdgeLeft
   * @return vector<mpoint>
   */
  vector<mpoint> predictEdgeRight(vector<mpoint> &pointsEdgeLeft);

  /**
   * @brief 在俯视域由右边缘预测左边缘
   *
   * @param pointsEdgeRight
   * @return vector<mpoint>
   */
  
vector<mpoint> predictEdgeLeft(vector<mpoint> &pointsEdgeRight);
  /**
   * @brief 按照坐标点的y排序
   *
   * @param points
   * @return vector<int>
   */
  void pointsSortForY(vector<mpoint> &points);
};