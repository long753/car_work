#pragma once
#include <vector>
#include <fstream>
#include <iostream>
#include <opencv2/highgui.hpp> //OpenCV终端部署
#include <opencv2/opencv.hpp>  //OpenCV终端部署
#include "../config/CarConfig.hpp"
#include "../utils/algorithm.h"
#include <cstdint>

using namespace std;
using namespace cv;

#define COLSIMAGE 320    // 图像的列数
#define ROWSIMAGE 240    // 图像的行数
#define COLSIMAGEIPM 320 // IPM图像的列数
#define ROWSIMAGEIPM 400 // IPM图像的行数
#define PWMSERVOMAX 1090 // 舵机PWM最大值（左）1840
#define PWMSERVOMID 880 // 舵机PWM中值 1520
#define PWMSERVOMIN 690 // 舵机PWM最小值（右）1200

#define LABEL_BOMB 0      // AI标签：爆炸物
#define LABEL_BRIDGE 1    // AI标签：坡道
#define LABEL_SAFETY 2    // AI标签：普通车辆
#define LABEL_CONE 3      // AI标签：锥桶
#define LABEL_CROSSWALK 4 // AI标签：斑马线
#define LABEL_DANGER 5    // AI标签：危险车辆
#define LABEL_EVIL 6      // AI标签：恐怖分子
#define LABEL_BLOCK 7     // AI标签：障碍物
#define LABEL_PATIENT 8   // AI标签：伤员
#define LABEL_PROP 9      // AI标签：道具车
#define LABEL_SPY 10      // AI标签：嫌疑车辆
#define LABEL_THIEF 11    // AI标签：盗贼
#define LABEL_TUMBLE 12   // AI标签：跌倒

enum class Scene {
  NormalScene,  // 基础赛道
  CrossScene,   // 十字道路
  RingScene,    // 环岛道路
  BridgeScene,  // 坡道区
  DangerScene,  // 危险区
  RescueScene,  // 救援区
  RacingScene,  // 追逐区
  BlocksScene,  // 障碍区
  ParkingScene, // 停车区
};

