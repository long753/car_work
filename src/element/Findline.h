#pragma once
#include "../include/common.hpp"
#include "../include/stop_watch.hpp"
#include <cmath>

using namespace cv;
using namespace std;

#define LEFT_EDGE_COL 4
#define RIGHT_EDGE_COL 315


enum class LineType{
  STRAIGHT,
  LEFTTURN,
  RIGHTTURN,
};

 enum class ImageType {
      Binary , // 二值化
      Rgb,        // RGB
};
class Findline {
public:
    ImageType imageType;
    LineType line_type;
    std::vector<mpoint> pointsEdgeLeft;  // 赛道左边缘点集
    std::vector<mpoint> pointsEdgeRight; // 赛道右边缘点集
    std::vector<mpoint> PreviouspointsEdgeLeft;  // 赛道左边缘点集
    std::vector<mpoint> PreviouspointsEdgeRight; // 赛道右边缘点集
    bool ai_edge_point_save{false};
    std::vector<cv::Point> left_point; //原始点集
    std::vector<cv::Point> right_point;
    cv::Mat kernel;
    int validRowsLeft = 0;   // 边缘有效行数（左）
    int validRowsRight = 0;  // 边缘有效行数（右）
    int empty_row_left = 0;  // 边缘空行左
    int empty_row_right = 0; // 边缘空行右
    std::array<int,240> width_length ={53, 54, 55, 56, 58, 59, 60, 61, 62, 64, 65, 66, 67, 68, 69,
71, 72, 73, 74, 74, 75, 75, 77, 78, 79, 81, 81, 83, 85, 85,
87, 89, 89, 91, 91, 93, 95, 95, 97, 97, 99, 100, 102, 103, 104,
106, 106, 108, 109, 110, 112, 112, 114, 115, 116, 118, 118, 120, 121, 122,
124, 125, 126, 127, 129, 129, 131, 132, 133, 135, 135, 137, 138, 139, 141,
142, 143, 144, 146, 147, 148, 149, 150, 152, 152, 154, 155, 156, 157, 158,
160, 160, 162, 164, 165, 166, 167, 169, 170, 171, 173, 173, 175, 176, 177,
179, 179, 181, 182, 183, 185, 185, 187, 188, 189, 190, 192, 193, 194, 196,
196, 198, 200, 200, 202, 203, 204, 205, 207, 208, 209, 210, 211, 213, 214,
215, 217, 217, 219, 220, 221, 223, 224, 225, 226, 227, 228, 230, 231, 232,
234, 234, 236, 237, 238, 240, 241, 242, 244, 245, 246, 247, 248, 249, 251,
252, 253, 255, 256, 257, 258, 259, 261, 261, 263, 265, 265, 267, 268, 269,
271, 272, 273, 274, 276, 276, 278, 279, 280, 282, 282, 283, 285, 285, 287,
289, 290, 291, 293, 293, 294, 294, 295, 295, 296, 297, 297, 298, 299, 299,
300, 300, 301, 302, 302, 303, 303, 304, 304, 305, 306, 306, 307, 307, 308,
309, 309, 310, 310, 311, 311, 319, 319, 319, 319, 319, 319, 319, 319, 319};
    uint16_t rowCutUp = 10;     // 图像顶部切行
    uint16_t rowCutBottom = 10; // 图像底部切行
    bool loseflag = false;      // 丢线标志
    bool special_element = false; // 特殊元素巡线 
    int guai_up_right;
    // int repairline_huanside = 319;
    int repairline_straight = CarParams->rowCutBottom + 20;
    std::vector<cv::Point> seeds_l{{0, 1},  {-1, 1}, {-1, 0}, {-1, -1},
                              {0, -1}, {1, -1}, {1, 0},  {1, 1}};
    std::vector<cv::Point> seeds_r{{0, 1},  {1, 1},   {1, 0},  {1, -1},
                              {0, -1}, {-1, -1}, {-1, 0}, {-1, 1}};
    int highest = 0;
    StopWatch time2;
    StopWatch time3;

    bool loop_inside = false;
    
    int state = 0;
    int loop_state = 0;
    cv::Point guai_point{0, 0};
    cv::Point guai_mid_point{0, 0};
    cv::Point guai_down_point{0, 0};
    cv::Point guai_bian{0, 0};
    bool guai_flag = true;
    int guai_up_prev = 0;
    int guai_update_count = 0; // 拐点丢失计数
    int guai_up_prev_x = 0;
    int guai_update_count_bian = 0; // 拐点丢失计数
    int guai_up_prev_bian = 0;
    int guai_update_count_x = 0; // 拐点丢失计数
    int guai_up_prev_z = 0;
    int guai_update_count_z = 0; // 拐点丢失计数
    int guai_up_prev_leave = 0;
    int guai_update_count_leave = 0; // 拐点丢失计数
    int guai_update_count_chu = 0;
    int guai_up_prev_chu = 0;
    int leave_flag = 0;
    int start_leave = 0;
    int zuo_num = 0;
    int you_num = 0;
    int rect_l=0;
    int rect_r=0;
    int flag_left_t=0;
    int flag_right_t=0;
    int exit_l=-1;
    int exit_r=-1;
    int flag_left_t_finish=0;
    int flag_right_t_finish=0;
    int in_l=0;
    int in_r=0;
    std::vector<int> dir_l; // 保存生长方向
    std::vector<int> dir_r;

    int endline_eight;
    std::array<int, 240> midline;
    std::array<int, 240> widthline;

    int highest_row = 240;
    Findline();
    void search_line(Mat &imageBinary); // 八邻域
    void drawImage(Mat &trackImage, std::vector<mpoint> pointsLeft,
                  std::vector<mpoint> pointsRight);
    int zuodandiao();
    int youdandiao();
    void midline_calculate();
    void edge_calculate();
    // void distinguish_road_type();


    int zuo_dan_diao_width();
    int you_dan_diao_width();
    auto get_findline_draw_task(){
      return 
        [*this](cv::Mat img) mutable{
      for (size_t i = 0; i < pointsEdgeLeft.size(); i++) {
      circle(img, pointsEdgeLeft[i], 1, Scalar(0, 255, 0), -1); // 绿色点
     }
      for (size_t i = 0; i < pointsEdgeRight.size(); i++) {
      circle(img, pointsEdgeRight[i], 1, Scalar(0, 255, 255), -1); // 黄色点
    }
      for (size_t i = endline_eight; i < midline.size(); i++){
        circle(img, Point(midline[i], i), 3,Scalar(0, 0, 255), -1);                          // 红色点
      }
      // std::cout << endline_eight;
        };
      
    }




private:
    cv::Point point_add(cv::Point &p1, cv::Point &p2);
};