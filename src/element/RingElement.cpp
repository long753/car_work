#include "RingElement.h"
#include "Findline.h"
#include <opencv2/imgproc.hpp>
#include <cmath>
#include <iostream>

// 构造函数
Ring::Ring() { 
 
}

// 重置函数
void Ring::reset() {
    RingType ringType = RingType::RingNone;
    RingStep ringStep = RingStep::None;
    repairline_straight = 30;

    ring_state_right.fill(0);
    ring_state_near_right.fill(0);
    ring_state_left.fill(0);
    ring_state_near_left.fill(0);

    corner_up_index = 0;
    corner_up_update_count = 0;

    corner_mid_index = 0;
    corner_mid_update_count = 0;

    corner_down_index = 0;
    corner_down_update_count = 0;

    corner_edge_index = 0;
    corner_edge_update_count = 0;

    corner_exit_index = 0;
    corner_exit_update_count = 0;

    exit_flag = 0;
    Entering_flag = 0;
    inside_flag = 0;

    Entering_time.reset();
    pre_entering_timer.reset();
    pre_entering_to_entering.reset();
    inside_to_exiting.reset();
    entering_to_inside.reset();
    reset_time.reset();

    start_angle = 0.0f;

    corner_up_point = cv::Point(0, 0);
    corner_mid_point = cv::Point(0, 0);
    corner_down_point = cv::Point(0, 0);
    corner_edge_point = cv::Point(0, 0);
    corner_exit_point = cv::Point(0, 0);

    timer1.reset();
}

// 圈搜索函数，目前没有利用陀螺仪积分来算角度，实在没时间写这么详细了，第二版可以尝试一下
void Ring::circle_search(Findline &findline, float angle) {
    if (image_to_enter == 1) {

        // 判断是否ringStep是否发生变化，同时对每一个阶段进行相应计算

        // 预入环
        if (ringStep == RingStep::PreEntering){

            //预入环操作








            // 判断是否更新
            if (ring.get_mid_cornor() == 0 && ring.get_up_cornor() == 1) {

                ring.ringStep = Ring::Entry;  
                ring.Entering_flag = 1;       
                ring.repair_line(ring.findline);
            }
        }

       // 入环
        else if (ringStep == RingStep::Entering){

            //入环操作








            // 判断是否更新
            if (ring.get_up_cornor() == 0 && 
                ring.check_far_corner_right() == 0 && 
                ring.check_far_corner_left() == 0) {

                if (ring.ringType == Ring::RingRight) {
                    // 如果是右入环，检查左近拐点
                    if (ring.check_near_corner_left() == 0) {
                        ring.ringStep = Ring::Inside;  
                        ring.inside_flag = 1;          
                    }
                } 
                else if (ring.ringType == Ring::RingLeft) {
                    // 如果是左入环，检查右近拐点
                    if (ring.check_near_corner_right() == 0) {
                        ring.ringStep = Ring::Inside;  
                        ring.inside_flag = 1;          
                    }
                }
            }
        }

        // 环内
        else if (ringStep == RingStep::Inside){




            // 判断是否更新
            if (ring.get_up_corner() == 0) {
                if (ring.ringType == Ring::RingRight) {
                    // 如果是右入环，检查左近拐点、右远拐点、左远拐点
                    if (ring.check_near_corner_left() == 1 || 
                        ring.check_far_corner_right() == 1 || 
                        ring.check_far_corner_left() == 1) {
                        ring.ringStep = Ring::Exiting;  
                        ring.repair_line_exit(Findline & findline);
                        ring.Entering_flag = 1;        
                    }
                } 
                else if (ring.ringType == Ring::RingLeft) {
                    // 如果是左入环，检查右近拐点、右远拐点、左远拐点
                    if (ring.check_near_corner_right() == 1 || 
                        ring.check_far_corner_right() == 1 || 
                        ring.check_far_corner_left() == 1) {
                        ring.ringStep = Ring::Exiting;  
                        ring.repair_line_exit(Findline & findline);
                        ring.Entering_flag = 1;         
                    }
                }
            }
        }

        // 出环
        else if (ringStep == RingStep::Exiting){




            // 判断是否更新
            if (ring.ringType == Ring::RingRight) {
                if (ring.straight_line_judge(ring.dir_l)) {
                    if (ring.get_up_corner() == 1) {
                        ring.ringStep = Ring::Finish;  
                    }
                }
            } 
            else if (ring.ringType == Ring::RingLeft) {
                if (ring.straight_line_judge(ring.dir_r)) {                   
                    if (ring.get_up_corner() == 1) {
                        ring.ringStep = Ring::Finish;  
                    }
                }
            }
        }

        // 完成
        else if (ringStep == RingStep::Finish){





            // 判断是否更新
            if (ring.get_up_corner() == 0) {
                ring.reset();
            }
        }
    }
}

// 检查远处右转拐点
int Ring::check_far_corner_right(Findline &findline) {
    // 示例逻辑：检测右侧线条数量是否超过阈值
    if (findline.pointsEdgeRight.size() > NO_LINE_RIGHT_Y) {
        return 1;
    }
    return 0;
}

// 检查近处右转拐点
int Ring::check_near_corner_right(Findline &findline) {
    if (findline.pointsEdgeRight.size() > NO_LINE_RIGHT_Y - 10) {
        return 1;
    }
    return 0;
}

// 检查右侧出口拐点
int Ring::check_exit_corner_right(Findline &findline) {
    if (findline.pointsEdgeRight.size() > NO_LINE_RIGHT_Y - 20) {
        return 1;
    }
    return 0;
}

// 检查远处左转拐点
int Ring::check_far_corner_left(Findline &findline) {
    if (findline.pointsEdgeLeft.size() > NO_LINE_LEFT_X) {
        return 1;
    }
    return 0;
}

// 检查近处左转拐点
int Ring::check_near_corner_left(Findline &findline) {
    if (findline.pointsEdgeLeft.size() > NO_LINE_LEFT_X - 2) {
        return 1;
    }
    return 0;
}

// 检查左侧出口拐点
int Ring::check_exit_corner_left(Findline &findline) {
    if (findline.pointsEdgeLeft.size() > NO_LINE_LEFT_X - 3) {
        return 1;
    }
    return 0;
}

// 获取上拐点
int Ring::get_up_corner(Findline &findline) {
    cv::Point corner_up_point {0,0};
    if (findline.pointsEdgeRight.empty() || findline.pointsEdgeLeft.empty()) {
        return 0;
    }

    // 假设取最前面的点作为拐点
    corner_up_point = findline.pointsEdgeRight.front();
    corner_up_update_count++;
    if (corner_up_update_count > 5) { // 更新计数达到阈值
        corner_up_update_count = 0;
        return 1;
    }
    return 0;
}

// 获取中拐点
int Ring::get_mid_corner(Findline &findline) {
    cv::Point corner_mid_point{0,0};
    if (findline.pointsEdgeRight.size() < 10 || findline.pointsEdgeLeft.size() < 10) {
        return 0;
    }

    // 假设取中间的点作为拐点
    corner_mid_point = findline.pointsEdgeRight[findline.pointsEdgeRight.size() / 2];
    corner_mid_update_count++;
    if (corner_mid_update_count > 5) {
        corner_mid_update_count = 0;
        return 1;
    }
    return 0;
}

// 获取下拐点
int Ring::get_down_corner(Findline &findline) {
    cv::Point corner_down_point{0,0};
    if (findline.pointsEdgeRight.empty() || findline.pointsEdgeLeft.empty()) {
        return 0;
    }

    // 假设取最后的点作为拐点
    corner_down_point = findline.pointsEdgeRight.back();
    corner_down_update_count++;
    if (corner_down_update_count > 5) {
        corner_down_update_count = 0;
        return 1;
    }
    return 0;
}

// 获取入环拐点
int Ring::get_edge_corner(Findline &findline) {
    cv::Point corner_edge_point {0,0};
    for (int i = 0; i < findline.pointsEdgeRight.size(); ++i) {
        if (findline.pointsEdgeRight[i].y > 250) { // 假设阈值
            corner_edge_point = findline.pointsEdgeRight[i];
            corner_edge_update_count++;
            if (corner_edge_update_count > 5) {
                corner_edge_update_count = 0;
                return 1;
            }
        }
    }
    return 0;
}

// 获取出环拐点
int Ring::get_exit_corner(Findline &findline) {
    cv::Point corner_exit_point {0,0};
    for (int i = 0; i < findline.pointsEdgeLeft.size(); ++i) {
        if (findline.pointsEdgeLeft[i].y < 50) { // 假设阈值
            corner_exit_point = findline.pointsEdgeLeft[i];
            corner_exit_update_count++;
            if (corner_exit_update_count > 5) {
                corner_exit_update_count = 0;
                return 1;
            }
        }
    }
    return 0;
}

// 检测到圆环存在,判断入环方向
int Ring::image_to_enter(Findline &findline) {

    if (straight_line_judge(findline.dir_l) ^ straight_line_judge(findline.dir_r)) {
        ringStep = RingStep::PreEntering;

        if (straight_line_judge(findline.dir_l)){
            ringType = RingType::RingLeft;
        }
        else{
            ringType = RingType::RingRight;
        }
        return 1;  
    }
    
    return 0;  
}


// 修复预入环赛道线
void Ring::repair_line_prev(Findline &findline) {
    std::vector<cv::Point> new_right_point;
    std::vector<cv::Point> new_left_point;

    if (ringType == RingType::RingRight) {                            // 右入环：
        for(int i = 1; i <= repairline_straight; ++i){

            double t = static_cast<double>(i) / (repairline_straight + 1);
            double new_x = corner_mid_point.x + t * (corner_down_point.x - corner_mid_point.x);
            double new_y = corner_mid_point.y + t * (corner_down_point.y - corner_mid_point.y);
            
            cv::Point new_point(static_cast<int>(new_x), static_cast<int>(new_y));
            new_right_point.emplace_back(new_point);
        }
        // 将生成的新点添加到findline.right_point
        findline.right_point.insert(pointsEdgeRight.end(), new_right_point.begin(), new_right_point.end());
    }
    else {                                                              // 左入环：
        for(int i = 1; i <= repairline_straight; ++i){
            double t = static_cast<double>(i) / (repairline_straight + 1);
            
            double new_x = corner_mid_point.x + t * (corner_down_point.x - corner_mid_point.x);
            double new_y = corner_mid_point.y + t * (corner_down_point.y - corner_mid_point.y);
            
            cv::Point new_point(static_cast<int>(new_x), static_cast<int>(new_y));
            new_left_point.emplace_back(new_point);
        }
        // 将生成的新点添加到findline.left_point
        findline.left_point.insert(pointsEdgeLeft.end(), new_left_point.begin(), new_left_point.end());
    }
}
  

// 修复入环赛道线
void Ring::repair_line(Findline &findline) {
    std::vector<cv::Point> new_right_point;
    std::vector<cv::Point> new_left_point;
    if (ringType == RingType::RingLeft) {                           // 左入环
        for(int i = 1; i <= repairline_straight; ++i){

            double t = static_cast<double>(i) / (repairline_straight + 1);
            double new_x = corner_up_point.x + t * (corner_edge_point.x - corner_up_point.x);
            double new_y = corner_up_point.y + t * (corner_edge_point.y - corner_up_point.y);
            
            cv::Point new_point(static_cast<int>(std::round(new_x)), static_cast<int>(std::round(new_y)));
            new_right_point.emplace_back(new_point);
        }
        // 将生成的新点添加到findline.right_point
        findline.right_point.insert(findline.right_point.end(), new_right_point.begin(), new_right_point.end());
    }
    else {                                                             // 右入环：     
        for(int i = 1; i <= repairline_straight; ++i){

            double t = static_cast<double>(i) / (repairline_straight + 1);
            double new_x = corner_up_point.x + t * (corner_edge_point.x - corner_up_point.x);
            double new_y = corner_up_point.y + t * (corner_edge_point.y - corner_up_point.y);
            
            cv::Point new_point(static_cast<int>(std::round(new_x)), static_cast<int>(std::round(new_y)));
            new_left_point.emplace_back(new_point);
        }
        // 将生成的新点添加到findline.left_point
        findline.left_point.insert(findline.left_point.end(), new_left_point.begin(), new_left_point.end());
    }
}



// 修复出环赛道线
void Ring::repair_line_exit(Findline &findline) {
    std::vector<cv::Point> new_right_point;
    std::vector<cv::Point> new_left_point;
    if (ringType == RingType::RingLeft) {                              // 左入环：
        for(int i = 1; i <= repairline_straight; ++i){                 

            double t = static_cast<double>(i) / (repairline_straight + 1);
            double new_x = corner_down_point.x + t * (corner_exit_point.x - corner_down_point.x);
            double new_y = corner_down_point.y + t * (corner_exit_point.y - corner_down_point.y);
            
            cv::Point new_point(static_cast<int>(std::round(new_x)), static_cast<int>(std::round(new_y)));
            new_right_point.emplace_back(new_point);
        }
        // 将生成的新点添加到findline.right_point
        findline.right_point.insert(findline.right_point.end(), new_right_point.begin(), new_right_point.end());
    }
    else {                                                             // 右入环：
        for(int i = 1; i <= repairline_straight; ++i){

            double t = static_cast<double>(i) / (repairline_straight + 1);
            double new_x = corner_down_point.x + t * (corner_exit_point.x - corner_down_point.x);
            double new_y = corner_down_point.y + t * (corner_exit_point.y - corner_down_point.y);

            cv::Point new_point(static_cast<int>(std::round(new_x)), static_cast<int>(std::round(new_y)));
            new_left_point.emplace_back(new_point);
        }
        // 将生成的新点添加到findline.left_point
        findline.left_point.insert(findline.left_point.end(), new_left_point.begin(), new_left_point.end());
    }
}


// 修复结束时的赛道线
void Ring::repair_line_exit(Findline &findline) {
    std::vector<cv::Point> new_right_point;
    std::vector<cv::Point> new_left_point;
    if (ringType == RingType::RingLeft) {                              // 左入环：
        for(int i = 1; i <= repairline_straight; ++i){                 

            double t = static_cast<double>(i) / (repairline_straight + 1);
            double new_x = corner_down_point.x + t * (corner_exit_point.x - corner_down_point.x);
            double new_y = corner_down_point.y + t * (corner_exit_point.y - corner_down_point.y);
            
            cv::Point new_point(static_cast<int>(std::round(new_x)), static_cast<int>(std::round(new_y)));
            new_right_point.emplace_back(new_point);
        }
        // 将生成的新点添加到findline.right_point
        findline.right_point.insert(findline.right_point.end(), new_right_point.begin(), new_right_point.end());
    }
    else {                                                             // 右入环：
        for(int i = 1; i <= repairline_straight; ++i){

            double t = static_cast<double>(i) / (repairline_straight + 1);
            double new_x = corner_down_point.x + t * (corner_exit_point.x - corner_down_point.x);
            double new_y = corner_down_point.y + t * (corner_exit_point.y - corner_down_point.y);

            cv::Point new_point(static_cast<int>(std::round(new_x)), static_cast<int>(std::round(new_y)));
            new_left_point.emplace_back(new_point);
        }
        // 将生成的新点添加到findline.left_point
        findline.left_point.insert(findline.left_point.end(), new_left_point.begin(), new_left_point.end());
    }
}


// 判断是否为直线
bool Ring::straight_line_judge(std::vector<int> dir) {
    const int CONTINUE_POINT_COUNT = 200; // 阈值
    const int CONTINUE_STRAIGHT_COUNT = 20;// 阈值
    int straight_line_count = std::count(dir.begin(), dir.end()-30, 7);

    if (dir.size() > CONTINUE_POINT_COUNT && straight_line_count < CONTINUE_STRAIGHT_COUNT){
        return true;
    }

    return false;
}
