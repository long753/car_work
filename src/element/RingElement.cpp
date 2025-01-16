#include "RingElement.h"
#include "Findline.h"
#include <opencv2/imgproc.hpp>
#include <cmath>
#include <iostream>

// 构造函数
Ring::Ring() { 
    reset(); 
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

// 圈搜索函数
void Ring::circle_search(Findline &findline, float angle) {
    switch (ringStep) {
        case RingStep::None:
            // 检测是否接近环岛
            if (check_far_corner_right(findline) || check_far_corner_left(findline)) {
                ringStep = RingStep::PreEntering;
                pre_entering_timer.reset();
                std::cout << "PreEntering Stage Started" << std::endl;
            }
            break;

        case RingStep::PreEntering:
            // 预入环岛阶段，等待一段时间后进入环岛
            if (pre_entering_timer.elapsed() > 1000) { // 例如等待1秒
                ringStep = RingStep::Entering;
                Entering_time.reset();
                std::cout << "Entering Stage Started" << std::endl;
            }
            break;

        case RingStep::Entering:
            // 检测入环岛的转角
            if (get_up_corner(findline)) {
                ringStep = RingStep::Inside;
                std::cout << "Inside RingScene" << std::endl;
            }
            break;

        case RingStep::Inside:
            // 在环岛中，持续检测环岛状态
            if (check_exit_corner_right(findline) || check_exit_corner_left(findline)) {
                ringStep = RingStep::Exiting;
                inside_to_exiting.reset();
                std::cout << "Exiting Stage Started" << std::endl;
            }
            break;

        case RingStep::Exiting:
            // 检测是否完成环岛任务
            if (check_exit_corner_right(findline) && check_exit_corner_left(findline)) {
                ringStep = RingStep::Finish;
                std::cout << "RingTask Finished" << std::endl;
                reset();
            }
            break;

        case RingStep::Finish:
            // 环岛任务完成，重置状态
            reset();
            break;

        default:
            reset();
            break;
    }

    // 更新当前场景类型
    if (ringStep != RingStep::None) {
        ringType = (check_far_corner_right(findline) || check_near_corner_right(findline)) ? RingType::RingRight : RingType::RingLeft;
    } else {
        ringType = RingType::RingNone;
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

// 获取出口拐点
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

// 预入环判定
int Ring::image_to_enter(Findline &findline) {
    // 示例逻辑：检测特定的线条模式以判断是否进入环岛
    if (findline.pointsEdgeRight.size() > NO_LINE_RIGHT_Y && findline.pointsEdgeLeft.size() > NO_LINE_LEFT_X) {
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



// 判断是否为直线（判定是否出环）
bool Ring::straight_line_judge(std::vector<int> dir) {
    int count = 0;
    for (auto d : dir) {
        if (abs(d) < 10) { // 阈值设定为10度
            count++;
        }
    }
    return (count > dir.size() / 2);
}
