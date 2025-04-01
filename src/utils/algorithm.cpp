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
    ringType = RingType::RingNone;
    ringStep = RingStep::None;
    repairline_straight = 30;
    ring_state_near_right = 0;
    ring_state_near_left = 0;

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

    corner_up_point = cv::Point(0, 0);
    corner_mid_point = cv::Point(0, 0);
    corner_down_point = cv::Point(0, 0);
    corner_edge_point = cv::Point(0, 0);
    corner_exit_point = cv::Point(0, 0);
}


// 圈搜索函数
void Ring::circle_search(Findline &findline, float angle) {

    //测试            
    // std::cout<<"测试开始"<<std::endl;
    // check_far_corner_right(findline);
    // check_mid_corner_right(findline);
    // check_near_corner_right(findline);
    // std::cout<<"测试结束"<<std::endl;

    if (ringStep == RingStep::None){
        image_to_enter(findline);
    }

    // 预入环
    if (ringStep == RingStep::PreEntering){
        flag_far = 0;
        flag_mid = 0;
        flag_near = 0;
        // start_angle = angle;
        if(ringType == RingType::RingLeft){
            std::cout<<"zuo zuo zuo zuo zuo zuo"<<std::endl;
        }

        //预入环操作
        if (ringType == RingType::RingRight){
            flag_far = check_far_corner_right(findline);
            // flag_mid = check_mid_corner_right(findline);
            // flag_near = check_near_corner_right(findline);
            get_up_corner(findline);
            get_mid_corner(findline);
            // repair_line_prev(findline);
        }
        else if (ringType == RingType::RingLeft){
            flag_far = check_far_corner_left(findline);
            flag_mid = check_mid_corner_left(findline);
            flag_near = check_near_corner_left(findline);
            get_up_corner(findline);
            get_mid_corner(findline);
            // repair_line_prev(findline);
        }

        //调试信息

        // std::cout<<"PreEntering"<<std::endl;
        // std::cout<<"findline.dir_r: "<<std::endl;
        // for(int i =0 ; i< findline.dir_r.size();++i){
        //     std::cout<<findline.dir_r[i]<<" ";
        // }
        // for(auto _m : findline.right_point){
        //     std::cout<<_m.x<<" ";
        // }
        // std::cout<<"flag_mid: "<<corner_down_index<<std::endl; //找不到？？？
        // std::cout<<"flag_far: "<<corner_up_index<<std::endl;     
        //std::cout<<"flag_near: "<<corner_down_index<<std::endl;   //ok
        // std::cout<<"angle: "<<angle<<std::endl;
        // std::cout<<std::endl;

        // 判断是否更新
        if (flag_mid == 0 && flag_far == 1) {
            // ringStep = RingStep::Entering;  
            Entering_flag = 1;       
        }
    }

    // 入环
    else if (ringStep == RingStep::Entering){
        flag_far = 0;
        flag_mid = 0;
        flag_near = 0;

        //入环操作
        if (ringType == RingType::RingRight){
            // flag_far = check_far_corner_right(findline);
            // get_up_corner(findline);
            repair_line_enter(findline);
        }
        else if (ringType == RingType::RingLeft){
            // flag_far = check_far_corner_left(findline);
            // get_up_corner(findline);
            repair_line_enter(findline);
        }
        
        // std::cout<<"Entering"<<std::endl;
        // std::cout<<"flag_far"<<flag_far<<std::endl;

        // 判断是否更新
        if (flag_far == 0 && Entering_flag == 1) {
            // ringStep = RingStep::Inside;  
            inside_flag = 1;          
        }
    }

    // 环内
    else if (ringStep == RingStep::Inside){
        flag_far = 0;
        flag_mid = 0;
        flag_near = 0;

        //环内操作
        if (ringType == RingType::RingRight){
            flag_near = check_exit_corner_right(findline);
        }
        else if (ringType == RingType::RingLeft){
            flag_near = check_exit_corner_left(findline);
        }
        std::cout<<"Inside"<<std::endl;

        // 判断是否更新
        if (flag_near == 1 && inside_flag == 1) {
            ringStep = RingStep::Exiting;  
            exit_flag = 1;  
        }
    }

    // 出环
    else if (ringStep == RingStep::Exiting){
        flag_far = 0;
        flag_mid = 0;
        flag_near = 0;
        
        //出环操作
        //check_corner(findline);
        //int find_up_corner = get_up_corner(findline);
        repair_line_exit(findline);
        std::cout<<"Exiting"<<std::endl;

        // 判断是否更新
        if(ringType == RingType::RingRight && straight_line_judge(findline.dir_l,findline.left_point) && exit_flag == 1){
            ringStep = RingStep::Finish;  
            reset();
        }
        if(ringType == RingType::RingLeft && straight_line_judge(findline.dir_r,findline.right_point) && exit_flag == 1){
            ringStep = RingStep::Finish;  
            reset();
        }
    }

    // 结束,考虑删除此状态，似乎影响不大
    else if (ringStep == RingStep::Finish){

        //结束操作
        if(ringType == RingType::RingRight){
            check_far_corner_right(findline);
            // get_up_corner(findline);
            // repair_line_finish(findline);
        }
        else if(ringType == RingType::RingLeft){    
            check_far_corner_left(findline);
            // get_up_corner(findline);
            // repair_line_finish(findline);
        }

        // 判断是否更新
        if (straight_line_judge(findline.dir_l,findline.left_point) && straight_line_judge(findline.dir_r,findline.right_point)) {
            reset();
        }
    }
}


// 检查远处右转拐点
int Ring::check_far_corner_right(Findline &findline) {
    return check_corner( findline.dir_r, findline.right_point, corner_up_index, 6, 4);
}

// 检查近处右转拐点
int Ring::check_near_corner_right(Findline &findline) {
    return check_corner( findline.dir_r, findline.right_point, corner_down_index, 4, 2);
}

// 检查右侧中拐点
int Ring::check_mid_corner_right(Findline &findline) {
    return check_corner( findline.dir_r, findline.right_point, corner_mid_index, 5, 3);
}

// 检查右环出口拐点
int Ring::check_exit_corner_right(Findline &findline) {
    return check_corner( findline.dir_l, findline.left_point, corner_exit_index, 4, 2);
}

// 检查远处左转拐点
int Ring::check_far_corner_left(Findline &findline) {
    return check_corner(findline.dir_l, findline.left_point, corner_up_index, 6, 4);
}

// 检查近处左转拐点
int Ring::check_near_corner_left(Findline &findline) {
    return check_corner(findline.dir_l, findline.left_point, corner_down_index, 4, 2);
}

// 检查左侧中拐点
int Ring::check_mid_corner_left(Findline &findline) {
    return check_corner( findline.dir_l, findline.left_point, corner_mid_index, 5, 3);
}

// 检查左环出口拐点
int Ring::check_exit_corner_left(Findline &findline) {
    return check_corner( findline.dir_r, findline.right_point, corner_exit_index, 4, 2);
}

// 获取上拐点
int Ring::get_up_corner(Findline &findline) {

    //初始化
    corner_up_point = cv::Point(0, 0);
    if(ringType == RingType::RingRight) {
        if(corner_up_index == 0 || corner_up_index >= findline.right_point.size()) {
            return 0;
        }

        corner_up_point = findline.right_point[corner_up_index];
        return 1;
    } else if(ringType == RingType::RingLeft) {
        if(corner_up_index == 0 || corner_up_index >= findline.left_point.size()) {
            return 0;
        }

        corner_up_point = findline.left_point[corner_up_index];
        return 1;
    }
}

// 获取中拐点
int Ring::get_mid_corner(Findline &findline) {

    //初始化
    corner_mid_point = cv::Point(0, 0);
    if(ringType == RingType::RingRight) {
        if(corner_mid_index == 0 || corner_mid_index >= findline.right_point.size()) {
            return 0;
        }
        corner_mid_point = findline.right_point[corner_mid_index];
        return 1;
    } else if(ringType == RingType::RingLeft) {
        if(corner_mid_index == 0 || corner_mid_index >= findline.left_point.size()) {
            return 0;
        }
        corner_mid_point = findline.left_point[corner_mid_index];
        return 1;
    }
}

// 获取下拐点
int Ring::get_down_corner(Findline &findline) {

    //初始化
    corner_down_point = cv::Point(0, 0);
    if(ringType == RingType::RingRight) {
        if(corner_down_index == 0 || corner_down_index >= findline.right_point.size()) {
            return 0;
        }

        corner_down_point = findline.right_point[corner_down_index];
        return 1;
    } else if(ringType == RingType::RingLeft) {
        if(corner_down_index == 0 || corner_down_index >= findline.left_point.size()) {
            return 0;
        }

        corner_down_point = findline.left_point[corner_down_index];
        return 1;
    }
}

// 获取入环拐点
int Ring::get_edge_corner(Findline &findline) {
    return 0;
}

// 获取出环拐点
int Ring::get_exit_corner(Findline &findline) {
    return 0;
}


// 检测圆环,判断入环方向
//当连续检测到三次右环时，判断为右环，防止误判十字
int Ring::image_to_enter(Findline &findline) {

    //增强鲁棒性方案
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // 疑似圆环
    // if(straight_line_judge(findline.dir_l, findline.left_point) ^ straight_line_judge(findline.dir_r, findline.right_point)) {  //左右两边类型不同
    //     if(straight_line_judge(findline.dir_l, findline.left_point) && ring_state_right == 0) {
    //         ring_state_near_right = ring_state_near_right + 1;
    //         if(ring_state_near_right > 3) {
    //             ringType = RingType::RingRight;
    //             ringStep = RingStep::PreEntering;
    //             // ring_state_near_right  = 1;  //根据场上的圆环数量来判断
    //             return 1;
    //         }
    //     } 
    //     else if(straight_line_judge(findline.dir_r, findline.right_point) && ring_state_left == 0) {
    //         ring_state_near_left = ring_state_near_left + 1;
    //         if(ring_state_near_left > 3) {
    //             ringType = RingType::RingLeft;
    //             ringStep = RingStep::PreEntering;
    //             // ring_state_near_left = 1;  //根据场上的圆环数量来判断
    //             return 1;
    //         }
    //     }
    // }
    // //不是圆环
    // else {
    //     ring_state_near_right = 0;
    //     ring_state_near_left = 0;
    // }
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    //原方案
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    if(straight_line_judge(findline.dir_l, findline.left_point) ^ straight_line_judge(findline.dir_r, findline.right_point)) {  //左右两边类型不同
        if(straight_line_judge(findline.dir_l, findline.left_point) && ring_state_right == 0) {
                ringType = RingType::RingRight;
                // std::cout<<"Right"<<std::endl;
                ringStep = RingStep::PreEntering;
                // ring_state_near_right  = 1;  //根据场上的圆环数量来判断
                return 1;
        } 
        else if(straight_line_judge(findline.dir_r, findline.right_point) && ring_state_left == 0) {
                ringType = RingType::RingLeft;
                // std::cout<<"Left"<<std::endl;
                ringStep = RingStep::PreEntering;
                // ring_state_near_left = 1;  //根据场上的圆环数量来判断
                return 1;
        }
    }
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    // std::cout<<"test left:"<<std:endl;
    // straight_line_judge(findline.dir_l, findline.left_point);
    // std::cout<<"test right:"<<std:endl;
    // straight_line_judge(findline.dir_r, findline.right_point);


    // std::cout<<"left : ";
    // // for(auto _m : findline.left_point){
    // //     std::cout<<_m.y<<" ";
    // // }
    // for(int i = 0 ; i<240 ; ++i){
    //     std::cout<<findline.leftpoint[i]<<" ";
    // }
    // std::cout<<std::endl;

    // std::cout<<"right : ";
    // // for(auto _m : findline.right_point){
    // //     std::cout<<_m.y<<" ";
    // // }
    // for(int i = 0 ; i<240 ; ++i){
    //     std::cout<<findline.rightpoint[i]<<" ";
    // }
    // std::cout<<std::endl;
    return 0;
}

// 修复预入环赛道线
void Ring::repair_line_prev(Findline &findline)
{
    ringType == RingType::RingRight;
    corner_mid_point = cv::Point(200,60);

    // RingRight
    if (ringType == RingType::RingRight)
    {
        // 1.1 检查 corner_mid_point / corner_down_point 是否有效
        // if (corner_mid_point.x == 0 && corner_mid_point.y == 0) {
        //     std::cout<<"didn't find mid_point"<<std::endl;
        //     return;
        // }
        if (corner_down_point.x == 0 && corner_down_point.y == 0) {
            corner_down_point = cv::Point(320, 210);//经验值，可以修改
        }

        int rowStart = corner_down_point.y;   // 例如 220
        int rowEnd   = corner_mid_point.y;    // 例如 60
        int colStart = corner_down_point.x;   // 例如 310
        int colEnd   = corner_mid_point.x;    // 例如 200

        // 如果下拐点行 <= 中拐点行，就不补线
        if (rowStart <= rowEnd) {
            return;
        }

        //     rowCur 从 rowStart 递减到 rowEnd，t 从 0~1
        for (int rowCur = rowStart; rowCur >= rowEnd; --rowCur)
        {
            
            float t = float(rowCur - rowStart) / float(rowEnd - rowStart);
            float colInterp = colStart + t * (colEnd - colStart);

            int final_row = rowCur;
            int final_col = static_cast<int>(colInterp);

            findline.pointsEdgeRight[235 - rowCur] = mpoint(final_row, final_col);
        }
    }

    //RingLeft
    else if (ringType == RingType::RingLeft)
    {
        // 检查 corner_mid_point / corner_down_point 
        if (corner_mid_point.x == 0 && corner_mid_point.y == 0){
            return;
        }
        if (corner_down_point.x == 0 && corner_down_point.y == 0){
            corner_down_point = cv::Point(230, 235);
        }

        // 取行列坐标
        int rowStart = corner_down_point.y;   
        int rowEnd   = corner_mid_point.y;    
        int colStart = corner_down_point.x;   
        int colEnd   = corner_mid_point.x;    

        if (rowStart <= rowEnd) {
            return;
        }

        //进行插值
        for (int rowCur = rowStart; rowCur >= rowEnd; --rowCur)
        {
            float t = float(rowCur - rowStart) / float(rowEnd - rowStart);
            float colInterp = colStart + t * (colEnd - colStart);
            int final_row = rowCur;
            int final_col = static_cast<int>(colInterp);
            findline.pointsEdgeRight[235 - rowCur] = mpoint(final_row, final_col);
        }
    }
}



// 修复入环赛道线
void Ring::repair_line_enter(Findline &findline){

    //测试
    // corner_up_point = cv::Point(200,60);
    // cv::Point top_left_point(5, 230);
    // ringType = RingType::RingRight;

    //RingRight
    if(ringType == RingType::RingRight){
        if (corner_up_point.x == 0 && corner_up_point.y == 0){
            // std::cout<<"corner_up_point is not found"<<std::endl;
            return;
        }

        // 左边线与屏幕上缘的交点
        cv::Point top_left_point(0, 0); 
        bool foundLeftTop = false;

        for (int i = 0; i < static_cast<int>(findline.pointsEdgeLeft.size()) - 1; ++i){
            auto &pt = findline.pointsEdgeLeft[i];
            // 防止近处干扰
            if (i >= 5){
                auto &pt2 = findline.pointsEdgeLeft[i + 1];
                auto &pt3 = findline.pointsEdgeLeft[i + 2];
                auto &pt4 = findline.pointsEdgeLeft[i + 3];
                auto &pt5 = findline.pointsEdgeLeft[i + 4];
                if (pt2.col != 3 && pt2.col != 4 && pt3.col != 3 && pt3.col != 4 && pt4.col != 3 && pt4.col != 4 && pt5.col != 3 && pt5.col != 4){
                    top_left_point = pt;
                    foundLeftTop = true;
                    break;
                }
            }
        }

        if (!foundLeftTop)
        {
            // std::cout<<"top_left_point is not found"<<std::endl;
            return;
        }

        int rowStart = top_left_point.y; //230
        int rowEnd   = corner_up_point.y; //60
        int colStart = top_left_point.x; //5
        int colEnd   = corner_up_point.x; //200
        if (rowStart <= rowEnd){
            return;
        }

        //补线操作
        for (int rowCur = rowStart; rowCur >= rowEnd; --rowCur){
            float t =  float(rowCur - rowStart) / float(rowEnd - rowStart);
            float colInterp = colStart + t * (colEnd - colStart);
            int final_row =  rowCur;  
            int final_col = static_cast<int>(colInterp);
            findline.pointsEdgeLeft[235-rowCur] = mpoint( final_row , final_col);
        }
    }

    // RingLeft
    if (ringType == RingType::RingLeft) {
        // 1. 检查 corner_up_point 是否有效
        if (corner_up_point.x == 0 && corner_up_point.y == 0) {
            // std::cout << "corner_up_point is not found" << std::endl;
            return;
        }

        // 2. 在右边线中寻找与屏幕上缘的交点 top_right_point
        cv::Point top_right_point(0, 0);
        bool foundRightTop = false;  // 根据需要改为 true/false 默认值

        for (int i = 0; i < static_cast<int>(findline.pointsEdgeRight.size()) - 1; ++i) {
            auto &pt = findline.pointsEdgeRight[i];
            // 防止近处干扰
            if (i >= 5) {
                auto &pt2 = findline.pointsEdgeRight[i + 1];
                auto &pt3 = findline.pointsEdgeRight[i + 2];
                auto &pt4 = findline.pointsEdgeRight[i + 3];
                auto &pt5 = findline.pointsEdgeRight[i + 4];

                // 只要后面几个点 col 都不是初始值(3 或 4)，就认为找到“上缘交点”
                if (pt2.col != 3 && pt2.col != 4 &&
                    pt3.col != 3 && pt3.col != 4 &&
                    pt4.col != 3 && pt4.col != 4 &&
                    pt5.col != 3 && pt5.col != 4)
                {
                    top_right_point = pt;     // 记录该点
                    foundRightTop = true;    
                    break;
                }
            }
        }

        if (!foundRightTop) {
            // std::cout << "top_right_point is not found" << std::endl;
            return;
        }

        // 3. 获取插值起止：rowStart/rowEnd, colStart/colEnd
        //    按照你右环的写法，rowStart = top_right_point.y, rowEnd = corner_up_point.y
        int rowStart = top_right_point.y;
        int rowEnd   = corner_up_point.y;
        int colStart = top_right_point.x;
        int colEnd   = corner_up_point.x;

        // 如果起始行 <= 结束行，直接返回
        if (rowStart <= rowEnd) {
            return;
        }

        // 4. 做插值，并将结果写到 pointsEdgeRight
        for (int rowCur = rowStart; rowCur >= rowEnd; --rowCur) {
            // 注意使用 float(...) 避免整型除法
            float t = float(rowCur - rowStart) / float(rowEnd - rowStart);
            float colInterp = colStart + t * (colEnd - colStart);

            int final_row = rowCur;
            int final_col = static_cast<int>(colInterp);
            // 保持和右环一样: array下标 = [235 - rowCur] (可根据场上需要做调整)
            findline.pointsEdgeRight[235 - rowCur] = mpoint(final_row, final_col);
        }
    }
}




// 修复出环赛道线
void Ring::repair_line_exit(Findline &findline)
{
    // RingRight
    if (ringType == RingType::RingRight)
    {
        // 检查是否有效
        if (corner_exit_point.x == 0 && corner_exit_point.y == 0){
            return;
        }
        if (findline.pointsEdgeRight.empty()){
            return;
        }

        // 获取 pointsEdgeRight 最后一个点
        auto &ptLast = findline.pointsEdgeRight.back(); 
        cv::Point lastRightPoint(ptLast.col, ptLast.row);//到时候看看要不要使用row和col

        //准备插值的起止
        int rowStart = corner_exit_point.y; 
        int rowEnd   = lastRightPoint.y;    
        int colStart = corner_exit_point.x; 
        int colEnd   = lastRightPoint.x;    
        if (rowStart <= rowEnd)
        {
            return;
        }

        // 补线
        for (int rowCur = rowStart; rowCur >= rowEnd; --rowCur)
        {
            float t = float(rowCur - rowStart) / float(rowEnd - rowStart);   
            float colInterp = colStart + t * (colEnd - colStart);

            int final_row = rowCur;
            int final_col = static_cast<int>(colInterp);
            int idx = 235 - rowCur;  // 或者 240 - rowCur
            if (idx >= 0 && idx < (int)findline.pointsEdgeLeft.size())
            {
                findline.pointsEdgeLeft[idx] = mpoint(final_row, final_col);
            }
        }
    }

    //(RingLeft
    else if (ringType == RingType::RingLeft)
    {
        // 检查是否有效
        if (corner_exit_point.x == 0 && corner_exit_point.y == 0){
            return;
        }
        if (findline.pointsEdgeLeft.empty()){
            return;
        }

        // 获取 pointsEdgeLeft 最后一个点
        auto &ptLast = findline.pointsEdgeLeft.back(); 
        cv::Point lastLeftPoint(ptLast.col, ptLast.row);

        // 计算起止
        int rowStart = corner_exit_point.y; 
        int rowEnd   = lastLeftPoint.y;     
        int colStart = corner_exit_point.x; 
        int colEnd   = lastLeftPoint.x;     
        if (rowStart <= rowEnd){
            return;
        }

        // 补线
        for (int rowCur = rowStart; rowCur >= rowEnd; --rowCur){
            float t = float(rowCur - rowStart) / float(rowEnd - rowStart);
            float colInterp = colStart + t * (colEnd - colStart);

            int final_row = rowCur;
            int final_col = static_cast<int>(colInterp);

            int idx = 235 - rowCur;  // 或者 240 - rowCur
            if (idx >= 0 && idx < (int)findline.pointsEdgeRight.size())
            {
                findline.pointsEdgeRight[idx] = mpoint(final_row, final_col);
            }
        }
    }
}



// 修复结束时的赛道线
void Ring::repair_line_finish(Findline &findline) {
//     std::vector<cv::Point> new_right_point;
//     std::vector<cv::Point> new_left_point;

//     if (ringType == RingType::RingRight) {                            // 右入环：
//         for(int i = 1; i <= repairline_straight; ++i){

//             double t = static_cast<double>(i) / (repairline_straight + 1);
//             double new_x = corner_mid_point.x + t * (corner_up_point.x - corner_mid_point.x);
//             double new_y = corner_mid_point.y + t * (corner_up_point.y - corner_mid_point.y);
            
//             cv::Point new_point(static_cast<int>(int(new_x)), static_cast<int>(int(new_y)));
//             new_right_point.emplace_back(new_point);
//         }
//         // 将生成的新点添加到findline.right_point
//         findline.right_point.insert(pointsEdgeRight.end(), new_right_point.begin(), new_right_point.end());
//         findline.edge_calculate();
//         findline.midline_calculate();
//     }
//     else {                                                              // 左入环：
//         for(int i = 1; i <= repairline_straight; ++i){
//             double t = static_cast<double>(i) / (repairline_straight + 1);
            
//             double new_x = corner_mid_point.x + t * (corner_up_point.x - corner_mid_point.x);
//             double new_y = corner_mid_point.y + t * (corner_up_point.y - corner_mid_point.y);
            
//             cv::Point new_point(static_cast<int>(new_x), static_cast<int>(new_y));
//             new_left_point.emplace_back(new_point);
//         }
//         // 将生成的新点添加到findline.left_point
//         findline.left_point.insert(pointsEdgeLeft.end(), new_left_point.begin(), new_left_point.end());
//         findline.edge_calculate();
//         findline.midline_calculate();
//     }
}

// 判断是否为直线
bool Ring::straight_line_judge(std::vector<int> dir, std::vector<cv::Point> points) {
    std::vector<int> suspect_corner;
    std::vector<std::pair<int,int>> line_type_and_count;
    int guai_count = 0;
    int current_corner = 0;
    int range_count = 5;
	int flag1 = 0;
	int flag2 = 0;
	int corner1_index = 0;
	int corner2_index = 0;

    for(size_t i = 0; i < dir.size(); i++) {
        if(dir[i] == 4 || dir[i] == 5) {
            if(!line_type_and_count.empty() && line_type_and_count.back().first != 4) {
                if(line_type_and_count.back().second > range_count) {
                    suspect_corner.emplace_back(i);
                    line_type_and_count.emplace_back(4, 1);
                } else {
                    line_type_and_count.pop_back();
                    if(!line_type_and_count.empty()) {
                        line_type_and_count.back().second++;
                    }
                }
            }
            if(!line_type_and_count.empty() && line_type_and_count.back().first == 4) {
                line_type_and_count.back().second++;
            } else {
                line_type_and_count.emplace_back(4, 1);
            }
        } else if(dir[i] == 1 || dir[i] == 2 || dir[i] == 3) {
            if(!line_type_and_count.empty() && line_type_and_count.back().first != 2) {
                if(line_type_and_count.back().second > range_count) {
                    suspect_corner.emplace_back(i);
                    line_type_and_count.emplace_back(2, 1);
                } else {
                    line_type_and_count.pop_back();
                    if(!line_type_and_count.empty()) {
                        line_type_and_count.back().second++;
                    }
                }
            }
            if(!line_type_and_count.empty() && line_type_and_count.back().first == 2) {
                line_type_and_count.back().second++;
            } else {
                line_type_and_count.emplace_back(2, 1);
            }
        } else if(dir[i] == 6 || dir[i] == 7 || dir[i] == 8) {
            if(!line_type_and_count.empty() && line_type_and_count.back().first != 6) {
                if(line_type_and_count.back().second > range_count) {
                    suspect_corner.emplace_back(i);
                    line_type_and_count.emplace_back(6, 1);
                } else {
                    line_type_and_count.pop_back();
                    if(!line_type_and_count.empty()) {
                        line_type_and_count.back().second++;
                    }
                }
            }
            if(!line_type_and_count.empty() && line_type_and_count.back().first == 6) {
                line_type_and_count.back().second++;
            } else {
                line_type_and_count.emplace_back(6, 1);
            }
        }
    }

    if(line_type_and_count.size() > 1 || line_type_and_count.size() - 1 == suspect_corner.size()) {
        for(size_t i = 0; i < line_type_and_count.size() - 1 && !suspect_corner.empty(); i++) {
            double angle = neighbour_points_angle(suspect_corner[i], points);
            // if(line_type_and_count[i].first == 4 && line_type_and_count[i + 1].first == 2
            //     && line_type_and_count[i].second >= 10 && line_type_and_count[i + 1].second >= 10
            //     && angle <= 120) {
			// 		flag1 =1;
			// 		corner1_index = suspect_corner[i];
			// 		std::cout<<"corner1_index corner1_index corner1_index corner1_index:"<<corner1_index<<std::endl;
			// 	}

			// if(line_type_and_count[i].first == 6 && line_type_and_count[i + 1].first == 4
			// && line_type_and_count[i].second >= 10 && line_type_and_count[i + 1].second >= 10
			// && angle <= 120) {
			// 	flag2 =1;
			// 	corner2_index = suspect_corner[i];
			// 	std::cout<<"corner2_index corner2_index corner2_index corner2_index:"<<corner2_index<<std::endl;
			// }

			// if(flag1 == 1 && flag2 == 1 ){
			// 	return true;
			// }

			// return false;

            if(line_type_and_count[i].second >= 10 && line_type_and_count[i + 1].second >= 10
                && 20 <= angle <= 120) { 
                if(current_corner == 0) {
                    current_corner = suspect_corner[i];
                    guai_count++;
                } else {
                    if(suspect_corner[i] - current_corner > 25) {
                        current_corner = suspect_corner[i]; 
                        guai_count++;
                    }
                }
            }
        }
    }
    std::cout<<guai_count;
    return guai_count < 4;
}

// 检查拐点的公共函数
int Ring::check_corner(std::vector<int> &dir, std::vector<cv::Point> &points,int &corner_index, int type1, int type2) {
    std::vector<int> suspect_corner;
    std::vector<std::pair<int,int>> line_type_and_count;
    int range_count = 5; // 阈值
    corner_index = 0; //初始化

	for(size_t i = 0;i < dir.size() ;i++){

        //非中点计算
        if(type1 != 5){
            //4
            if(dir[i] ==4 ||dir[i] ==5 ){ 
                //有可能是拐点的情况
                if(!line_type_and_count.empty() && line_type_and_count.back().first != 4){ 
                    if(line_type_and_count.back().second >range_count){
                        suspect_corner.emplace_back(i);
                        line_type_and_count.emplace_back(4,1);
                    }
                    else{
                        line_type_and_count.pop_back(); //把阈值内的点都删掉
                        if(!line_type_and_count.empty()){
                            line_type_and_count.back().second++;
                        }
                    }
                }
                //还是直线的情况
                if(!line_type_and_count.empty()&& line_type_and_count.back().first ==4){
                    line_type_and_count.back().second++;
                }
                //第一次
                else {
                    line_type_and_count.emplace_back(4,1); 
                }
            }

            //2
            if(dir[i] ==1 ||dir[i] ==2 ||dir[i] ==3 ){     
                //有可能是拐点的情况
                if(!line_type_and_count.empty() && line_type_and_count.back().first != 2){ 
                    if(line_type_and_count.back().second >range_count){
                        suspect_corner.emplace_back(i);
                        line_type_and_count.emplace_back(2,1);
                    }
                    else{
                        line_type_and_count.pop_back(); //把阈值内的点都删掉
                        if(!line_type_and_count.empty()){
                            line_type_and_count.back().second++;
                        }
                    }
                }
                //还是直线的情况
                if(!line_type_and_count.empty()&& line_type_and_count.back().first ==2){
                    line_type_and_count.back().second++;
                }
                //第一次
                else {
                    line_type_and_count.emplace_back(2,1); 
                }
            }

            //6
            if(dir[i] ==6 ||dir[i] ==7 ||dir[i] ==8 ){     
                //有可能是拐点的情况
                if(!line_type_and_count.empty() && line_type_and_count.back().first != 6){ 
                    if(line_type_and_count.back().second >range_count){
                        suspect_corner.emplace_back(i);
                        line_type_and_count.emplace_back(6,1);
                    }
                    else{
                        line_type_and_count.pop_back(); //把阈值内的点都删掉
                        if(!line_type_and_count.empty()){
                            line_type_and_count.back().second++;
                        }
                    }
                }
                //还是直线的情况
                if(!line_type_and_count.empty()&& line_type_and_count.back().first ==6){
                    line_type_and_count.back().second++;
                }
                //第一次
                else {
                    line_type_and_count.emplace_back(6,1); 
                }
            }
        }

        //中点计算
        else if(type1 == 5){
            //5
            if(dir[i] ==5 ||dir[i] ==6){     
                //有可能是拐点的情况
                if(!line_type_and_count.empty() && line_type_and_count.back().first != 5){ 
                    if(line_type_and_count.back().second >range_count){
                        suspect_corner.emplace_back(i);
                        line_type_and_count.emplace_back(5,1);
                    }
                    else{
                        line_type_and_count.pop_back(); //把阈值内的点都删掉
                        if(!line_type_and_count.empty()){
                            line_type_and_count.back().second++;
                        }
                    }
                }
                //还是直线的情况
                if(!line_type_and_count.empty()&& line_type_and_count.back().first ==5){
                    line_type_and_count.back().second++;
                }
                //第一次
                else {
                    line_type_and_count.emplace_back(5,1); 
                }
            }

            //3
            if(dir[i] ==3 ||dir[i] ==2){     
                //有可能是拐点的情况
                if(!line_type_and_count.empty() && line_type_and_count.back().first != 3){ 
                    if(line_type_and_count.back().second >range_count){
                        suspect_corner.emplace_back(i);
                        line_type_and_count.emplace_back(3,1);
                    }
                    else{
                        line_type_and_count.pop_back(); //把阈值内的点都删掉
                        if(!line_type_and_count.empty()){
                            line_type_and_count.back().second++;
                        }
                    }
                }
                //还是直线的情况
                if(!line_type_and_count.empty()&& line_type_and_count.back().first ==3){
                    line_type_and_count.back().second++;
                }
                //第一次
                else {
                    line_type_and_count.emplace_back(3,1); 
                }
            }

            //4
            if(dir[i] ==4){
                //续直线
                if(!line_type_and_count.empty()){
                    if(line_type_and_count.back().first == 3 || line_type_and_count.back().first == 5){
                        line_type_and_count.back().second++;
                    }
                }
            }

            //0                      
            else{     
                //有可能是拐点的情况
                if(!line_type_and_count.empty() && line_type_and_count.back().first != 0){ 
                    if(line_type_and_count.back().second >range_count){
                        suspect_corner.emplace_back(i);
                        line_type_and_count.emplace_back(0,1);
                    }
                    else{
                        line_type_and_count.pop_back(); //把阈值内的点都删掉
                        if(!line_type_and_count.empty()){
                            line_type_and_count.back().second++;
                        }
                    }
                }
                //还是直线的情况
                if(!line_type_and_count.empty()&& line_type_and_count.back().first ==0){
                    line_type_and_count.back().second++;
                }
                //第一次
                else {
                    line_type_and_count.emplace_back(0,1); 
                }
            }
        }
    }

    bool find = false;
    // std::cout<<"开始检测拐点： "<<std::endl;
    // std::cout<<"line_type_and_count.size()"<<line_type_and_count.size()<<std::endl;
    // std::cout<<"line_type_and_count[1].second"<<line_type_and_count[1].second<<std::endl;
    // std::cout<<"type:"<<type1<<std::endl;
    if(line_type_and_count.size() > 1 || line_type_and_count.size() - 1 == suspect_corner.size()) {
        for(size_t i = 0; i < line_type_and_count.size() - 1 && !suspect_corner.empty(); i++) {
            double angle = neighbour_points_angle(suspect_corner[i], points);
            // if(line_type_and_count[i].first == type1 && line_type_and_count[i + 1].first == type2
            //     && line_type_and_count[i].second >= 10 && line_type_and_count[i + 1].second >= 10
            //     && !find && angle <= 130) {
            //     corner_index = suspect_corner[i];
            //     find = true;
            // }
            if(line_type_and_count[i].first == type1 && line_type_and_count[i + 1].first == type2){
                std::cout<<"line_type_and_count[i].second:"<<line_type_and_count[i].second<<std::endl;
                std::cout<<"line_type_and_count[i+1].second:"<<line_type_and_count[i+1].second<<std::endl;
                std::cout<<"angle:"<<angle<<std::endl;
                std::cout<<"suspect_corner[i]"<<suspect_corner[i]<<std::endl;
                std::cout<<"上面的点的y:" <<points[i+10].y<<std::endl;
                std::cout<<"上面的点的x:" <<points[i+10].x<<std::endl;
                std::cout<<"下面的点的y:" <<points[i-10].y<<std::endl;
                std::cout<<"上面的点的x:" <<points[i+10].x<<std::endl;
            }
        }
    }
    std::cout<<std::endl<<"over "<<std::endl;
    return find ? 1 : 0;
}
