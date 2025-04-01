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
    std::cout<<"测试开始"<<std::endl;

    if (ringStep == RingStep::None && image_to_enter(findline)){
        start_angle = angle;
    }

    // 预入环
    if (ringStep == RingStep::PreEntering){
        flag_far = 0;
        flag_mid = 0;
        flag_near = 0;

        //预入环操作
        if (ringType == RingType::RingRight){
            flag_far = check_far_corner_right(findline);
            flag_mid = check_mid_corner_right(findline);
            flag_near = check_near_corner_right(findline);
            get_up_corner(findline);
            get_mid_corner(findline);
            get_down_corner(findline);
            repair_line_prev(findline);
        }
        else if (ringType == RingType::RingLeft){
            flag_far = check_far_corner_left(findline);
            flag_mid = check_mid_corner_left(findline);
            flag_near = check_near_corner_left(findline);
            get_up_corner(findline);
            get_mid_corner(findline);
            get_down_corner(findline);
            repair_line_prev(findline);
        }

        //调试信息
        // std::cout<<"PreEntering"<<std::endl;
        // std::cout<<"findline.dir_r: "<<std::endl;
        // for(int i =0 ; i< findline.dir_r.size();++i){
        //     std::cout<<findline.dir_r[i]<<" ";
        // }
        // std::cout<<std::endl;
        // for(auto _m : findline.right_point){
        //     std::cout<<_m.x<<" ";
        // }
        // std::cout<<"flag_mid: "<<corner_down_index<<std::endl; //找不到？？？
        // std::cout<<"flag_far: "<<corner_up_index<<std::endl;     
        //std::cout<<"flag_near: "<<corner_down_index<<std::endl;   //ok
        // std::cout<<"angle: "<<angle<<std::endl;
        // std::cout<<std::endl;
         //陀螺仪测试
        float angle_diff = angle - start_angle;
        if(angle_diff < 0){
            angle_diff = angle_diff + 360;
        }//陀螺仪顺时针为正，逆时针为负
        std::cout<<"陀螺仪转动角度: "<<angle_diff<<std::endl;

        // 判断是否更新
        if(corner_down_update_count < 3 && flag_near == 0){
            corner_down_update_count = corner_down_update_count+1;
        }
        if(corner_down_update_count < 3 && flag_near != 0){
            corner_down_update_count = 0;
        }
        if(corner_down_update_count == 3){ 
            if(corner_mid_update_count < 3 && flag_mid == 0){
                corner_mid_update_count = corner_mid_update_count +1;
            }  
            if(corner_mid_update_count < 3 && flag_mid != 0){
                corner_mid_update_count = 0;
            }
        }
        // std::cout<<" flag_mid: "<<flag_mid<<std::endl;
        // std::cout<<" corner_down_update_count: "<<corner_down_update_cosunt<<std::endl;
        // std::cout<<" corner_mid_update_count: "<<corner_mid_update_count<<std::endl;
        std::cout<<"测试结束"<<std::endl;
        if(corner_mid_update_count == 3 && corner_down_update_count == 3 ) {//中拐点以及下拐点消失，认为进入Entering
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
            flag_far = check_far_corner_right(findline);
            get_up_corner(findline);
            repair_line_enter(findline);
        }
        else if (ringType == RingType::RingLeft){
            flag_far = check_far_corner_left(findline);
            get_up_corner(findline);
            repair_line_enter(findline);
        }
        float angle_diff = angle - start_angle;
        if(angle_diff < 0){
            angle_diff = angle_diff + 360;
        }
        
        //调试信息
        std::cout<<"Entering"<<std::endl;
        // std::cout<<"flag_far"<<flag_far<<std::endl;

        // 判断是否更新
        if (corner_up_update_count < 10 && flag_far == 0) {
            corner_up_update_count = corner_up_update_count +1;
        }
        if (corner_up_update_count < 10 && flag_far != 0) {
            corner_up_update_count = 0;
        }
        if(corner_up_update_count == 10){ //上拐点消失,拐了一定角度——20
            ringStep = RingStep::Inside;  
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
        float angle_diff = angle - start_angle;
        if(angle_diff < 0){
            angle_diff = angle_diff + 360;
        }

        //调试信息
        std::cout<<"Inside"<<std::endl;

        // 判断是否更新
        if(corner_exit_update_count < 10 && flag_near != 0){
            corner_exit_update_count = corner_exit_update_count + 1;
        }
        if(corner_exit_update_count < 10 && flag_near == 0){
            corner_exit_update_count = 0;
        }
        if (corner_exit_update_count == 10 ) { //消失拐点出现，且转动一定角度——330
            ringStep = RingStep::Exiting;  
            exit_flag = 1;  
            corner_exit_update_count = 0;
        }
    }

    // 出环
    else if (ringStep == RingStep::Exiting){
        flag_far = 0;
        flag_mid = 0;
        flag_near = 0;
        
        //出环操作
        if (ringType == RingType::RingRight){
            flag_near = check_exit_corner_right(findline);
        }
        else if (ringType == RingType::RingLeft){
            flag_near = check_exit_corner_left(findline);
        }
        repair_line_exit(findline); //目前是计划使用特定直线，因为另外一个点不好找

        //调试信息
        std::cout<<"Exiting"<<std::endl;

        // 判断是否更新
        if(corner_exit_update_count < 5 && flag_near == 0){
            corner_exit_update_count = corner_exit_update_count + 1;
        }
        if(corner_exit_update_count < 5 && flag_near != 0){
            corner_exit_update_count = 0;
        }
        if(corner_exit_update_count == 5){
           ringStep = RingStep::Finish;
        }
    }

    // 结束,考虑删除此状态，似乎影响不大
    else if (ringStep == RingStep::Finish){

        //结束操作
        if(ringType == RingType::RingRight){
            check_far_corner_right(findline);
            get_up_corner(findline);
            repair_line_finish(findline);
        }
        else if(ringType == RingType::RingLeft){    
            check_far_corner_left(findline);
            get_up_corner(findline);
            repair_line_finish(findline);
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
    
    //初始化
    corner_exit_point = cv::Point(0, 0);
    if(ringType == RingType::RingRight) {
        if(corner_exit_index == 0 || corner_exit_index >= findline.left_point.size()) {
            return 0;
        }
        corner_exit_point = findline.left_point[corner_exit_index];
        return 1;
    } else if(ringType == RingType::RingLeft) {
        if(corner_exit_index == 0 || corner_exit_index >= findline.right_point.size()) {
            return 0;
        }
        corner_exit_point = findline.right_point[corner_exit_index];
        return 1;
    }
}


// 检测圆环,判断入环方向
//当连续检测到三次右环时，判断为右环，防止误判十字
int Ring::image_to_enter(Findline &findline) {
    if(findline.ring_state_right == 2 || findline.ring_state_left == 2){
        return 0;
    }

    //原版逻辑
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    if(straight_line_judge(findline.dir_l, findline.left_point) ^ straight_line_judge(findline.dir_r, findline.right_point)) {  //左右两边类型不同
        //参数
        int suspectPointIndex_cur = 0;
        //右环
        if(straight_line_judge(findline.dir_l, findline.left_point) && guaiCount_left <= 2) {  //左线是直线
            bool errorJudge = false; // 是否误判圆环（十字）
            for (int i = 0; i < findline.dir_l.size(); ++i) {
                if (findline.dir_l[i] == 2 && findline.dir_l[i + 1] == 4 && i - suspectPointIndex_cur > 20) {
                    int suspectPoint_row = findline.left_point[i].y;
                    suspectPointIndex_cur = i;
                    int startIndex = std::max(0, suspectPoint_row - 20); 
                    int endIndex = std::min((int)findline.pointsEdgeLeft.size(), suspectPoint_row + 20);
                    bool topCondition = false; 
                    int topCount = 0; //上面符合条件的点的个数
                    bool bottomCondition = false; 
                    int bottomCount = 0;//下面符合条件的点的个数

                    for (int j = startIndex; j < suspectPoint_row; ++j) {
                        if (findline.pointsEdgeLeft.size()>240-j && findline.pointsEdgeLeft[240-j].col < 5) {
                            topCount = topCount +1; 
                            if(topCount > (suspectPointIndex_cur - startIndex)*0.8 ){  
                                topCondition = true;
                            }
                            break;
                        }
                    }
                    for (int j = suspectPoint_row + 1; j < endIndex; ++j) {
                        if (findline.pointsEdgeLeft.size()>240-j && findline.pointsEdgeLeft[240-j].col > 10) {
                            bottomCount = bottomCount+1;
                            if(bottomCount >(endIndex - suspectPointIndex_cur)*0.8){
                                bottomCondition = true; 
                            }
                            break;
                        }
                    }
                    if (topCondition && bottomCondition) {
                        errorJudge = true;
                        break; 
                    }
                }
            }
            if(!errorJudge){
                ringType = RingType::RingRight;
                ringStep = RingStep::PreEntering;
                return 1;                    
            }


            //增加鲁棒性
            // ring_state_near_right = ring_state_near_right + 1;
            // if(ring_state_near_right > 3) {
            //     ringType = RingType::RingRight;
            //     ringStep = RingStep::PreEntering;
            //     return 1;
            // }
        } 

        //左环
        else if(straight_line_judge(findline.dir_r, findline.right_point)  && guaiCount_right <= 2) {//右线是直线
            bool errorJudge = false; // 是否误判圆环（十字）
            for (int i = 0; i < findline.dir_r.size(); ++i) {
                if (findline.dir_r[i] == 2 && findline.dir_r[i + 1] == 4 && i - suspectPointIndex_cur > 20) {
                    int suspectPoint_row = findline.right_point[i].y;
                    suspectPointIndex_cur = i;
                    int startIndex = std::max(0, suspectPoint_row - 20); 
                    int endIndex = std::min((int)findline.pointsEdgeRight.size(), suspectPoint_row + 20);
                    bool topCondition = false; 
                    int topCount = 0; //上面符合条件的点的个数
                    bool bottomCondition = false; 
                    int bottomCount = 0;//下面符合条件的点的个数

                    for (int j = startIndex; j < suspectPoint_row; ++j) {
                        if (findline.pointsEdgeRight.size()>240-j&&findline.pointsEdgeRight[240-j].col > 315) {
                            topCount = topCount +1; 
                            if(topCount > (suspectPointIndex_cur - startIndex)*0.8 ){  
                                topCondition = true;
                            }
                            break;
                        }
                    }
                    for (int j = suspectPoint_row + 1; j < endIndex; ++j) {
                        if (findline.pointsEdgeRight.size()>240-j && findline.pointsEdgeRight[240-j].col > 10) {
                            bottomCount = bottomCount+1;
                            if(bottomCount >(endIndex - suspectPointIndex_cur)*0.8){
                                bottomCondition = true; 
                            }
                            break;
                        }
                    }
                    if (topCondition && bottomCondition) {
                        errorJudge = true;
                        break; 
                    }
                }
            }

            if(!errorJudge){
                ringType = RingType::RingLeft;
                ringStep = RingStep::PreEntering;
                return 1;
            }


            //增强鲁棒性
            // ring_state_near_left = ring_state_near_left + 1;
            // if(ring_state_near_left > 3) {
            //     ringType = RingType::RingLeft;
            //     ringStep = RingStep::PreEntering;
            //     return 1;
            // }
        }
    }
    else {
        ring_state_near_right = 0;
        ring_state_near_left = 0;
    }
    // std::cout<<"guaiCount_right: "<<guaiCount_right<<std::endl;
    // std::cout<<"guaiCount_left: "<<guaiCount_left<<std::endl;

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    //新逻辑
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // if(straight_line_judge(findline.dir_l, findline.pointsEdgeLeft) ^ straight_line_judge(findline.dir_r, findline.pointsEdgeRight)) {  //左右两边类型不同
    //     if(straight_line_judge(findline.dir_l, findline.pointsEdgeLeft)) { 

    //         //增加鲁棒性
    //         // ring_state_near_right = ring_state_near_right + 1;
    //         // if(ring_state_near_right > 3) {
    //         //     ringType = RingType::RingRight;
    //         //     ringStep = RingStep::PreEntering;
    //         //     return 1;
    //         // }

    //         //原版
    //         // ringType = RingType::RingRight;
    //         // ringStep = RingStep::PreEntering;
    //         // return 1;
    //     } 
    //     else if(straight_line_judge(findline.dir_r, findline.pointsEdgeRight)) {
    //         // ring_state_near_left = ring_state_near_left + 1;
    //         // if(ring_state_near_left > 3) {
    //         //     ringType = RingType::RingLeft;
    //         //     ringStep = RingStep::PreEntering;
    //         //     return 1;
    //         // }
    //         // ringType = RingType::RingLeft;
    //         // ringStep = RingStep::PreEntering;
    //         // return 1;
    //     }
    // }
    // else {
    //     ring_state_near_right = 0;
    //     ring_state_near_left = 0;
    // }
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    return 0;
}

// 修复预入环赛道线
void Ring::repair_line_prev(Findline &findline)
{
    // RingRight
    if (ringType == RingType::RingRight)
    {
        //检查 corner_mid_point / corner_down_point 如果没有找到，就给一个默认值
        if (corner_mid_point.x == 0 && corner_mid_point.y == 0) {
            corner_mid_point = cv::Point(240,60);

        }
        if (corner_down_point.x == 0 && corner_down_point.y == 0) {
            corner_down_point = cv::Point(320, 210);
        }

        int rowStart = corner_down_point.y;   
        int rowEnd   = corner_mid_point.y;    
        int colStart = corner_down_point.x;   
        int colEnd   = corner_mid_point.x;    

        if (rowStart <= rowEnd) {
            return;
        }

        //补线
        for (int rowCur = rowStart; rowCur >= rowEnd; --rowCur)
        {
            float t = float(rowCur - rowStart) / float(rowEnd - rowStart);
            float colInterp = colStart + t * (colEnd - colStart);
            int final_row = rowCur;
            int final_col = static_cast<int>(colInterp);
            if(findline.pointsEdgeRight.size() > 235 - rowCur){
                findline.pointsEdgeRight[235 - rowCur] = mpoint(final_row, final_col);
            }
        }
    }

    //RingLeft
    else if (ringType == RingType::RingLeft)
    {
        //检查 corner_mid_point / corner_down_point 如果没有找到，就给一个默认值
        // if (corner_mid_point.x == 0 && corner_mid_point.y == 0){
        //     corner_mid_point = cv::Point(40, 60);
        // }
        corner_mid_point = cv::Point(80, 60);
        if (corner_down_point.x == 0 && corner_down_point.y == 0){
            corner_down_point = cv::Point(5, 210);
        }

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
            if(findline.pointsEdgeLeft.size() > 235 - rowCur){
                findline.pointsEdgeLeft[235 - rowCur] = mpoint(final_row, final_col);
            }
        }
    }
}



// 修复入环赛道线
void Ring::repair_line_enter(Findline &findline){

    //RingRight
    if(ringType == RingType::RingRight){
        //检测corner_up_point是否有效
        if (corner_up_point.x == 0 && corner_up_point.y == 0){
            corner_up_point = cv::Point(300,80);
        }

        // 左边线与屏幕上缘的交点
        cv::Point top_left_point(0, 0); 
        bool foundLeftTop = false;

        for (int i = 0; i < static_cast<int>(findline.pointsEdgeLeft.size()) - 1; ++i){
            auto &pt = findline.pointsEdgeLeft[i];
            // 防止近处干扰
        //     if (i >= 5){
        //         auto &pt2 = findline.pointsEdgeLeft[i + 1];
        //         auto &pt3 = findline.pointsEdgeLeft[i + 2];
        //         auto &pt4 = findline.pointsEdgeLeft[i + 3];
        //         auto &pt5 = findline.pointsEdgeLeft[i + 4];
        //         if (pt2.col != 3 && pt2.col != 4 && pt3.col != 3 && pt3.col != 4 && pt4.col != 3 && pt4.col != 4 && pt5.col != 3 && pt5.col != 4
        //             && pt.row >200){ //防止这个点的位置太诡异了
        //             top_left_point = pt;
        //             foundLeftTop = true;
        //             break;
        //         }
        //     }
        }
        if (!foundLeftTop)
        {
            top_left_point = cv::Point(5,200);
        }


        int rowStart = top_left_point.y; 
        int rowEnd   = corner_up_point.y; 
        int colStart = top_left_point.x; 
        int colEnd   = corner_up_point.x; 
        if (rowStart <= rowEnd){
            return;
        }

        //补线操作
        for (int rowCur = rowStart; rowCur >= rowEnd; --rowCur){
            float t =  float(rowCur - rowStart) / float(rowEnd - rowStart);
            float colInterp = colStart + t * (colEnd - colStart);
            int final_row =  rowCur;  
            int final_col = static_cast<int>(colInterp);
            if(findline.pointsEdgeLeft.size() > 236-rowCur){
                findline.pointsEdgeLeft[235-rowCur] = mpoint( final_row , final_col);
            }
        }
    }

    // RingLeft
    if (ringType == RingType::RingLeft) {
        // 检查 corner_up_point 是否有效
        // if (corner_up_point.x == 0 && corner_up_point.y == 0) {
        //     corner_up_point = cv::Point(40,60);
        // }
        corner_up_point = cv::Point(20,40);

        // top_right_point
        cv::Point top_right_point(0, 0);
        bool foundRightTop = false;  

        for (int i = 0; i < static_cast<int>(findline.pointsEdgeRight.size()) - 1; ++i) {
            auto &pt = findline.pointsEdgeRight[i];
            //数组越界需要修改
        //     if (i >= 5) {
        //         auto &pt2 = findline.pointsEdgeRight[i + 1];
        //         auto &pt3 = findline.pointsEdgeRight[i + 2];
        //         auto &pt4 = findline.pointsEdgeRight[i + 3];
        //         auto &pt5 = findline.pointsEdgeRight[i + 4];

        //         if (pt2.col != 319 && pt2.col != 318 &&
        //             pt3.col != 319 && pt3.col != 318 &&
        //             pt4.col != 319 && pt4.col != 318 &&
        //             pt5.col != 319 && pt5.col != 318 &&
        //             pt.row > 200){
        //             top_right_point = pt;     
        //             foundRightTop = true;    
        //             break;
        //         }
        //     }
        }
        if (!foundRightTop) {
            top_right_point = cv::Point(315, 200);
        }

        int rowStart = top_right_point.y;
        int rowEnd   = corner_up_point.y;
        int colStart = top_right_point.x;
        int colEnd   = corner_up_point.x;

        if (rowStart <= rowEnd) {
            return;
        }

        //补线
        for (int rowCur = rowStart; rowCur >= rowEnd; --rowCur) {
            float t = float(rowCur - rowStart) / float(rowEnd - rowStart);
            float colInterp = colStart + t * (colEnd - colStart);
            int final_row = rowCur;
            int final_col = static_cast<int>(colInterp);
            if(findline.pointsEdgeRight.size() > 236 - rowCur){
                findline.pointsEdgeRight[235 - rowCur] = mpoint(final_row, final_col);
            }
        }
    }
}




// 修复出环赛道线
void Ring::repair_line_exit(Findline &findline)
{
    // RingRight
    if (ringType == RingType::RingRight)
    {
        int rowStart = 180; 
        int rowEnd   = 60;    
        int colStart = 5; 
        int colEnd   = 320;    
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

    //RingLeft
    else if (ringType == RingType::RingLeft)
    {
        int rowStart = 180; 
        int rowEnd   = 60;     
        int colStart = 320; 
        int colEnd   = 5;     
        if (rowStart <= rowEnd){
            return;
        }

        // 补线
        for (int rowCur = rowStart; rowCur >= rowEnd; --rowCur){
            float t = float(rowCur - rowStart) / float(rowEnd - rowStart);
            float colInterp = colStart + t * (colEnd - colStart);
            int final_row = rowCur;
            int final_col = static_cast<int>(colInterp);
            int idx = 235 - rowCur;  
            if (idx >= 0 && idx < (int)findline.pointsEdgeRight.size())
            {
                findline.pointsEdgeRight[idx] = mpoint(final_row, final_col);
            }
        }
    }
}


// 修复结束时的赛道线
void Ring::repair_line_finish(Findline &findline) {
    // RingRight
    if (ringType == RingType::RingRight)
    {
        //检查 corner_mid_point / corner_up_point 如果没有找到，就给一个默认值
        corner_mid_point = cv::Point(320, 210);
        if (corner_up_point.x == 0 && corner_up_point.y == 0) {
            corner_up_point = cv::Point(240,100);
        }

        int rowEnd = corner_up_point.y;   
        int rowStart   = corner_mid_point.y;    
        int colEnd = corner_up_point.x;   
        int colStart   = corner_mid_point.x;    

        if (rowStart <= rowEnd) {
            return;
        }

        //补线
        for (int rowCur = rowStart; rowCur >= rowEnd; --rowCur)
        {
            float t = float(rowCur - rowStart) / float(rowEnd - rowStart);
            float colInterp = colStart + t * (colEnd - colStart);
            int final_row = rowCur;
            int final_col = static_cast<int>(colInterp);
            if(findline.pointsEdgeRight.size() > 235 - rowCur){
                findline.pointsEdgeRight[235 - rowCur] = mpoint(final_row, final_col);
            }
        }
    }

    //RingLeft
    else if (ringType == RingType::RingLeft)
    {
        //检查 corner_mid_point / corner_up_point 如果没有找到，就给一个默认值
        corner_mid_point = cv::Point(5, 210);
        if (corner_up_point.x == 0 && corner_up_point.y == 0){
            corner_up_point = cv::Point(40, 100);
        }

        int rowEnd = corner_up_point.y;   
        int rowStart   = corner_mid_point.y;    
        int colEnd = corner_up_point.x;   
        int colStart   = corner_mid_point.x;    

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
            if(findline.pointsEdgeRight.size() > 235 - rowCur){
                findline.pointsEdgeRight[235 - rowCur] = mpoint(final_row, final_col);
            }
        }
    }
}


// 判断是否为直线
bool Ring::straight_line_judge(std::vector<int> dir, std::vector<cv::Point> points) {
    std::vector<int> suspect_corner;
    std::vector<std::pair<int,int>> line_type_and_count;
    int guai_count = 0;
    int current_corner = 0;
    int range_count = 5;
    /////////////////////////////////////////测试参数
    int corner_up_index = 0;
    int corner_down_index = 0;
    int corner_mid_index = 0;

    for(size_t i = 0; i < dir.size(); i++) {
        if(dir[i] == 4 || dir[i] == 5) {
            if(!line_type_and_count.empty() && line_type_and_count.back().first != 4) {
                if(line_type_and_count.back().second > range_count) {
                    suspect_corner.emplace_back(i);
                    line_type_and_count.emplace_back(4, 1);
                } else {
                    line_type_and_count.pop_back();
                    if(!suspect_corner.empty()){
                            suspect_corner.pop_back();  //把可疑点删掉
                        }
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
        } 
        else if(dir[i] == 1 || dir[i] == 2 || dir[i] == 3) {
            if(!line_type_and_count.empty() && line_type_and_count.back().first != 2) {
                if(line_type_and_count.back().second > range_count) {
                    suspect_corner.emplace_back(i);
                    line_type_and_count.emplace_back(2, 1);
                } else {
                    line_type_and_count.pop_back();
                    if(!suspect_corner.empty()){
                            suspect_corner.pop_back();  //把可疑点删掉
                        }
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
        } 
        else if(dir[i] == 6 || dir[i] == 7 || dir[i] == 8) {
            if(!line_type_and_count.empty() && line_type_and_count.back().first != 6) {
                if(line_type_and_count.back().second > range_count) {
                    suspect_corner.emplace_back(i);
                    line_type_and_count.emplace_back(6, 1);
                } else {
                    line_type_and_count.pop_back();
                    if(!suspect_corner.empty()){
                            suspect_corner.pop_back();  //把可疑点删掉
                        }
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
            if( line_type_and_count[i].second >= 10){
                for(int j = i + 1; j < line_type_and_count.size(); j++){
                    if( line_type_and_count[j].second >= 10 && suspect_corner.size()>j-1){
                        double angle = neighbour_points_angle(suspect_corner[j-1], points);
                        if(current_corner == 0  && angle <140 ){
                            current_corner = suspect_corner[j-1];
                            guai_count++;
                        }
                        else if(current_corner != 0&& angle <140 ){
                            if(suspect_corner[j-1] - current_corner > 25){
                                current_corner = suspect_corner[j-1];
                                guai_count++;
                            }
                        }
                        break;                          
                    }
                }
            }
        }
    }
    if(!points.empty()){
        if(points[0].x < 160){
                guaiCount_left = guai_count;
        }
        else{
                guaiCount_right = guai_count;
        }
    }
     return guai_count <= 4;  

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////修改版
    //如果点数量很少，可以直接认为连续
    // if (points.size() < 100 ) {
    //     return true; 
    // }

    // // 1) 区分左右线
    // bool isLeftLine = (points[0].col < 160);  // 原先 points[0].x < 160

    // // 丢线的判定阈值
    // int lostThreshold = 10;     // 连续多少个点算丢线
    // int checkEnd = points.size() - 40;

    // // 2) 准备一个结构体来记录“丢线片段”的起始/结束下标
    // struct LostSegment {
    //     int startIdx;
    //     int endIdx;
    // };
    // std::vector<LostSegment> lostSegments;

    // // 3) 遍历并找丢线片段
    // int i = 0;
    // while (i < checkEnd) {
    //     // 判断是否满足丢线条件
    //     bool condition = false;
    //     if (isLeftLine) {
    //         // 左线：col < 5
    //         condition = (points[i].col < 5);
    //     } else {
    //         // 右线：col > 315
    //         condition = (points[i].col > 315);
    //     }

    //     if (condition) {
    //         // 找到一个满足丢线条件的起点，开始计数
    //         int start = i;

    //         // 连续满足丢线条件，i 往后移
    //         while (i < checkEnd) {
    //             bool cond2 = false;
    //             if (isLeftLine) {
    //                 cond2 = (points[i].col < 5);
    //             } else {
    //                 cond2 = (points[i].col > 315);
    //             }
    //             if (!cond2) {
    //                 break;
    //             }
    //             i++;
    //         }

    //         int count = i - start; // 连续的长度
    //         if (count >= lostThreshold) {
    //             LostSegment seg;
    //             seg.startIdx = start;
    //             seg.endIdx   = i - 1; // i此时已指向不满足条件的下一个点
    //             lostSegments.push_back(seg);
    //         }
    //         // 注意这里没有 else { i++ }，因为 i 已经前进到区段末尾/下一个不满足的点
    //     } else {
    //         // 不满足丢线条件，简单 i++
    //         i++;
    //     }
    // }

    // //测试
    // std::cout<<"lostSegments.size: "<<lostSegments.size()<<std::endl;

    // //如果没有或只有一个丢线片段，就默认连续(返回true)
    // if (lostSegments.size() < 2) {
    //     return true;
    // }
    // //如果丢线太多则说明边线判定错误
    // if (lostSegments.size() > 5) {
    //     return true;
    // }


    // // 5) 判断相邻两个丢线片段之间的 row值差
    // for (int idx = 0; idx < (int)lostSegments.size() - 1; idx++) {
    //     const LostSegment &segA = lostSegments[idx];
    //     const LostSegment &segB = lostSegments[idx + 1];

    //     // segA 的最后一个点： points[segA.endIdx].row
    //     // segB 的第一个点   ： points[segB.startIdx].row
    //     int rowA = points[segA.endIdx].row;
    //     int rowB = points[segB.startIdx].row;

    //     if (std::abs(rowA - rowB) > 30) {
    //         // 大于30，认为不连续
    //         return false;
    //     }
    // }
    // return true;


    //////////////////////////////////////////////////////////////////////////////////////////////////////////

}

// 检查拐点的公共函数
int Ring::check_corner(std::vector<int> &dir, std::vector<cv::Point> &points,int &corner_index, int type1, int type2) {
    std::vector<int> suspect_corner;
    std::vector<std::pair<int,int>> line_type_and_count;
    int range_count = 5; 
    corner_index = 0; 

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
                        line_type_and_count.pop_back(); 
                        if(!suspect_corner.empty()){
                            suspect_corner.pop_back();  
                        }
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
                        line_type_and_count.pop_back(); 
                        if(!suspect_corner.empty()){
                            suspect_corner.pop_back();  
                        } 
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
                        line_type_and_count.pop_back(); 
                        if(!suspect_corner.empty()){
                            suspect_corner.pop_back(); 
                        }
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
            if(dir[i] ==4 || dir[i] ==5 ||dir[i] ==6){       //若中点的位置太靠后，则可以将4放在3类型里面
                //有可能是拐点的情况
                if(!line_type_and_count.empty() && line_type_and_count.back().first != 5){ 
                    if(line_type_and_count.back().second >range_count){
                        suspect_corner.emplace_back(i);
                        line_type_and_count.emplace_back(5,1);
                    }
                    else{
                        line_type_and_count.pop_back(); 
                        if(!suspect_corner.empty()){
                            suspect_corner.pop_back(); 
                        }
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
                        line_type_and_count.pop_back(); 
                        if(!suspect_corner.empty()){
                            suspect_corner.pop_back();  
                        }
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

            //0   
            if(dir[i] ==0 || dir[i] ==1 ||dir[i] ==7 ||dir[i] ==8){     
                //有可能是拐点的情况
                if(!line_type_and_count.empty() && line_type_and_count.back().first != 0){ 
                    if(line_type_and_count.back().second >range_count){
                        suspect_corner.emplace_back(i);
                        line_type_and_count.emplace_back(0,1);
                    }
                    else{
                        line_type_and_count.pop_back(); 
                        if(!suspect_corner.empty()){
                            suspect_corner.pop_back(); 
                        }
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
    //std::cout<<"line_type_and_count.size(): "<<line_type_and_count.size()<<std::endl;
    // std::cout<<"type:"<<type1<<std::endl;
    if(line_type_and_count.size() > 1 || line_type_and_count.size() - 1 == suspect_corner.size()) {
        for(size_t i = 0; i < line_type_and_count.size() - 1 && !suspect_corner.empty(); i++) {
            
            // std::cout<<i<<" line_type:"<<line_type_and_count[i].first<<std::endl;
            // std::cout<<i<<" line_count:"<<line_type_and_count[i].second<<std::endl;
            // std::cout<<i<<" angle: "<<angle<<std::endl;

            //防止出现反光点导致无法识别拐点
            if(line_type_and_count[i].first == type1
                && line_type_and_count[i].second >= 15
                && !find){
                for(int j = i + 1; j < line_type_and_count.size(); j++){
                    if( line_type_and_count[j].second >= 15 && suspect_corner.size()>j-1){
                        if(line_type_and_count[j].first == type2){
                            double angle = neighbour_points_angle(suspect_corner[j-1], points);
                            if(angle <120){
                                corner_index = suspect_corner[j-1];
                                find = true;                     
                            }                           
                        }
                        break;
                    }
                }
            }
        }
    }
    return find ? 1 : 0;
}
