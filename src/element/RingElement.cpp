#include "RingElement.h"
#include "Findline.h"
#include <opencv2/imgproc.hpp>
#include <cmath>
#include <iostream>


// 构造函数
Ring::Ring() { 
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

        // 判断是否ringStep是否发生变化，同时对每一个阶段进行相应操作

        // 预入环
        if (ringStep == RingStep::PreEntering){

            //预入环操作
            check_corner(findline);
            int find_up_corner = get_up_corner(findline);
            int find_mid_corner = get_mid_corner(findline);
            int find_down_corner = get_down_corner(findline);
            repair_line_prev(findline);

            // 判断是否更新
            if (find_mid_corner == 0 && find_up_corner == 1) {
                ringStep = RingStep::Entering;  
                Entering_flag = 1;       
            }
        }

       // 入环
        else if (ringStep == RingStep::Entering){

            //入环操作
            check_corner(findline);
            int find_up_corner = get_up_corner(findline);
            repair_line_enter(findline);

            // 判断是否更新
            if (find_up_corner == 0 && Entering_flag == 1) {
                ringStep = RingStep::Inside;  
                inside_flag = 1;          
            }
        }

        // 环内
        else if (ringStep == RingStep::Inside){

            //环内操作
            check_corner(findline);
            if (ringType == RingType::RingRight){
                int find_eixt_corner = check_exit_corner_right(findline);
            }
            else if (ringType == RingType::RingLeft){
                int find_eixt_corner = check_exit_corner_left(findline);
            }

            // 判断是否更新
            if (find_eixt_corner == 1 && inside_flag == 1) {
                ringStep = RingStep::Exiting;  
                exit_flag = 1;  
            }
        }

        // 出环
        else if (ringStep == RingStep::Exiting){

            //出环操作
            //check_corner(findline);
            //int find_up_corner = get_up_corner(findline);
            repair_line_exit(findline);

            // 判断是否更新
            if(ringType == RingType::RingRight && straight_line_judge(findline.dir_l) && exit_flag == 1){
                ringStep = RingStep::Finish;  
                reset();
            }
            if(ringType == RingType::RingLeft && straight_line_judge(findline.dir_r) && exit_flag == 1){
                ringStep = RingStep::Finish;  
                reset();
            }
        }

        // 结束,考虑删除此状态，似乎影响不大
        else if (ringStep == RingStep::Finish){

            //结束操作
            check_corner(findline);
            int find_up_corner = get_up_corner(findline);
            repair_line_finish(findline);

            // 判断是否更新
            if (straight_line_judge(findline.dir_l) && straight_line_judge(findline.dir_r)) {
                reset();
            }
        }
    }
}


//检擦拐点,只包含了进入圆环时的拐点检测，出圆环的那个拐点有点诡异，需要单独写。效率不高，需要重新写
int Ring::check_corner(Findline &findline) {
    std::vector<int> suspect_right_corner;
    std::vector<int> suspect_left_corner;
    std::vector<int> suspect_right_mid_corner;
    std::vector<int> suspect_left_mid_corner;
    std::vector<std::pair<int,int>> left_line_type_and_count;
    std::vector<std::pair<int,int>> right_line_type_and_count;   
    std::vector<std::pair<int,int>> left_line_mid_and_count;
    std::vector<std::pair<int,int>> right_line_mid_and_count;

    //右侧 
	for(size_t i = 0;i < findline.dir_r.size() ;i++){

		//4
		if(findline.dir_r[i] ==4 ||findline.dir_r[i] ==5 ){ 
		    if(!right_line_type_and_count.empty() && right_line_type_and_count.back().first != 4){ 
			    if(right_line_type_and_count.back().second >1){
			        suspect_right_corner.emplace_back(i);
			    }
                else{
			        right_line_type_and_count.pop_back(); 
			        if(!right_line_type_and_count.empty()){
				        right_line_type_and_count.back().second++;
			        }

			    if(!suspect_right_corner.empty()){
				    suspect_right_corner.pop_back();
			    }
			    }
		    }
		    if(!right_line_type_and_count.empty()&& right_line_type_and_count.back().first ==4){
			    right_line_type_and_count.back().second++;
		    }
            else {
			    right_line_type_and_count.emplace_back(4,1); 
		    }
		}

		// 2
		else if(findline.dir_r[i] ==2 ||findline.dir_r[i] ==3 ||  findline.dir_r[i] ==1){
		    if(!right_line_type_and_count.empty() && right_line_type_and_count.back().first!=2){
			    if(right_line_type_and_count.back().second >1){
			        suspect_right_corner.emplace_back(i);
			    }
                else{
			        right_line_type_and_count.pop_back(); 
			        if(!right_line_type_and_count.empty()){
				        right_line_type_and_count.back().second++; 
			        }
			        if(!suspect_right_corner.empty()){
				        suspect_right_corner.pop_back();
			        }
			    }
		    }
            if(!right_line_type_and_count.empty()&& right_line_type_and_count.back().first ==2){
                right_line_type_and_count.back().second++;
            }
            else {
                right_line_type_and_count.emplace_back(2,1); 
            }

		}

		//6
		else if(findline.dir_r[i] == 6 || findline.dir_r[i] == 7){
            if(!right_line_type_and_count.empty() && right_line_type_and_count.back().first!=6){ 
                if(right_line_type_and_count.back().second >1){
                    suspect_right_corner.emplace_back(i);
                }
                else{
                    right_line_type_and_count.pop_back(); 
                    if(!right_line_type_and_count.empty()){
                        right_line_type_and_count.back().second++; 
                    }
                    if(!suspect_right_corner.empty()){
                        suspect_right_corner.pop_back();
                    }

                }
            }
            if(!right_line_type_and_count.empty()&& right_line_type_and_count.back().first ==6){
                right_line_type_and_count.back().second++;
            }
            else {
                right_line_type_and_count.emplace_back(6,1); 
            }
		}
    }

    for(size_t i = 0;i < findline.dir_r.size() ;i++){
        //左偏5
		if(findline.dir_r[i] == 5 || findline.dir_r[i] == 6 || findline.dir_r[i] == 7){
            if(!right_line_mid_and_count.empty() && right_line_mid_and_count.back().first!=5){ 
                if(right_line_mid_and_count.back().second >1){
                    suspect_right_mid_corner.emplace_back(i);
                }
                else{
                    right_line_mid_and_count.pop_back(); 
                    if(!right_line_mid_and_count.empty()){
                        right_line_mid_and_count.back().second++; 
                    }
                    if(!suspect_right_mid_corner.empty()){
                        suspect_right_mid_corner.pop_back();
                    }

                }
            }
            if(!right_line_mid_and_count.empty()&& right_line_mid_and_count.back().first ==5){
                right_line_mid_and_count.back().second++;
            }
            else {
                right_line_mid_and_count.emplace_back(5,1); 
            }
		}

        //右偏3  4放在这里可以使中拐点偏下，可以更快的判定圆环，防止没有检测到圆环
		else if(findline.dir_r[i] == 1 || findline.dir_r[i] == 2 || findline.dir_r[i] == 3 || findline.dir_r[i] == 4){
            if(!right_line_mid_and_count.empty() && right_line_mid_and_count.back().first!=3){ 
                if(right_line_mid_and_count.back().second >1){
                    suspect_right_mid_corner.emplace_back(i);
                }
                else{
                    right_line_mid_and_count.pop_back(); 
                    if(!right_line_mid_and_count.empty()){
                        right_line_mid_and_count.back().second++; 
                    }
                    if(!suspect_right_mid_corner.empty()){
                        suspect_right_mid_corner.pop_back();
                    }

                }
            }
            if(!right_line_mid_and_count.empty()&& right_line_mid_and_count.back().first ==3){
                right_line_mid_and_count.back().second++;
            }
            else {
                right_line_mid_and_count.emplace_back(3,1); 
            }
        }
    }

  	//校验是否符合要求
	bool RD_find =false;
	bool RU_find = false;
    bool RM_find = false;

   	if (right_line_type_and_count.size()>1){
		for(size_t i =0 ;i<right_line_type_and_count.size()-1 && !suspect_right_corner.empty();i++){
			double angle = neighbour_points_angle(suspect_right_corner[i] ,findline.right_point);

			if(right_line_type_and_count[i].first ==4 && right_line_type_and_count[i+1].first ==2
			&&right_line_type_and_count[i].second>=4 && right_line_type_and_count[i+1].second>=4
			&&RD_find == false){
			if(40<=angle && angle<=120){ //大约小于154度
				ring::corner_down_index = suspect_right_corner[i];
                ring::corner_down_update_count = ring::corner_down_update_count + 1;
				RD_find = true;
				}
			}
			else if(right_line_type_and_count[i].first ==6 &&
			(right_line_type_and_count[i+1].first ==4 || right_line_type_and_count[i+1].first ==2)
			&&right_line_type_and_count[i].second>=4 && right_line_type_and_count[i+1].second>=4
			&&RU_find == false ){
				if(40<=angle && angle<=120){
                    ring::corner_up_index = suspect_right_corner[i];
                    ring::corner_up_update_count = ring::corner_up_update_count + 1;
                    RU_find = true;
				}				
			}
		}
    }      
   	if (right_line_mid_and_count.size()>1){
		for(size_t i =0 ;i < right_mid_type_and_count.size()-1 && !suspect_right_mid_corner.empty();i++){
			double angle = neighbour_points_angle(suspect_right_mid_corner[i] ,findline.right_point);

			if(right_line_mid_and_count[i].first == 5 && right_line_mid_and_count[i+1].first == 3
			&&right_line_mid_and_count[i].second>=4 && right_line_mid_and_count[i+1].second>=4
			&&RM_find == false){
                if(40<=angle && angle<=80){ //这个要好好测试，我不知道，我不知道，我不知道，我不知道，我不知道，我不知道，我不知道
                    ring::corner_mid_index = suspect_right_mid_corner[i];
                    ring::corner_mid_update_count = ring::corner_mid_update_count + 1;
                    RM_find = true;  
                }
            }
        }
	}    


    //左侧
	for(size_t i = 0;i < findline.dir_l.size() ;i++){

		//4
		if(findline.dir_l[i] ==4 ||findline.dir_l[i] ==5 ){ 
			if(!left_line_type_and_count.empty() && left_line_type_and_count.back().first != 4){ 
				if(left_line_type_and_count.back().second >1){
				    suspect_left_corner.emplace_back(i);
				}
                else{
					left_line_type_and_count.pop_back(); 
					if(!left_line_type_and_count.empty()){
					    left_line_type_and_count.back().second++;
					}
					if(!suspect_left_corner.empty()){
						suspect_left_corner.pop_back();
					}

				}
			}
			if(!left_line_type_and_count.empty()&& left_line_type_and_count.back().first ==4){
				left_line_type_and_count.back().second++;
			}
            else {
				left_line_type_and_count.emplace_back(4,1); 
			}
		}

		// 2
		else if(findline.dir_l[i] ==2 ||findline.dir_l[i] ==3 ||  findline.dir_l[i] ==1){
            if(!left_line_type_and_count.empty() && left_line_type_and_count.back().first!=2){
                if(left_line_type_and_count.back().second >1){
                    suspect_left_corner.emplace_back(i);
                    left_line_type_and_count.back().second++;
                }
                else{
                    left_line_type_and_count.pop_back(); 
                    if(!left_line_type_and_count.empty()){
                        left_line_type_and_count.back().second++;
                    }
                    if(!suspect_left_corner.empty()){
                        suspect_left_corner.pop_back();
                    }
                }
            }
            if(!left_line_type_and_count.empty()&& left_line_type_and_count.back().first ==2){
                left_line_type_and_count.back().second++;
            }
            else {
                left_line_type_and_count.emplace_back(2,1); 
            }

		}

		//6
		else if(findline.dir_l[i] == 6 || findline.dir_l[i] == 7){
			if(!left_line_type_and_count.empty() && left_line_type_and_count.back().first!=6){ 
				if(left_line_type_and_count.back().second >1){
				    suspect_left_corner.emplace_back(i);
				}
                else{
				    left_line_type_and_count.pop_back(); 
                    if(!left_line_type_and_count.empty()){
                        left_line_type_and_count.back().second++;
                    }
                    if(!suspect_left_corner.empty()){
                        suspect_left_corner.pop_back();
                    }
				}
			}
			if(!left_line_type_and_count.empty()&& left_line_type_and_count.back().first ==6){
				left_line_type_and_count.back().second++;
			}
            else {
				left_line_type_and_count.emplace_back(6,1); 
			}
		}
    }

    for(size_t i = 0;i < findline.dir_r.size() ;i++){
        // 右偏5
        if(findline.dir_l[i] == 5 || findline.dir_l[i] == 6 || findline.dir_l[i] == 7){
            if(!left_line_mid_and_count.empty() && left_line_mid_and_count.back().first != 5){ 
                if(left_line_mid_and_count.back().second > 1){
                    suspect_left_mid_corner.emplace_back(i);
                }
                else{
                    left_line_mid_and_count.pop_back(); 
                    if(!left_line_mid_and_count.empty()){
                        left_line_mid_and_count.back().second++; 
                    }
                    if(!suspect_left_mid_corner.empty()){
                        suspect_left_mid_corner.pop_back();
                    }
                }
            }
            if(!left_line_mid_and_count.empty() && left_line_mid_and_count.back().first == 5){
                left_line_mid_and_count.back().second++;
            }
            else {
                left_line_mid_and_count.emplace_back(5, 1); 
            }
        }

        // 左偏3
        else if(findline.dir_l[i] == 1 || findline.dir_l[i] == 2 || findline.dir_l[i] == 3 || findline.dir_l[i] == 4){
            if(!left_line_mid_and_count.empty() && left_line_mid_and_count.back().first != 3){ 
                if(left_line_mid_and_count.back().second > 1){
                    suspect_left_mid_corner.emplace_back(i);
                }
                else{
                    left_line_mid_and_count.pop_back(); 
                    if(!left_line_mid_and_count.empty()){
                        left_line_mid_and_count.back().second++; 
                    }
                    if(!suspect_left_mid_corner.empty()){
                        suspect_left_mid_corner.pop_back();
                    }
                }
            }
            if(!left_line_mid_and_count.empty() && left_line_mid_and_count.back().first == 3){
                left_line_mid_and_count.back().second++;
            }
            else {
                left_line_mid_and_count.emplace_back(3, 1); 
            }
        }
    }

  	//校验是否符合要求
	bool LD_find =false;
	bool LU_find = false;
    bool LM_find = false;

   	if (left_line_type_and_count.size()>1){
		for(size_t i =0 ;i<left_line_type_and_count.size()-1 && !suspect_left_corner.empty();i++){
			double angle = neighbour_points_angle(suspect_left_corner[i] ,findline.left_point);

			if(left_line_type_and_count[i].first ==4 &&left_line_type_and_count[i+1].first ==2
			&&left_line_type_and_count[i].second>=4&&left_line_type_and_count[i+1].second>=4
			&&LD_find == false){
			if(40<=angle && angle<=120){ //大约小于154度
				ring::corner_down_index = suspect_left_corner[i];
                ring::corner_down_update_count = ring::corner_down_update_count + 1;
				LD_find = true;
				}
			}
			else if(left_line_type_and_count[i].first ==6 &&
			(left_line_type_and_count[i+1].first ==4 ||left_line_type_and_count[i+1].first ==2)
			&&left_line_type_and_count[i].second>=4&&left_line_type_and_count[i+1].second>=4
			&&LU_find == false ){
				if(40<=angle && angle<=120){
                    ring::corner_up_index = suspect_left_corner[i];
                    ring::corner_up_update_count = ring::corner_up_update_count + 1;
                    LU_find = true;
				}				
			}
		}
    }      
   	if (left_line_mid_and_count.size()>1){
		for(size_t i =0 ;i<left_mid_type_and_count.size()-1 && !suspect_left_mid_corner.empty();i++){
			double angle = neighbour_points_angle(suspect_left_mid_corner[i] ,findline.left_point);

			if(left_line_mid_and_count[i].first == 5 && left_line_mid_and_count[i+1].first == 3
			&&left_line_mid_and_count[i].second>=4&&left_line_mid_and_count[i+1].second>=4
			&&LM_find == false){
                if(40<=angle && angle<=80){ //这个要好好测试，我不知道，我不知道，我不知道，我不知道，我不知道，我不知道，我不知道
                    ring::corner_mid_index = suspect_left_mid_corner[i];
                    ring::corner_mid_update_count = ring::corner_mid_update_count + 1;
                    LM_find = true;
				}
			}
        }
    }  	
}

// 检查远处右转拐点
int check_far_corner_right(Findline &findline) {
    if ring::corner_up_index != 0 && ring::corner_up_update_count ==1 {
        if (ringType == RingRight) {
            return 1;
        }
    }
    return 0;
}


// 检查近处右转拐点
int Ring::check_near_corner_right(Findline &findline) {
    if ring::corner_down_index != 0 && ring::corner_down_update_count ==1 {
        if (ringType == RingRight) {
            return 1;
        }
    }
    return 0;
}

//检测右侧中拐点
int check_mid_corner_right(Findline & findline){
    if ring::corner_mid_index != 0 && ring::corner_mid_update_count ==1 {
        if (ringType == RingRight) {
            return 1;
        }
    }
    return 0;
}

// 检查右环出口拐点
int Ring::check_exit_corner_right(Findline &findline) {
    std::vector<int> suspect_left_corner
    std::vector<std::pair<int,int>> left_line_type_and_count;

	//右上
	if(findline.dir_l[i] ==4 || findline.dir_l[i] ==5 || findline.dir_l[i] ==6){ 
		if(!left_line_type_and_count.empty() && left_line_type_and_count.back().first != 4){ 
			if(left_line_type_and_count.back().second >1){
			    suspect_left_corner.emplace_back(i);
			}
            else{
				left_line_type_and_count.pop_back(); 
				if(!left_line_type_and_count.empty()){
				    left_line_type_and_count.back().second++;
				}
				if(!suspect_left_corner.empty()){
					suspect_left_corner.pop_back();
				}
			}
		}
		if(!left_line_type_and_count.empty()&& left_line_type_and_count.back().first ==4){
			left_line_type_and_count.back().second++;
		}
        else {
			left_line_type_and_count.emplace_back(4,1); 
		}
	}

    //左
	if(findline.dir_l[i] ==1 || findline.dir_l[i] ==2 || findline.dir_l[i] ==3){ 
		if(!left_line_type_and_count.empty() && left_line_type_and_count.back().first != 2){ 
			if(left_line_type_and_count.back().second >1){
			    suspect_left_corner.emplace_back(i);
			}
            else{
				left_line_type_and_count.pop_back(); 
				if(!left_line_type_and_count.empty()){
				    left_line_type_and_count.back().second++;
				}
				if(!suspect_left_corner.empty()){
					suspect_left_corner.pop_back();
				}

			}
		}
		if(!left_line_type_and_count.empty()&& left_line_type_and_count.back().first ==2){
			left_line_type_and_count.back().second++;
		}
        else {
			left_line_type_and_count.emplace_back(2,1); 
		}
	}

    //校验是否符合要求
   	bool LD_find =false;
   	if (left_line_type_and_count.size()>1){
		for(size_t i =0 ;i<left_line_type_and_count.size()-1 && !suspect_left_corner.empty();i++){
			double angle = neighbour_points_angle(suspect_left_corner[i] ,findline.left_point);

			if(left_line_type_and_count[i].first ==4 &&left_line_type_and_count[i+1].first ==2
			&&left_line_type_and_count[i].second>=4&&left_line_type_and_count[i+1].second>=4
			&&LD_find == false){
                if(40<=angle && angle<=120){ 
                    //预留位置，暂时不用
                    // ring::corner_down_index = suspect_left_corner[i];
                    // ring::corner_down_update_count = ring::corner_down_update_count + 1;
                    LD_find = true;
                }
			}  
        }
    }  

    if(LD_find = true){    //预留位置进行修改，暂时不用
        return 1;
    }
    return 0;
}


// 检查远处左转拐点
int Ring::check_far_corner_left(Findline &findline) {
    if ring::corner_up_index != 0 && ring::corner_up_update_count ==1 {
        if (ringType == RingLeft) {
            return 1;
        }
    }
    return 0;
}


// 检查近处左转拐点
int Ring::check_near_corner_left(Findline &findline) {
    if ring::corner_down_index != 0 && ring::corner_down_update_count ==1 {
        if (ringType == RingLeft) {
            return 1;
        }
    }
    return 0;
}

// 检查左侧中拐点
int check_mid_corner_left(Findline & findline){
    if ring::corner_mid_index != 0 && ring::corner_mid_update_count ==1 {
        if (ringType == RingLeft) {
            return 1;
        }
    }
    return 0;
}

// 检查左环出口拐点
int Ring::check_exit_corner_left(Findline &findline) {
    std::vector<int> suspect_right_corner
    std::vector<std::pair<int,int>> right_line_type_and_count;

	//左上
	if(findline.dir_r[i] ==4 || findline.dir_r[i] ==5 || findline.dir_r[i] ==6){ 
		if(!right_line_type_and_count.empty() && right_line_type_and_count.back().first != 4){ 
			if(right_line_type_and_count.back().second >1){
			    suspect_right_corner.emplace_back(i);
			}
            else{
				right_line_type_and_count.pop_back(); 
				if(!right_line_type_and_count.empty()){
				    right_line_type_and_count.back().second++;
				}
				if(!suspect_right_corner.empty()){
					suspect_right_corner.pop_back();
				}
			}
		}
		if(!right_line_type_and_count.empty() && right_line_type_and_count.back().first ==4){
			right_line_type_and_count.back().second++;
		}
        else {
			right_line_type_and_count.emplace_back(4,1); 
		}
	}

    //左
	if(findline.dir_r[i] ==1 || findline.dir_r[i] ==2 || findline.dir_r[i] ==3){ 
		if(!right_line_type_and_count.empty() && right_line_type_and_count.back().first != 2){ 
			if(right_line_type_and_count.back().second >1){
			    suspect_right_corner.emplace_back(i);
			}
            else{
				right_line_type_and_count.pop_back(); 
				if(!right_line_type_and_count.empty()){
				    right_line_type_and_count.back().second++;
				}
				if(!suspect_right_corner.empty()){
					suspect_right_corner.pop_back();
				}

			}
		}
		if(!right_line_type_and_count.empty()&& right_line_type_and_count.back().first ==2){
			right_line_type_and_count.back().second++;
		}
        else {
			right_line_type_and_count.emplace_back(2,1); 
		}
	}

    //校验是否符合要求
   	bool RD_find =false;
   	if (right_line_type_and_count.size()>1){
		for(size_t i =0 ;i<right_line_type_and_count.size()-1 && !suspect_right_corner.empty();i++){
			double angle = neighbour_points_angle(suspect_right_corner[i] ,findline.right_point);

			if(right_line_type_and_count[i].first == 4 && right_line_type_and_count[i+1].first ==2
			&&right_line_type_and_count[i].second >= 4 && right_line_type_and_count[i+1].second>=4
			&&RD_find == false){
                if(40<=angle && angle<=120){ 
                    //预留位置，暂时不用
                    // ring::corner_down_index = suspect_left_corner[i];
                    // ring::corner_down_update_count = ring::corner_down_update_count + 1;
                    RD_find = true;
                }
			}  
        }
    }  

    if(RD_find = true){    //预留位置进行修改，暂时不用
        return 1;
    }
    return 0;
}

// 获取上拐点
int Ring::get_up_corner(Findline &findline) {
    if (corner_up_index != 0) {
        
        if (findline.ringType == RingRight) {
            corner_up_point = findline.right_point[corner_up_index];
            return 1; 
        }
        
        else if (findline.ringType == RingLeft) {
            corner_up_point = findline.left_point[corner_up_index];
            return 1; 
        }
    }
    return 0;
}

// 获取中拐点
int Ring::get_mid_corner(Findline &findline) {
    if (corner_mid_index != 0) {
        
        if (findline.ringType == RingRight) {
            corner_mid_point = findline.right_point[corner_mid_index];
            return 1; 
        }
        
        else if (findline.ringType == RingLeft) {
            corner_mid_point = findline.left_point[corner_mid_index];
            return 1; 
        }
    }
    return 0;
}

// 获取下拐点
int Ring::get_down_corner(Findline &findline) {
    if (corner_down_index != 0) {
        
        if (findline.ringType == RingRight) {
            corner_down_point = findline.right_point[corner_down_index];
            return 1; 
        }
        
        else if (findline.ringType == RingLeft) {
            corner_down_point = findline.left_point[corner_down_index];
            return 1; 
        }
    }
    return 0;
}

// 获取入环拐点
int Ring::get_edge_corner(Findline &findline) {
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
    std::vector<cv::Point> new_points;

    for(int i = 1; i <= repairline_straight; ++i){

        double tx = (static_cast<double>(i) * (300 - corner_up_point.x)) / ((repairline_straight + 1) * (corner_mid_point.x - corner_up_point.x));
        double ty = (static_cast<double>(i) * (220 - corner_up_point.y)) / ((repairline_straight + 1) * (corner_mid_point.y - corner_up_point.y));
        double new_x = corner_up_point.x + tx * (corner_mid_point.x - corner_up_point.x);
        double new_y = corner_up_point.y + ty * (corner_mid_point.y - corner_up_point.y);
            
        cv::Point new_point(static_cast<int>(new_x), static_cast<int>(new_y));
        new_points.emplace_back(new_point);
    }

    if (ringType == RingType::RingRight) {                          
        findline.right_point.insert(findline.right_point.end(), new_points.begin(), new_points.end());
    }
    else {                                                           
        findline.left_point.insert(findline.left_point.end(), new_points.begin(), new_points.end());
    }

    findline.edge_calculate();
    findline.midline_calculate();
}


// 修复入环赛道线
void Ring::repair_line_enter(Findline &findline) {
    std::vector<cv::Point> new_right_point;
    std::vector<cv::Point> new_left_point;

    if (ringType == RingType::RingRight) {
        float b = 200;  
        float k = -1.0 / 3.0;  

        for (int i = 0; i < 30; ++i) {
            float x = (float)i * (320.0f / 29);  
            float y = k * x + b;  
            new_right_point.emplace_back((int)x, (int)y);
        }

        findline.right_point.insert(findline.right_point.end(), new_right_point.begin(), new_right_point.end());
    }
    else if (ringType == RingType::RingLeft) {
        float b = 200;  
        float k = 1.0 / 3.0;  

        for (int i = 0; i < 30; ++i) {
            float x = (float)i * (320.0f / 29);  
            float y = k * x + b;  
            new_left_point.emplace_back((int)x, (int)y);
        }

        findline.left_point.insert(findline.left_point.end(), new_left_point.begin(), new_left_point.end());
    }

    findline.edge_calculate();
    findline.midline_calculate();
}



// 修复出环赛道线
void Ring::repair_line_exit(Findline &findline) {
    std::vector<cv::Point> new_right_point;
    std::vector<cv::Point> new_left_point;

    if (ringType == RingType::RingRight) {
        float b = 200;  
        float k = -1.0 / 3.0;  

        for (int i = 0; i < 30; ++i) {
            float x = (float)i * (320.0f / 29);  
            float y = k * x + b;  
            new_right_point.emplace_back((int)x, (int)y);
        }

        findline.right_point.insert(findline.right_point.end(), new_right_point.begin(), new_right_point.end());
    }
    else if (ringType == RingType::RingLeft) {
        float b = 200;  
        float k = 1.0 / 3.0;  

        for (int i = 0; i < 30; ++i) {
            float x = (float)i * (320.0f / 29);  
            float y = k * x + b;  
            new_left_point.emplace_back((int)x, (int)y);
        }

        findline.left_point.insert(findline.left_point.end(), new_left_point.begin(), new_left_point.end());
    }

    findline.edge_calculate();
    findline.midline_calculate();
}


// 修复结束时的赛道线
void Ring::repair_line_finish(Findline &findline) {
    std::vector<cv::Point> new_right_point;
    std::vector<cv::Point> new_left_point;

    if (ringType == RingType::RingRight) {                            // 右入环：
        for(int i = 1; i <= repairline_straight; ++i){

            double t = static_cast<double>(i) / (repairline_straight + 1);
            double new_x = corner_mid_point.x + t * (corner_up_point.x - corner_mid_point.x);
            double new_y = corner_mid_point.y + t * (corner_up_point.y - corner_mid_point.y);
            
            cv::Point new_point(static_cast<int>(int(new_x)), static_cast<int>(int(new_y)));
            new_right_point.emplace_back(new_point);
        }
        // 将生成的新点添加到findline.right_point
        findline.right_point.insert(pointsEdgeRight.end(), new_right_point.begin(), new_right_point.end());
        findline.edge_calculate();
        findline.midline_calculate();
    }
    else {                                                              // 左入环：
        for(int i = 1; i <= repairline_straight; ++i){
            double t = static_cast<double>(i) / (repairline_straight + 1);
            
            double new_x = corner_mid_point.x + t * (corner_up_point.x - corner_mid_point.x);
            double new_y = corner_mid_point.y + t * (corner_up_point.y - corner_mid_point.y);
            
            cv::Point new_point(static_cast<int>(new_x), static_cast<int>(new_y));
            new_left_point.emplace_back(new_point);
        }
        // 将生成的新点添加到findline.left_point
        findline.left_point.insert(pointsEdgeLeft.end(), new_left_point.begin(), new_left_point.end());
        findline.edge_calculate();
        findline.midline_calculate();
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
