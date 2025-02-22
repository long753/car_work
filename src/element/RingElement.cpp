#include "RingElement.h"
#include "Findline.h"
#include <opencv2/imgproc.hpp>
#include <cmath>
#include <iostream>


// 构造函数
Ring::Ring() { 
    int corner_up_index = 0;
    int corner_mid_index = 0;
    int corner_down_index = 0;
    int corner_edge_index = 0;
    int corner_exit_index = 0;
}


// 重置函数
void Ring::reset() {
    ringType = RingType::RingNone;
    ringStep = RingStep::None;
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

    corner_up_point = cv::Point(0, 0);
    corner_mid_point = cv::Point(0, 0);
    corner_down_point = cv::Point(0, 0);
    corner_edge_point = cv::Point(0, 0);
    corner_exit_point = cv::Point(0, 0);

}


// 圈搜索函数，目前没有利用陀螺仪积分来算角度，实在没时间写这么详细了，第二版可以尝试一下
void Ring::circle_search(Findline &findline, float angle) {
    if (image_to_enter(findline) == 1) {

        // 判断是否ringStep是否发生变化，同时对每一个阶段进行相应操作

        // 预入环
        if (ringStep == RingStep::PreEntering){

            //预入环操作
            if (ringType == RingType::RingRight){
                check_far_corner_right(findline);
                check_mid_corner_right(findline);
                check_near_corner_right(findline);
                get_up_corner(findline);
                get_mid_corner(findline);
                get_down_corner(findline);
                repair_line_prev(findline);
            }
            else if (ringType == RingType::RingLeft){
                check_far_corner_left(findline);
                check_mid_corner_left(findline);
                check_near_corner_left(findline);
                get_up_corner(findline);
                get_mid_corner(findline);
                get_down_corner(findline);
                repair_line_prev(findline);
            }

            // 判断是否更新
            if (get_mid_corner(findline) == 0 && get_up_corner(findline) == 1) {
                ringStep = RingStep::Entering;  
                Entering_flag = 1;       
            }
        }

        // 入环
        else if (ringStep == RingStep::Entering){

            //入环操作
            if (ringType == RingType::RingRight){
                // check_far_corner_right(findline);
                // get_up_corner(findline);
                repair_line_enter(findline);
            }
            else if (ringType == RingType::RingLeft){
                // check_far_corner_left(findline);
                // get_up_corner(findline);
                repair_line_enter(findline);
            }

            // 判断是否更新
            if (get_up_corner(findline) == 0 && Entering_flag == 1) {
                ringStep = RingStep::Inside;  
                inside_flag = 1;          
            }
        }

        // 环内
        else if (ringStep == RingStep::Inside){

            //环内操作
            if (ringType == RingType::RingRight){
                check_exit_corner_right(findline);
            }
            else if (ringType == RingType::RingLeft){
                check_exit_corner_left(findline);
            }

            // 判断是否更新
            if (check_exit_corner_left(findline) == 1 && inside_flag == 1) {
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
}


// 检查远处右转拐点
int Ring::check_far_corner_right(Findline &findline) {
    std::vector<int> suspect_right_corner;
    std::vector<std::pair<int,int>> right_line_type_and_count;
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

  	//校验是否符合要求
	bool RU_find = false;

   	if (right_line_type_and_count.size()>1 || right_line_type_and_count.size()-1 == suspect_right_corner.size()){
		for(size_t i =0 ;i<right_line_type_and_count.size()-1 && !suspect_right_corner.empty();i++){
			double angle = neighbour_points_angle(suspect_right_corner[i] ,findline.right_point);

			if(right_line_type_and_count[i].first ==6 &&
			(right_line_type_and_count[i+1].first ==4 || right_line_type_and_count[i+1].first ==2)
			&&right_line_type_and_count[i].second>=4 && right_line_type_and_count[i+1].second>=4
			&&RU_find == false ){
				if(40<=angle && angle<=120){
                    corner_up_index = suspect_right_corner[i];
                    // corner_up_update_count = corner_up_update_count + 1;
                    RU_find = true;
				}				
			}
		}
    }   
    if(RU_find == true){    //预留位置进行修改，暂时不用
        return 1;
    }
    return 0;
}


// 检查近处右转拐点
int Ring::check_near_corner_right(Findline &findline) {
    std::vector<int> suspect_right_corner;
    std::vector<std::pair<int,int>> right_line_type_and_count;
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

  	//校验是否符合要求
	bool RD_find = false;

   	if (right_line_type_and_count.size()>1 || right_line_type_and_count.size()-1 == suspect_right_corner.size()){
		for(size_t i =0 ;i<right_line_type_and_count.size()-1 && !suspect_right_corner.empty();i++){
			double angle = neighbour_points_angle(suspect_right_corner[i] ,findline.right_point);

			if(right_line_type_and_count[i].first ==4 &&
			(right_line_type_and_count[i+1].first ==2 || right_line_type_and_count[i+1].first ==6)
			&& right_line_type_and_count[i].second>=4 && right_line_type_and_count[i+1].second>=4
			&& RD_find == false ){
				if(40<=angle && angle<=120){
                    corner_up_index = suspect_right_corner[i];
                    // corner_up_update_count = corner_up_update_count + 1;
                    RD_find = true;
				}				
			}
		}
    }   
    if(RD_find == true){    //预留位置进行修改，暂时不用
        return 1;
    }
    return 0;
}

//检测右侧中拐点
int Ring::check_mid_corner_right(Findline & findline){
    std::vector<int> suspect_right_mid_corner;
    std::vector<std::pair<int,int>> right_line_mid_and_count;
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
	bool RM_find = false;
    if (right_line_mid_and_count.size()>1 || right_line_mid_and_count.size()-1 == suspect_right_mid_corner.size()){
		for(size_t i =0 ;i < right_line_mid_and_count.size()-1 && !suspect_right_mid_corner.empty();i++){
			double angle = neighbour_points_angle(suspect_right_mid_corner[i] ,findline.right_point);

			if(right_line_mid_and_count[i].first == 5 && right_line_mid_and_count[i+1].first == 3
			&&right_line_mid_and_count[i].second>=4 && right_line_mid_and_count[i+1].second>=4
			&&RM_find == false){
                if(40<=angle && angle<=80){ //这个要好好测试，我不知道，我不知道，我不知道，我不知道，我不知道，我不知道，我不知道
                    corner_mid_index = suspect_right_mid_corner[i];
                    // corner_mid_update_count = corner_mid_update_count + 1;
                    RM_find = true;  
                }
            }
        }
	} 
    if(RM_find == true){    //预留位置进行修改，暂时不用
        return 1;
    }
    return 0;   
}

// 检查右环出口拐点
int Ring::check_exit_corner_right(Findline &findline) {
    std::vector<int> suspect_left_corner;
    std::vector<std::pair<int,int>> left_line_type_and_count;

    for(size_t i = 0;i < findline.dir_r.size() ;i++){
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
    }

    //校验是否符合要求
   	bool LD_find =false;
   	if (left_line_type_and_count.size()>1 || left_line_type_and_count.size()-1 == suspect_left_corner.size()){
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

    if(LD_find == true){    //预留位置进行修改，暂时不用
        return 1;
    }
    return 0;
}


// 检查远处左转拐点
int Ring::check_far_corner_left(Findline &findline) {
    std::vector<int> suspect_left_corner;
    std::vector<std::pair<int,int>> left_line_type_and_count;

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

  	//校验是否符合要求
	bool LU_find = false;

   	if (left_line_type_and_count.size()>1 || left_line_type_and_count.size()-1 == suspect_left_corner.size()){
		for(size_t i =0 ;i<left_line_type_and_count.size()-1 && !suspect_left_corner.empty();i++){
			double angle = neighbour_points_angle(suspect_left_corner[i] ,findline.left_point);

			if(left_line_type_and_count[i].first ==6 &&
			(left_line_type_and_count[i+1].first ==4)
			&& left_line_type_and_count[i].second>=4&&left_line_type_and_count[i+1].second>=4
			&& LU_find == false ){
				if(40<=angle && angle<=120){
                    corner_up_index = suspect_left_corner[i];
                    // corner_up_update_count = corner_up_update_count + 1;
                    LU_find = true;
				}				
			}
		}
    } 
    if(LU_find == true){    //预留位置进行修改，暂时不用
        return 1;
    }
    return 0;

}


// 检查近处左转拐点
int Ring::check_near_corner_left(Findline &findline) {
    std::vector<int> suspect_left_corner;
    std::vector<std::pair<int,int>> left_line_type_and_count;
    int range_count = 4; // 阈值

	for(size_t i = 0;i < findline.dir_l.size() ;i++){

		//4
		if(findline.dir_l[i] ==4 ||findline.dir_l[i] ==5 ){ 
            //有可能是拐点的情况
			if(!left_line_type_and_count.empty() && left_line_type_and_count.back().first != 4){ 
				if(left_line_type_and_count.back().second >range_count){
				    suspect_left_corner.emplace_back(i);
                    left_line_type_and_count.emplace_back(4,1);
				}
                else{
                    for(int i = 0;i<left_line_type_and_count.back().second;i++) {
                        left_line_type_and_count.pop_back(); //把阈值内的点都删掉
                    }
					if(!left_line_type_and_count.empty()){
					    left_line_type_and_count.back().second++;
					}
				}
			}
            //还是直线的情况
			if(!left_line_type_and_count.empty()&& left_line_type_and_count.back().first ==4){
				left_line_type_and_count.back().second++;
			}
            //第一次
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

  	//校验是否符合要求
	bool LD_find =false;

   	if (left_line_type_and_count.size()>1 || left_line_type_and_count.size()-1 == suspect_left_corner.size()){
		for(size_t i =0 ;i<left_line_type_and_count.size()-1 && !suspect_left_corner.empty();i++){
			double angle = neighbour_points_angle(suspect_left_corner[i] ,findline.left_point);

			if(left_line_type_and_count[i].first ==4 &&left_line_type_and_count[i+1].first ==2
			&&left_line_type_and_count[i].second>=4&&left_line_type_and_count[i+1].second>=4
			&&LD_find == false){
                if(40<=angle && angle<=120){ //大约小于154度
                    corner_down_index = suspect_left_corner[i];
                    // corner_down_update_count = corner_down_update_count + 1;
                    LD_find = true;
                }
			}
		}
    } 
    if(LD_find == true){    //预留位置进行修改，暂时不用
        return 1;
    }
    return 0;
}

// 检查左侧中拐点
int Ring::check_mid_corner_left(Findline & findline){
    std::vector<int> suspect_left_mid_corner;  
    std::vector<std::pair<int,int>> left_line_mid_and_count;

    for(size_t i = 0;i < findline.dir_l.size() ;i++){
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
    bool LM_find = false;

   	if (left_line_mid_and_count.size()>1 || left_line_mid_and_count.size()-1 == suspect_left_mid_corner.size()){
		for(size_t i =0 ;i<left_line_mid_and_count.size()-1 && !suspect_left_mid_corner.empty();i++){
			double angle = neighbour_points_angle(suspect_left_mid_corner[i] ,findline.left_point);

			if(left_line_mid_and_count[i].first == 5 && left_line_mid_and_count[i+1].first == 3
			&& left_line_mid_and_count[i].second>=4 && left_line_mid_and_count[i+1].second>=4
			&& LM_find == false){
                if(40<=angle && angle<=80){ //这个要好好测试，我不知道，我不知道，我不知道，我不知道，我不知道，我不知道，我不知道
                    corner_mid_index = suspect_left_mid_corner[i];
                    // corner_mid_update_count = corner_mid_update_count + 1;
                    LM_find = true;
				}
			}
        }
    }  	

    if(LM_find == true){    //预留位置进行修改，暂时不用
        return 1;
    }
    return 0;
}

// 检查左环出口拐点
int Ring::check_exit_corner_left(Findline &findline) {
    std::vector<int> suspect_right_corner;
    std::vector<std::pair<int,int>> right_line_type_and_count;

    for(size_t i = 0;i < findline.dir_r.size() ;i++){
        //左上4
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

        //左2
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
    }

    //校验是否符合要求
   	bool RD_find =false;
   	if (right_line_type_and_count.size()>1 || right_line_type_and_count.size()-1 == suspect_right_corner.size()){
		for(size_t i =0 ;i<right_line_type_and_count.size()-1 && !suspect_right_corner.empty();i++){
			double angle = neighbour_points_angle(suspect_right_corner[i] ,findline.right_point);

			if(right_line_type_and_count[i].first == 4 && right_line_type_and_count[i+1].first ==2
			&&right_line_type_and_count[i].second >= 4 && right_line_type_and_count[i+1].second>=4
			&&RD_find == false){
                if(40<=angle && angle<=120){ 
                    //预留位置，暂时不用
                    corner_down_index = suspect_right_corner[i];
                    // corner_down_update_count = corner_down_update_count + 1;
                    RD_find = true;
                }
			}  
        }
    }  


    if(RD_find == true){  //预留位置进行修改，暂时不用
        return 1;
    }
    return 0;
}

// 获取上拐点
int Ring::get_up_corner(Findline &findline) {
    if (corner_up_index == 0 || corner_up_index >= findline.right_point.size() || corner_up_index >= findline.left_point.size()){
        return 0;
    }

    if (ringType == RingType::RingRight) {
        corner_up_point = findline.right_point[corner_up_index];
        return 1; 
    }
        
    else if (ringType == RingType::RingRight) {
        corner_up_point = findline.left_point[corner_up_index];
        return 1; 
    }
}

// 获取中拐点
int Ring::get_mid_corner(Findline &findline ) { 
    if (corner_mid_index == 0 || corner_mid_index >= findline.right_point.size() || corner_mid_index >= findline.left_point.size()){
        return 0;
    }

    if (ringType == RingType::RingRight) {
        corner_mid_point = findline.right_point[corner_mid_index];
        return 1; 
    }
    
    else if (ringType == RingType::RingRight) {
        corner_mid_point = findline.left_point[corner_mid_index];
        return 1; 
    }
}

// 获取下拐点
int Ring::get_down_corner(Findline &findline) {
    if (corner_down_index == 0 || corner_down_index >= findline.right_point.size() || corner_down_index >= findline.left_point.size()){
        return 0;
    }

    if (ringType == RingType::RingRight) {
        corner_down_point = findline.right_point[corner_down_index];
        return 1; 
    }
    
    else if (ringType == RingType::RingRight) {
        corner_down_point = findline.left_point[corner_down_index];
        return 1; 
    }
}

// 获取入环拐点
int Ring::get_edge_corner(Findline &findline) {
//     for (int i = 0; i < findline.pointsEdgeRight.size(); ++i) {
//         if (findline.pointsEdgeRight[i].y > 250) { // 假设阈值
//             corner_edge_point = findline.pointsEdgeRight[i];
//             corner_edge_update_count++;
//             if (corner_edge_update_count > 5) {
//                 corner_edge_update_count = 0;
//                 return 1;
//             }
//         }
//     }
    return 0;
}

// 获取出环拐点
int Ring::get_exit_corner(Findline &findline) {
//     for (int i = 0; i < findline.pointsEdgeLeft.size(); ++i) {
//         if (findline.pointsEdgeLeft[i].y < 50) { // 假设阈值
//             corner_exit_point = findline.pointsEdgeLeft[i];
//             corner_exit_update_count++;
//             if (corner_exit_update_count > 5) {
//                 corner_exit_update_count = 0;
//                 return 1;
//             }
//         }
//     }
    return 0;
}


// 检测圆环,判断入环方向
int Ring::image_to_enter(Findline &findline) {
    if (straight_line_judge(findline.dir_l,findline.left_point) ^ straight_line_judge(findline.dir_r,findline.right_point)) {
    // if (straight_line_judge(findline.dir_l) == straight_line_judge(findline.dir_r)) {
        ringStep = RingStep::PreEntering;
        // ringStep = RingStep::Entering;

        if (straight_line_judge(findline.dir_l,findline.left_point)){
            ringType = RingType::RingRight;
        }
        else{
            ringType = RingType::RingLeft;
        }
        return 1;  
    }
    //  std::cout<<"识别失败"<<std::endl;
    return 0;
}


// 修复预入环赛道线
void Ring::repair_line_prev(Findline &findline) {
    std::vector<cv::Point> new_right_point;
    std::vector<cv::Point> new_left_point;

    if (ringType == RingType::RingRight) {  
        for(int i = 1; i <= repairline_straight; ++i){

            double tx = (static_cast<double>(i) * (310 - corner_up_point.x)) / ((repairline_straight + 1) * (corner_mid_point.x - corner_up_point.x));
            double ty = (static_cast<double>(i) * (220 - corner_up_point.y)) / ((repairline_straight + 1) * (corner_mid_point.y - corner_up_point.y));
            double new_x = corner_up_point.x + tx * (corner_mid_point.x - corner_up_point.x);
            double new_y = corner_up_point.y + ty * (corner_mid_point.y - corner_up_point.y);
                
            cv::Point new_point(static_cast<int>(new_x), static_cast<int>(new_y));
            new_right_point.emplace_back(new_point);
        }
        findline.right_point.insert(findline.right_point.end(), new_right_point.begin(), new_right_point.end());
    }
    else if (ringType == RingType::RingLeft) {
        for(int i = 1; i <= repairline_straight; ++i){
            double tx = (static_cast<double>(i) * (10 - corner_up_point.x)) / ((repairline_straight + 1) * (corner_mid_point.x - corner_up_point.x));
            double ty = (static_cast<double>(i) * (220 - corner_up_point.y)) / ((repairline_straight + 1) * (corner_mid_point.y - corner_up_point.y));
            double new_x = corner_up_point.x + tx * (corner_mid_point.x - corner_up_point.x);
            double new_y = corner_up_point.y + ty * (corner_mid_point.y - corner_up_point.y);
                
            cv::Point new_point(static_cast<int>(new_x), static_cast<int>(new_y));
            new_left_point.emplace_back(new_point);
        }
        findline.left_point.insert(findline.left_point.end(), new_left_point.begin(), new_left_point.end());
    }
    findline.edge_calculate();
    findline.midline_calculate();
}


// 修复入环赛道线
void Ring::repair_line_enter(Findline &findline) {
    // std::vector<cv::Point> new_right_point;
    // std::vector<cv::Point> new_left_point;

    if (ringType == RingType::RingRight) {
        float b = 200;  
        float k = -1.0 / 3.0;  

        for (int i = 0; i < 30; ++i) {
            float x = (float)i * (320.0f / 29);  
            float y = k * x + b;  
            // new_right_point.emplace_back((int)x, (int)y);
            findline.right_point.emplace_back((int)x, (int)y);
        }

        // findline.right_point.insert(findline.right_point.end(), new_right_point.begin(), new_right_point.end());
    }
    else if (ringType == RingType::RingLeft) {
        float b = 200;  
        float k = 1.0 / 3.0;  

        for (int i = 0; i < 30; ++i) {
            float x = (float)i * (320.0f / 29);  
            float y = k * x + b;  
            // new_left_point.emplace_back((int)x, (int)y);
            findline.left_point.emplace_back((int)x, (int)y);
        }

        // findline.left_point.insert(findline.left_point.end(), new_left_point.begin(), new_left_point.end());
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
    int current_corner = 0;//最近的拐点序号
    int range_count = 5; // 阈值，超过这个阈值认为是直线

    //调试信息
    // std::cout<<dir.size()<<std::endl; 
    // std::cout<<line_type_and_count.size()<<std::endl;

	for(size_t i = 0;i < dir.size() ;i++){

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

    //计算拐点个数
   	if (line_type_and_count.size()>1 || line_type_and_count.size()-1 == suspect_corner.size()){    
		for(size_t i =0 ;i<line_type_and_count.size()-1 && !suspect_corner.empty();i++){
			double angle = neighbour_points_angle(suspect_corner[i] , points);

			if(line_type_and_count[i].first ==4 &&line_type_and_count[i+1].first ==2
			&&line_type_and_count[i].second>=4&&line_type_and_count[i+1].second>=4
			&& angle<=120){
                if(current_corner == 0){ 
                    current_corner = suspect_corner[i];
                    guai_count = guai_count+1;
                }
                else{
                    if(suspect_corner[i] - current_corner > 20){
                        current_corner = suspect_corner[i];
                        guai_count = guai_count+1;
                    }
                }
			}
		}
    }

    std::cout<<suspect_corner.size()<<std::endl;
    std::cout<<guai_count<<std::endl;
 
    if(guai_count >= 2){   
        return false;
    }
    return true;
}