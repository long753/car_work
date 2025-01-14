#include "CrossElement.h"
#include <vector>
void Cross::reset(void) {
  crossroadType = CrossroadType::None; // 十字道路类型
}

/**
 * @brief 十字道路识别与图像处理
 *
 * @param track 赛道识别结果
 * @param imagePath 输入图像
 */
bool Cross::cross_search(Findline &findline){

	crossroadType = CrossroadType::None; // 十字道路类型
	pointBreakLU = cv::Point(0, 0);
	pointBreakLD = cv::Point(0, 0);
	pointBreakRU = cv::Point(0, 0);
	pointBreakRD = cv::Point(0, 0);
	
	std::vector<int> suspect_left_corner; //可疑的左拐点索引
	std::vector<int> suspect_right_corner; // 可疑的右拐点

	//正入十字判断

	//寻找左上和左下拐点
	std::vector<std::pair<int,int>> left_line_type_and_count;// 左边点类型和计数
	for(size_t i = 0;i < findline.dir_l.size() ;i++){
		//4
		if(findline.dir_l[i] ==4 ||findline.dir_l[i] ==5 ){ //判断是直线
			if(!left_line_type_and_count.empty() && left_line_type_and_count.back().first != 4){ //前一个处理
				if(left_line_type_and_count.back().second >1){
				suspect_left_corner.emplace_back(i);
				}else{
					left_line_type_and_count.pop_back(); //弹出最后一个
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
			}else 
			{
				left_line_type_and_count.emplace_back(4,1); //4或5都用4代表
			}
		}
		// 2
		else if(findline.dir_l[i] ==2 ||findline.dir_l[i] ==3 ||  findline.dir_l[i] ==1){
		if(!left_line_type_and_count.empty() && left_line_type_and_count.back().first!=2){
			if(left_line_type_and_count.back().second >1){
			suspect_left_corner.emplace_back(i);
			left_line_type_and_count.back().second++;
			}else{
			left_line_type_and_count.pop_back(); //弹出最后一个
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
		}else {
			left_line_type_and_count.emplace_back(2,1); //2或3都用2代表
		}

		}
		//6
		else if(findline.dir_l[i] == 6 || findline.dir_l[i] == 7){
			if(!left_line_type_and_count.empty() && left_line_type_and_count.back().first!=6){ //保存跳变点
				if(left_line_type_and_count.back().second >1){
				suspect_left_corner.emplace_back(i); // 压入索引
				}else{
				left_line_type_and_count.pop_back(); //弹出只有一个的线
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
			}else {
				left_line_type_and_count.emplace_back(6,1); //2或3都用2代表
			}
		}
  	}
	//校验是否符合要求
	bool LD_find =false;
	bool LU_find = false;
	// std::cout<<"test:"<<left_line_type_and_count.size()<<std::endl;
   	if (left_line_type_and_count.size()>1){
		for(size_t i =0 ;i<left_line_type_and_count.size()-1 && !suspect_left_corner.empty();i++){
			double angle = neighbour_points_angle(suspect_left_corner[i] ,findline.left_point);
			// std::cout << "leftangle" << angle << "\n";
			if(left_line_type_and_count[i].first ==4 &&left_line_type_and_count[i+1].first ==2
			&&left_line_type_and_count[i].second>=4&&left_line_type_and_count[i+1].second>=4
			&&LD_find == false){
			if(40<=angle && angle<=120){ //大约小于154度
				pointBreakLD = findline.left_point[suspect_left_corner[i]];
				pointLD_index = suspect_left_corner[i];
				LD_find = true;
				}
				
			}
			else if(left_line_type_and_count[i].first ==6 &&
			(left_line_type_and_count[i+1].first ==4 ||left_line_type_and_count[i+1].first ==2)
			&&left_line_type_and_count[i].second>=4&&left_line_type_and_count[i+1].second>=4
			&&LU_find == false ){
				if(40<=angle && angle<=120){
				pointBreakLU = findline.left_point[suspect_left_corner[i]];
				pointLU_index = suspect_left_corner[i];
				LU_find = true;
				}
				
			}
			if(LD_find && LU_find){
			break;
			}
		}
   }
   
//    for(auto _d : findline.dir_l){
// 	std::cout << _d << " ";
//    }
//    std::cout << "\n";
	//寻找右上和右下的拐点
	std::vector<std::pair<int,int>> right_line_type_and_count;// 左边点类型和计数
	for(size_t i = 0;i < findline.dir_r.size() ;i++){
		//4
		
		if(findline.dir_r[i] ==4 ||findline.dir_r[i] ==5 ){ //判断是直线
		if(!right_line_type_and_count.empty() && right_line_type_and_count.back().first != 4){ //前一个处理
			if(right_line_type_and_count.back().second >1){
			suspect_right_corner.emplace_back(i);
			}else{
			right_line_type_and_count.pop_back(); //弹出最后一个
			if(!right_line_type_and_count.empty()){
				right_line_type_and_count.back().second++; //将这个点加给前一个类型
			}

			if(!suspect_right_corner.empty()){
				suspect_right_corner.pop_back();
			}
			}
		}
		if(!right_line_type_and_count.empty()&& right_line_type_and_count.back().first ==4){
			right_line_type_and_count.back().second++;
		}else 
		{
			right_line_type_and_count.emplace_back(4,1); //4或5都用4代表
		}
		}
		// 2
		else if(findline.dir_r[i] ==2 ||findline.dir_r[i] ==3 ||  findline.dir_r[i] ==1){
		if(!right_line_type_and_count.empty() && right_line_type_and_count.back().first!=2){
			if(right_line_type_and_count.back().second >1){
			suspect_right_corner.emplace_back(i);
			}else{
			right_line_type_and_count.pop_back(); //弹出最后一个
			if(!right_line_type_and_count.empty()){
				right_line_type_and_count.back().second++; //将这个点加给前一个类型
			}
			if(!suspect_right_corner.empty()){
				suspect_right_corner.pop_back();
			}

			}
		
		}
		
		if(!right_line_type_and_count.empty()&& right_line_type_and_count.back().first ==2){
			right_line_type_and_count.back().second++;
		}else {
			right_line_type_and_count.emplace_back(2,1); //2或3都用2代表
		}

		}
		//6
		else if(findline.dir_r[i] == 6 || findline.dir_r[i] == 7){
		if(!right_line_type_and_count.empty() && right_line_type_and_count.back().first!=6){ //保存跳变点
			if(right_line_type_and_count.back().second >1){
			suspect_right_corner.emplace_back(i);

			}else{
			right_line_type_and_count.pop_back(); //弹出只有一个的点
			if(!right_line_type_and_count.empty()){
				right_line_type_and_count.back().second++; //将这个点加给前一个类型
			}
			if(!suspect_right_corner.empty()){
				suspect_right_corner.pop_back();
			}

			}
		}
		if(!right_line_type_and_count.empty()&& right_line_type_and_count.back().first ==6){
			right_line_type_and_count.back().second++;
		}else {
			right_line_type_and_count.emplace_back(6,1); //6或7都用6代表
		}
		}
	}
  	 //校验是否符合要求
   	bool RD_find =false;
   	bool RU_find = false;
   	if (right_line_type_and_count.size()>1){
		for(size_t i =0 ;i<right_line_type_and_count.size()-1 && !suspect_right_corner.empty() ;i++){
			double angle = neighbour_points_angle(suspect_right_corner[i] ,findline.right_point);
			// std::cout <<"rightAngle" << angle << "\n";
				//std::cout << "内积" << _c << "\n";
			if(right_line_type_and_count[i].first ==4 &&right_line_type_and_count[i+1].first ==2
			&&right_line_type_and_count[i].second>=4&&right_line_type_and_count[i+1].second>=4
			&&RD_find == false){
				if(40<=angle && angle<=120){//大约150度
				pointBreakRD = findline.right_point[suspect_right_corner[i]];
				pointRD_index = suspect_right_corner[i];
				RD_find = true;
				} 
				
			}
			else if(right_line_type_and_count[i].first ==6 &&
			(right_line_type_and_count[i+1].first ==4 || right_line_type_and_count[i+1].first ==2)
			&&right_line_type_and_count[i].second>=4&&right_line_type_and_count[i+1].second>=4
			&&RU_find == false){
				if(40<=angle && angle<=120){ //大约150度
				pointBreakRU = findline.right_point[suspect_right_corner[i]];
				pointRU_index=suspect_right_corner[i];
				RU_find = true;
				}
				
			}
			if(RD_find && RU_find){
				break;
			}
		}
   }
//    std::cout << "LD" << pointBreakLD.x<<" " << pointBreakLD.y << " ";
//    std::cout << "LU" << pointBreakLU.x<<" " << pointBreakLU.y << " ";
//    std::cout << "RD" << pointBreakRD.x<<" " << pointBreakRD.y << " ";
//    std::cout << "RU" << pointBreakRU.x<<" " << pointBreakRU.y << "\n";
//    for(auto rlt: right_line_type_and_count){
// 	std::cout << "linetype" << rlt.first <<"count" << rlt.second << "\n";
//    }
//    std::cout <<"*******" << "\n";
   //空行统计
   int left_empty_line = 0;
   int right_empty_line = 0;
   for(auto p : findline.left_point){
	if(p.x <=6){
		left_empty_line++;
	}
	
   }
   for(auto p : findline.right_point){
	if(p.x >=315){
		right_empty_line++;
	}
   }
 //高度低于切行就忽略 默认20行
  if(RU_find && pointBreakRU.y < CarParams->edgeCutup){
	RU_find =false;
  }
  if(RD_find && pointBreakRD.y < CarParams->edgeCutup){ 
	RD_find =false;
  };
  if(LU_find && pointBreakLU.y < CarParams->edgeCutup){
	LU_find =false;
  }
  if(LD_find && pointBreakLD.y < CarParams->edgeCutup){ 
	LD_find =false;
  }

	// std::cout <<"rightEmpty"<<right_empty_line << " " << static_cast<int>(findline.right_point.size()*8/10) <<"\n";
  if(RU_find && RD_find && LU_find && LD_find){
	this->crossroadType = CrossroadType::CrossroadStraight;//直入
	repair_line(findline);
	return true;
  }else if(RU_find && RD_find && left_empty_line > static_cast<int>(findline.left_point.size()*8/10)){
	this->crossroadType = CrossroadType::CrossroadRight; //右十字
	repair_line(findline);
	return true;
  }else if (LU_find && LD_find&& right_empty_line > static_cast<int>(findline.right_point.size()*8/10)){
	this->crossroadType = CrossroadType::CrossroadLeft; //左十字
	repair_line(findline);
	return true;
  }else if(LU_find && LD_find &&!RU_find && RD_find){ //三点十字
	this->crossroadType = CrossroadType::CrossroadStraight;//直入
	repair_line_tripoints(findline);
	return true;
  }else if (RU_find && RD_find &&!LU_find && LD_find){ //三点十字
	this->crossroadType = CrossroadType::CrossroadStraight;//直入
	repair_line_tripoints(findline);
	return true;
  }


  else
   {
	this->crossroadType= CrossroadType::None;
	
  }

   return false;

}

void Cross::repair_line(Findline & findline){
	std::vector<cv::Point> new_left_point;
	std::vector<cv::Point> new_right_point;
	if(pointBreakLU!= cv::Point(0,0) && pointBreakLD != cv::Point(0,0)){
	for(size_t i =0;i <= pointLD_index ;i++){
		new_left_point.emplace_back(findline.left_point[i]);
	}
	float k = (float)(pointBreakLU.y-pointBreakLD.y)/((float)(pointBreakLU.x-pointBreakLD.x)+0.001f);
    float b = pointBreakLU.y - k*(pointBreakLU.x);
    for(int i = pointBreakLD.y; i>= pointBreakLU.y;i--)
    {
        new_left_point.emplace_back((int)((i-b)/k),i);
    }
	
	for(size_t i = pointLU_index ;i < findline.left_point.size();i++){
		new_left_point.emplace_back(findline.left_point[i]);
	}
	}

	if(pointBreakRU!= cv::Point(0,0) && pointBreakRD != cv::Point(0,0)){
	for(size_t i =0;i <= pointRD_index ;i++){
		new_right_point.emplace_back(findline.right_point[i]);
	}

	float k = (float)(pointBreakRU.y-pointBreakRD.y)/((float)(pointBreakRU.x-pointBreakRD.x)+0.001f);
    float b = pointBreakRU.y - k*(pointBreakRU.x);
    for(int i = pointBreakRD.y; i>= pointBreakRU.y;i--)
    {
        new_right_point.emplace_back((int)((i-b)/k),i);
    }
	
	for(size_t i = pointRU_index ;i < findline.right_point.size();i++){
		new_right_point.emplace_back(findline.right_point[i]);
	}
	}
	if(this->crossroadType == CrossroadType::CrossroadStraight){
		//findline.edge_calculate(new_left_point,new_right_point);
	}else if(this->crossroadType == CrossroadType::CrossroadRight){
		//findline.edge_calculate(findline.left_point,new_right_point);
	}else if(this->crossroadType == CrossroadType::CrossroadLeft){
		//findline.edge_calculate(new_left_point,findline.right_point);
	}

	
  }

void Cross::repair_line_tripoints(Findline & findline){
	std::vector<cv::Point> new_left_point;
	std::vector<cv::Point> new_right_point;
	//左边两点是完整的，右边只有右下角一点
	if(pointBreakLU!= cv::Point(0,0) && pointBreakLD != cv::Point(0,0)){
	//左边界补线
		for(size_t i =0;i <= pointLD_index ;i++){
		new_left_point.emplace_back(findline.left_point[i]);
	}
	float k = (float)(pointBreakLU.y-pointBreakLD.y)/((float)(pointBreakLU.x-pointBreakLD.x)+0.001f);
    float b = pointBreakLU.y - k*(pointBreakLU.x);
    for(int i = pointBreakLD.y; i>= pointBreakLU.y;i--)
    {
        new_left_point.emplace_back((int)((i-b)/k),i);
    }
	
	for(size_t i = pointLU_index ;i < findline.left_point.size();i++){
		new_left_point.emplace_back(findline.left_point[i]);
	}

	//绘制右边
	int near_point_index = (pointRD_index-5) > 0 ? (pointRD_index-5) : 0;

	auto near_point = findline.right_point[near_point_index];
	float _k = (float)(pointBreakRD.y-near_point.y)/((float)(pointBreakRD.x-near_point.x)+0.001f);
	float _b = near_point.y - _k*(near_point.x);
	for(size_t i =0;i <= pointRD_index ;i++){
		new_right_point.emplace_back(findline.right_point[i]);
	}
	
	for(int i =  pointBreakRD.y; i>= findline.pointsEdgeLeft.back().row;i--) //补到和左边界同高
    {
		
        new_right_point.emplace_back((int)((i-_b)/_k),i);
    }
	//右边两点是完整的，左边只有左下角一点
	}else if(pointBreakRU!= cv::Point(0,0) && pointBreakRD != cv::Point(0,0)){
	//先绘制右边线

	for(size_t i =0;i <= pointRD_index ;i++){
		new_right_point.emplace_back(findline.right_point[i]);
	}

	float k = (float)(pointBreakRU.y-pointBreakRD.y)/((float)(pointBreakRU.x-pointBreakRD.x)+0.001f);
    float b = pointBreakRU.y - k*(pointBreakRU.x);
    for(int i = pointBreakRD.y; i>= pointBreakRU.y;i--)
    {
        new_right_point.emplace_back((int)((i-b)/k),i);
    }
	
	for(size_t i = pointRU_index ;i < findline.right_point.size();i++){
		new_right_point.emplace_back(findline.right_point[i]);
	}
	//绘制左边
	int near_point_index = (pointLD_index-5) > 0 ? (pointLD_index-5) : 0;

	auto near_point = findline.left_point[near_point_index];
	float _k = (float)(pointBreakLD.y-near_point.y)/((float)(pointBreakLD.x-near_point.x)+0.001f);
	float _b = near_point.y - _k*(near_point.x);
	for(size_t i =0;i <= pointRD_index ;i++){
		new_left_point.emplace_back(findline.left_point[i]);
	}
	
	for(int i =  pointBreakLD.y; i>= findline.pointsEdgeRight.back().row;i--) //补到和左边界同高
    {
		
        new_left_point.emplace_back((int)((i-_b)/_k),i);
    }
	}
	//findline.edge_calculate(new_left_point,new_right_point);
} 