#include "Findline.h"
#include "RingElement.h"
#include <iostream>

Findline::Findline(){
    ImageType imageType = ImageType::Binary; // 赛道识别输入图像类型：二值化图像
    line_type = LineType::STRAIGHT;
    kernel = getStructuringElement(0, Size(3, 3));
} 


  /**
   * @brief 显示赛道线识别结果
   *
   * @param trackImage 需要叠加显示的图像
   */
void Findline::drawImage(Mat &trackImage , std::vector<mpoint>pointsLeft,std::vector<mpoint>pointsRight ) {
    
    for (size_t i = 0; i < pointsLeft.size(); i++) {
      circle(trackImage, pointsLeft[i], 1,
             Scalar(0, 255, 0), -1); // 绿色点
    }
    for (size_t i = 0; i < pointsRight.size(); i++) {
      circle(trackImage, pointsRight[i], 1,
             Scalar(0, 255, 255), -1); // 黄色点
    }
//    circle(trackImage, mpoint(160,120), 1,
//              Scalar(0, 255, 255), -1); // 黄色点
  }


cv::Point Findline::point_add(cv::Point & p1 , cv::Point &p2){
    return {p1.x+p2.x,p1.y+p2.y};
}


/**
这个是八邻域的识别函数 

*/

void Findline::search_line(Mat & imgb){
    bool left_stop = false;
    bool right_stop = false;
    this->highest_row = 240;
    pointsEdgeLeft.clear();
    pointsEdgeRight.clear();
    left_point.clear();
    right_point.clear();
    dir_l.clear();
    dir_r.clear();

    Mat tmp_img = imgb.clone();
    
    cv::morphologyEx(imgb, tmp_img, cv::MorphTypes::MORPH_OPEN, kernel); //开运算
    cv::rectangle(tmp_img, Point(0, 0), Point(320, 240), 0, 5);


    ///////////////////////////////////////////////////////////////
  
    int rect_num_l=0;
    int ring_l=0;
    rect_l=0;
    flag_left_t=0;
    for(int i=80;i<220;i++){
        for(int j=10;j<13;j++){
            if(tmp_img.at<uchar>(i,j)==255){
                rect_num_l++;
            }
        }
    }
    //cout<<endl<<"rect:"<<rect_num_l<<endl;
    if(rect_num_l>370){
        //cout<<"111111111111111"<<endl;
        rect_l=1;
    }
    for(int i=100;i<180;i++){
        for(int j=30;j<33;j++){
            if(tmp_img.at<uchar>(i,j)==255){
                ring_l++;
            }
        }
    }
    if(ring_l<220){
        //cout<<"111111111111111"<<endl;
        flag_left_t=1;
    }


    int rect_num_r=0;
    int ring_r=0;
    rect_r=0;
    flag_right_t=0;
    for(int i=80;i<220;i++){
        for(int j=307;j<310;j++){
            if(tmp_img.at<uchar>(i,j)==255){
                rect_num_r++;
            }
        }
    }
    //cout<<endl<<"rect:"<<rect_num_r<<endl;
    if(rect_num_r>370){
        rect_r=1;
    }
    for(int i=110;i<150;i++){
        for(int j=287;j<290;j++){
            if(tmp_img.at<uchar>(i,j)==255){
                ring_r++;
            }
        }
    }
    if(ring_r<100){
        //cout<<"111111111111111"<<endl;
        flag_right_t=0;
    }
    flag_left_t_finish=0;
    flag_right_t_finish=0;
    int f_num_l=0;
    int f_num_r=0;
   for(int i=120;i<180;i++){
            if(tmp_img.at<uchar>(i,240)==255){
                f_num_l++; 
        }
    }
    if(f_num_l>40){
        flag_left_t_finish=1;
    }
    for(int i=120;i<180;i++){
            if(tmp_img.at<uchar>(i,80)==255){
                f_num_r++; 
        }
    }
    if(f_num_r>40){
        flag_right_t_finish=1;
    }
     exit_l=1;
     exit_r=1;
     for(int i=60;i<160;i++){
            if(tmp_img.at<uchar>(i,10)!=255&&tmp_img.at<uchar>(i+1,10)!=255&&
               tmp_img.at<uchar>(i+2,10)==255&&tmp_img.at<uchar>(i+3,10)==255){
                exit_l=i;
                //break;
            }
        }
     for(int i=exit_l+5;i<160;i++){
            if(tmp_img.at<uchar>(i,10)!=255&&tmp_img.at<uchar>(i+1,10)!=255)
              exit_l=1;
     }
     int continue_l=0;
    for(int i=exit_l-5;i>60;i--){
        if(continue_l==0){
          if(tmp_img.at<uchar>(i,10)==255)
             continue_l++;
        }
        else if(continue_l!=0){
          if(tmp_img.at<uchar>(i,10)==255){
            continue_l++;
          }
          else {
            continue_l=0;
          }
          if(continue_l>20){
            exit_l=1;
          }
        }
    }
   
     for(int i=60;i<160;i++){
             if(tmp_img.at<uchar>(i,310)!=255&&tmp_img.at<uchar>(i+1,310)!=255&&
               tmp_img.at<uchar>(i+2,310)==255&&tmp_img.at<uchar>(i+3,310)==255){
                exit_r=i;
            }
        }
     for(int i=exit_r+5;i<160;i++){
            if(tmp_img.at<uchar>(i,310)!=255&&tmp_img.at<uchar>(i+1,310)!=255)
              exit_r=1;
     }
      int continue_r=0;
    for(int i=exit_r-5;i>60;i--){
        if(continue_r==0){
          if(tmp_img.at<uchar>(i,310)==255)
             continue_r++;
        }
        else if(continue_r!=0){
          if(tmp_img.at<uchar>(i,310)==255){
            continue_r++;
          }
          else {
            continue_r=0;
          }
          if(continue_r>20){
            exit_r=1;
          }
        }
    }
    int continue_hang=0;
    for(int i=10;i<310;i++){
        if(continue_hang==0){
            if(tmp_img.at<uchar>(60,i)==255)
              continue_hang++;
        }
        else if(continue_hang!=0){
          if(tmp_img.at<uchar>(60,i)==255){
            continue_hang++;
          }
          else {
            continue_hang=0;
          }
          if(continue_hang>20){
            exit_r=1;
            exit_l=1;
          }
        }
    }
    in_l=0;
    int in_num_l=0;
   for(int i=220;i<230;i++){
    for(int j=300;i<310;i++){
        if(tmp_img.at<uchar>(i,j)==255){
            in_num_l++;
        }
    }
   }
   if(in_num_l<20){
      in_l=1;
   }
   in_r=0;
    int in_num_r=0;
   for(int i=220;i<230;i++){
    for(int j=1;i<10;i++){
        if(tmp_img.at<uchar>(i,j)==255){
            in_num_r++;
        }
    }
   }
   if(in_num_r<20){
      in_r=1;
   }
    //cout<<endl<<"exit_r:"<<exit_r<<endl;
    //cout<<endl<<"exit_l:"<<exit_l<<endl;
    
   // cout<<endl<<"111111111111111:"<<(tmp_img.at<uchar>(200,120)==255)<<endl;
  ////////////////////////////////////////////////////////////////////


    std::unique_ptr<uint8_t[][240]> readed_point(new uint8_t[320][240]{ 0 }); //读取过的点
    bool l_flag = false;
    bool r_flag = false;

    cv::Point  l_start{ 0,0 };
    cv::Point r_start{ 0,0 };

    // int imgh = ROWSIMAGE - 40;
    auto mat_ptr = tmp_img.ptr<uchar>(CarParams->imgh);
    for (int i = CarParams->imgh; i > CarParams->imgh - 3; i--) {
        if (!r_flag) {
            for (int j = COLSIMAGE / 2; j < COLSIMAGE; j++) {
                if (mat_ptr[j] == 255 && mat_ptr[j - 1] == 255 && mat_ptr[j - 2] == 255 &&mat_ptr[j - 4] == 255 &&mat_ptr[j - 6] == 255 &&
                mat_ptr[j - 8] == 255 &&mat_ptr[j - 10] == 255 &&
                    mat_ptr[j + 1] == 0 && mat_ptr[j + 2] == 0) {
                    r_flag = true;
                    r_start.x = j;
                    r_start.y = i;
                    // std::cout << "1" << endl;
                    break;
                }
            }
        }
        if (!l_flag) {
            for (int j = COLSIMAGE / 2; j > 1; j--) {
                if (mat_ptr[j] == 255 && mat_ptr[j + 2] == 255 && mat_ptr[j + 4] == 255 &&mat_ptr[j + 4] == 255 &&mat_ptr[j + 6] == 255 &&
                mat_ptr[j + 8] == 255 &&mat_ptr[j + 10] == 255 &&
                    mat_ptr[j - 1] == 0 && mat_ptr[j - 2] == 0) {
                    l_flag = true;
                    l_start.x = j;
                    l_start.y = i;
                    // std::cout << "2" << endl;
                    break;
                }
            }
        }
        if (l_flag && r_flag) {
            break;
        }
        else {
        if (!r_flag) {
            for (int j = COLSIMAGE - 3; j > 0; j--) {
                if (mat_ptr[j] == 255 && mat_ptr[j - 4] == 255 && mat_ptr[j - 8] == 255 && mat_ptr[j - 15] == 255 && mat_ptr[j + 1] == 0 && mat_ptr[j + 2] == 0) {
                    r_flag = true;
                    r_start.x = j;
                    r_start.y = i;
                    // std::cout << "3" << endl;

                    break;
                }
            }
        }
        if (!l_flag) {
            for (int j = 2; j < COLSIMAGE - 1; j++) {
                if (mat_ptr[j] == 255 && mat_ptr[j +4] == 255 && mat_ptr[j + 8] == 255 && mat_ptr[j + 15] == 255 && mat_ptr[j - 1] == 0 && mat_ptr[j - 2] == 0) {
                    l_flag = true;
                    l_start.x = j;
                    l_start.y = i;
                    // std::cout << "4" << endl;
                    break;
                }
            }
        }
        }
        if (l_flag && r_flag) {
            break;
        }
    }


    if (!l_flag || !r_flag) {
        left_point.clear();
        right_point.clear();
        return;
    }

    vector<cv::Point> left_field;
    vector<cv::Point> right_field;
    int left_highest = 240;
    int right_highest = 240;


    cv::Point center_l = l_start;
    cv::Point center_r = r_start;

    vector<cv::Point> tmp_l;
    vector<cv::Point> tmp_r;
    std::vector<int> tmp_dir_l;
    std::vector<int> tmp_dir_r;
    while (true)
    {
        left_field.clear();
        right_field.clear();
       
        if (!right_point.empty()&& !right_stop&&right_point.back().y < CarParams->rowCutUp ) {
            right_stop = true;

        }
        if (!left_point.empty()&&!left_stop && left_point.back().y < CarParams->rowCutUp){
            left_stop = true;
        }
        if (left_stop && right_stop) break;

        left_point.emplace_back(center_l);

        highest_row = min(center_l.y,highest_row);

        if (readed_point[center_l.x][center_l.y] <= 3) {
            readed_point[center_l.x][center_l.y]++;
        }
        else {

            left_stop = true;

        }

        right_point.emplace_back(center_r);
  
        highest_row = min(center_r.y,highest_row);
        if (readed_point[center_r.x][center_r.y] <= 3) {
            readed_point[center_r.x][center_r.y] ++;
        }
        else {

            right_stop = true;
        }
        if (left_stop && right_stop) break;

        tmp_l.clear();
        tmp_dir_l.clear();
        for (int i = 0; i < 8; i++) {
            int _x = center_l.x;
            int _y = center_l.y;
            if (tmp_img.at<uchar>(seeds_l[i].y+_y,seeds_l[i].x+_x) == 0 && tmp_img.at<uchar>(seeds_l[(i + 1) & 7].y+_y,seeds_l[(i + 1) & 7].x+_x) == 255) {
                tmp_l.emplace_back(seeds_l[i].x+_x,seeds_l[i].y+_y);
                //生长方向
                tmp_dir_l.emplace_back(i);
            }

        }
        if (!tmp_l.empty() && !left_stop) {

            int _i =0;
            for(auto m:tmp_l){

                if (readed_point[m.x][m.y] == 0) {
                    center_l = m;
                    dir_l.emplace_back(tmp_dir_l[_i]);
                    break;
                }
                _i++;
            }
        }
    


        tmp_r.clear();
        tmp_dir_r.clear();
    
        for (int i = 0; i < 8; i++) {
            int _x = center_r.x;
            int _y = center_r.y;
            if (tmp_img.at<uchar>(seeds_r[i].y+_y,seeds_r[i].x+_x) == 0 && tmp_img.at<uchar>(seeds_r[(i + 1) & 7].y+_y,seeds_r[(i + 1) & 7].x+_x) == 255) {
                tmp_r.emplace_back(seeds_r[i].x+_x,seeds_r[i].y+_y);
                //生长方向
                tmp_dir_r.emplace_back(i);
            }

        }
        if (!tmp_r.empty()) {
            int _i =0;
            for (auto m : tmp_r) {
                if (readed_point[m.x][m.y] == 0) {
                    center_r = m;
                    dir_r.emplace_back(tmp_dir_r[_i]);
                    break;
                }
                        _i++;
            }

        }
    
        if (!right_point.empty() && !left_point.empty()) {
            if (abs(right_point.back().x - left_point.back().x) < 5
                && abs(right_point.back().y - left_point.back().y) < 5
                ) {

                this->highest = max(right_point.back().y, left_point.back().y);
                break;
            }
            if(!dir_l.empty() && dir_l.back()==7
            && right_point.back().y > left_point.back().y
            ){

                center_l =left_point.back();
                left_point.pop_back();
                dir_l.pop_back();
            
            }
            if(!dir_r.empty() && dir_r.back()==7
            && left_point.back().y> right_point.back().y
            ){
            
                center_r = right_point.back();
                right_point.pop_back();
                dir_r.pop_back();
        
            }
        }

    }

    this->edge_calculate();
}

// 判断赛道类型
void Findline::edge_calculate(){
   empty_row_left = 0;
   empty_row_right = 0;
   std::vector<int> road_width;
    int high = 240;
    for(auto _m : left_point){
        if(_m.y != high){
            pointsEdgeLeft.emplace_back(_m);
            high = _m.y;
            if(abs(_m.x-LEFT_EDGE_COL)<2 && _m.y > ROWSIMAGE * 0.3){
                empty_row_left++;
            }
        }
        if(high < CarParams->rowCutUp){
            break;
        }
    }
    high = 240;
    for(auto _m:right_point){
        if(_m.y != high){
        pointsEdgeRight.emplace_back(_m);
        high=_m.y;
            if(abs(_m.x-RIGHT_EDGE_COL)<3 && _m.y > ROWSIMAGE * 0.3 ){
                empty_row_right++;
            }
        }
        if(high < CarParams->rowCutUp){
            break;
        }
    }
    int last_width = 320;
    int count = 0;
    int _size = min(pointsEdgeLeft.size(),pointsEdgeRight.size());
    for(size_t i = 0 ;i < _size ;i++){
        int width = pointsEdgeRight[i].col-pointsEdgeLeft[i].col;
        if(width  >0 
        && width  <= last_width){
            count ++ ;
            last_width = width;
        }
    }

}

void Findline::midline_calculate(){
    std::array<int,240> leftpoint;
    std::array<int,240> rightpoint;
    leftpoint.fill(4);
    rightpoint.fill(315);
    endline_eight = 240;
    for(auto p : this->pointsEdgeLeft){
      leftpoint[p.row] = p.col;
      endline_eight = min(p.row,endline_eight);
    };
    for(auto p : this->pointsEdgeRight){
      rightpoint[p.row] = p.col;
      endline_eight = min(p.row,endline_eight);
    };
    endline_eight++;
    this->midline.fill(160);
    if(line_type == LineType::STRAIGHT){
        for(int i = 238 ;i >=0 ; i--){
            if((leftpoint[i]==4)&&(rightpoint[i]==315)){
                midline[i]=midline[i+1];
            }
            else {
                midline[i] = (leftpoint[i]+rightpoint[i])/2;
            }
        }
    }


}









   

















int Findline::zuodandiao() {
    int num_4=0;
    int num_5=0;
    int num_78=0;
    int abnormal=0;
   for(int i=dir_l.size()/8;i<dir_l.size()*20/24;i++){//18/24
    if(dir_l[i]==4){
        num_4++;
   }
}
 for(int i=dir_l.size()/8;i<dir_l.size()*20/24;i++){//18/24
    if(dir_l[i]==5){
        num_5++;
   }
}
for(int i=dir_l.size()/8;i<dir_l.size()*20/24;i++){//18/24
    if(dir_l[i]==7||dir_l[i]==8){
        num_78++;
   }
}
for(int i=left_point.size()/8;i<left_point.size()-5;i++){//18/24
    if(left_point[i].x>240&&left_point[i].y>15){
        abnormal++;
        cout<<"col:"<<left_point[i].x<<",row:"<<left_point[i].y<<endl;
   }
}
//  if((num_4+num_5)>dir_r.size()*ring_params->right_num45){ //9/24
//     cout<<"r1"<<endl;
//    }
//    if((num_5>ring_params->right_num4*num_4)){
//     cout<<"r2"<<endl;
//    }
//    if((num_4>ring_params->right_num5*num_5)){ //0.3
//     cout<<"r3"<<endl;
//    }
//    cout<<"zuo_num78:"<<num_78<<endl;
//    cout<<"zuo_abnormal:"<<abnormal<<endl;
//    cout<<"per:"<<(num_4+num_5+0.00001)/(dir_l.size()+0.000001)<<endl;
//    cout<<"per_4"<<(num_4+0.000001)/(num_5+0.000001)<<endl;
//    cout<<"pre_5"<<(num_5+0.000001)/(num_4+0.000001)<<endl;
 if((num_4+num_5)>dir_l.size()*ring_params->right_num45&&
    (num_5>ring_params->right_num4*num_4)&&
    (num_4>ring_params->right_num5*num_5)&&
    (num_78<10)&&abnormal<7&&dir_r.size()>400){
     //zuo_num++;
    //  cout<<"zuo"<<endl;
     return 1;
   }
  

   return 0;
//    else{
//     zuo_num=0;
//    }
}
int Findline::youdandiao() {
    int num_4=0;
    int num_5=0;
    int num_78=0;
    int abnormal=0;
   for(int i=dir_r.size()/8;i<dir_r.size()*20/24;i++){//18/24
    if(dir_r[i]==4){
        num_4++;
    } 
   }
   for(int i=dir_r.size()/8;i<dir_r.size()*20/24;i++){//18/24
    if(dir_r[i]==5){
        num_5++;
    } 
   }
   for(int i=dir_r.size()/8;i<dir_r.size()*20/24;i++){//18/24
    if(dir_r[i]==7||dir_r[i]==8){
        num_78++;
    } 
   }
   cout<<"right_point:"<<right_point.size()<<endl;
   for(int i=right_point.size()/8;i<right_point.size();i++){//18/24
    if(right_point[i].x<80&&right_point[i].y>15){
        abnormal++;
        
        cout<<"you_col:"<<right_point[i].x<<",you_row:"<<right_point[i].y<<endl;
   }
   }
//    if((num_4+num_5)>dir_r.size()*ring_params->left_num45){ //9/24
//     cout<<"l1"<<endl;
//    }
//    if((num_5>ring_params->left_num4*num_4)){
//     cout<<"l2"<<endl;
//    }
//    if((num_4>ring_params->left_num5*num_5)){ //0.3
//     cout<<"l3"<<endl;
//    }
//    cout<<"you_num78:"<<num_78<<endl;
//    cout<<"you_abnormal:"<<abnormal<<endl;
//    cout<<"per:"<<(num_4+num_5+0.00001)/(dir_r.size()+0.000001)<<endl;
//    cout<<"per_4"<<(num_4+0.000001)/(num_5+0.000001)<<endl;
//    cout<<"pre_5"<<(num_5+0.000001)/(num_4+0.000001)<<endl;
   if((num_4+num_5)>dir_r.size()*ring_params->left_num45&&
   (num_5>ring_params->left_num4*num_4)&&
   (num_4>ring_params->left_num5*num_5)&&
   num_78<10&&abnormal<7&&dir_l.size()>400){//13/24
    //cout<<"you"<<endl;
    return 1;
    // you_num++;
   }
   
   return 0;
//    else{
//     you_num=0;
//    }
}







int Findline::zuo_dan_diao_width(){
    int num=0;
   for(int i=0;i<dir_l.size()*3/4;i++){
         if(left_point[i+1].x-left_point[i].x>=0&&
         left_point[i+1].x-left_point[i].x<=2){
            num++;
         }
   }
   if(num<dir_l.size()*13/20){
    //cout<<"zuo:"<<num<<endl;
    return 0;
   }
   for(int i=dir_l.size()*9/10;i<dir_l.size()*9/10+5 && i<dir_l.size();i++){
    if(left_point[i].x<60||left_point[i].x>180){
           return 0;
    }
   }
//    cout<<"zuo"<<endl;
   return 1;
}


int Findline::you_dan_diao_width(){
     int num=0;
   for(int i=0;i<dir_r.size()*3/4;i++){
         if(right_point[i+1].x-right_point[i].x<=0&&
         right_point[i+1].x-right_point[i].x>=-2){
            num++;
         }
   }
   if(num<dir_r.size()*13/20){
   // cout<<"you:"<<num<<endl;
    return 0;
   }
for(int i=dir_r.size()*9/10;i<dir_r.size()*9/10+5 && i<dir_r.size();i++){
    if(right_point[i].x>260||right_point[i].x<140){
           return 0;
    }
   }
//    cout<<"you"<<endl;
   return 1;

}

