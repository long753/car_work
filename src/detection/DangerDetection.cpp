#include "DangerDetection.h"
#include <algorithm>

bool Danger::process(Findline &track, vector<PredictResult> predict)
{
    enable = false;
    if(dangerstep == DangerStep::None){
        for (auto pre : predict)
        {
            if (pre.type == LABEL_BOMB){
                dangerstep = DangerStep::Enable;
                first_cone = true;
                return true;
            }
        }
      
        return false;//未识别
    }

    if(dangerstep == DangerStep::Enable){

        block_state = false;
        vector<PredictResult> resultsObs; // 锥桶AI检测数据
        for(auto pre : predict){
            if (pre.type == LABEL_CONE) { 
                resultsObs.emplace_back(pre);
            }
            else if(pre.type == LABEL_BLOCK){
                resultsObs.emplace_back(pre);
                block_state = true;
            }
        }

        if (resultsObs.size()<=0){//连续50帧无元素退出
            counterRec++;
            if (counterRec >= ai_params->danger_exit_frame){
                std::cout<<"exit, ai size = 0"<<std::endl;
                counterRec=0;
                return false;
            }
            return true;
        }
        else{
            counterRec = 0;
        }

        // 选取距离最近的锥桶
        int areaMax = 0; 
        if(resultsObs.size()>=2){
            std::sort(resultsObs.begin(),resultsObs.end(),[&](PredictResult &p1 , PredictResult &p2){
            int area1 = p1.height+p1.y;
            int area2 = p2.height+p2.y;
            areaMax = max(area1,area2);
            return area1 > area2; //越近越大
            });
        }
        
        nearest_object = resultsObs.front();
        enable = true; // 场景检测使能标志

        if (nearest_object.y > ROWSIMAGE*0.15){
            dangerstep = DangerStep::Cruise;
            std::cout<<"enter Cruise"<<std::endl;
        }
    }

    if(dangerstep == DangerStep::Cruise){
        counterSession++;
        if (counterSession<CarParams->danger_slowcount){
            return true;
        }
        block_state = false;
        vector<PredictResult> resultsObs; // 锥桶AI检测数据
        for(auto pre : predict){
            if (pre.type == LABEL_CONE) {  
                resultsObs.emplace_back(pre);
            }
            else if(pre.type == LABEL_BLOCK){
                resultsObs.emplace_back(pre);
                block_state = true;
            }
        }
        if (resultsObs.size()<=0){
            counterRec++;
            if (counterRec >= 50){//连续五0帧无元素退出
                std::cout<<"exit, ai size = 0"<<std::endl;
                counterRec=0;
                counterImmunity = 0;
                counterSession = 0;
                return false;
            }
            return true;
        }
        else{
            counterRec = 0;
        }

        // 选取距离最近的锥桶
        int areaMax = 0; // 框左下角row 
        if(resultsObs.size()>=2){
            std::sort(resultsObs.begin(),resultsObs.end(),[&](PredictResult &p1 , PredictResult &p2){
            int area1 = p1.height+p1.y;
            int area2 = p2.height+p2.y;
            areaMax = max(area1,area2);
            return area1 > area2; //越近越大
            });
        }
        
        nearest_object = resultsObs.front();
        enable = true; // 场景检测使能标志
        int row = abs(CarParams->imgh - nearest_object.y - nearest_object.height);//从下往上数
        if (row < 0){
            std::cout<<"无需规划路径"<<std::endl;
            return enable;
        }
        int disLeft = abs(nearest_object.x  - track.pointsEdgeLeft[row].col);//左侧到左赛道边界
        int disRight = abs(track.pointsEdgeRight[row].col - nearest_object.x - nearest_object.width);//右侧到右赛道边界
        
        if (disLeft<=disRight) cone_left = true;//&& (nearest_object.x+nearest_object.width)<COLSIMAGE/2
        else cone_left = false;
        
        if (ring_params->danger_correct){
            if (cone_left&&nearest_object.type == LABEL_CONE){
                int count_left = std::count_if(track.pointsEdgeLeft.begin(), track.pointsEdgeLeft.end(),
                                    [](const mpoint& point) {
                                        return point.col < 5;
                                    });
                if (count_left>ROWSIMAGE*7/12) {
                    cone_left = !cone_left;
                    std::cout<<"纠正锥桶方向,使左打:"<<count_left<<std::endl;
                }
            }
            if (!cone_left&&nearest_object.type == LABEL_CONE){
                int count_right = std::count_if(track.pointsEdgeRight.begin(), track.pointsEdgeRight.end(),
                                    [](const mpoint& point) {
                                    return point.col > 314;
                                    });
                if (count_right>ROWSIMAGE*7/12){
                    cone_left = !cone_left;
                    std::cout<<"纠正锥桶方向2:"<<count_right<<std::endl;
                }
            }
        }
        
        if ((nearest_object.x + nearest_object.width > track.pointsEdgeLeft[row].col||CarParams->danger_col_debug) &&
            (nearest_object.y+nearest_object.height>ROWSIMAGE*ring_params->danger_pingbi_row)&&
            track.pointsEdgeRight[row].col > nearest_object.x &&
            cone_left) //[1] 障碍物靠左
        {
            if (nearest_object.type == LABEL_BLOCK) // 黑色路障特殊处理
            {
                counterImmunity++;
                if (counterImmunity>CarParams->danger_blockcount 
                &&nearest_object.y+nearest_object.height>ROWSIMAGE*(ring_params->danger_block_pingbi)){
                    curtailTracking(track, false);
                    std::cout<<"黑色路障左"<<std::endl;
                }
                else{
                    std::cout<<"block屏蔽计数"<<counterImmunity <<std::endl;
                    curtailTracking(track, true);
                }
            }
            else
            {
                vector<mpoint> points(4); // 三阶贝塞尔曲线
                points[0] = track.pointsEdgeLeft[row / 2];
                points[1] = {nearest_object.y + nearest_object.height, min(nearest_object.x + 2*nearest_object.width+CarParams->danger_anglesize,COLSIMAGE-1)};
                points[2] = {(nearest_object.y + nearest_object.height + nearest_object.y) / 2, 
                                nearest_object.x + 2*min(nearest_object.width+CarParams->danger_anglesize,COLSIMAGE)};
                if (nearest_object.y > track.pointsEdgeLeft[track.pointsEdgeLeft.size() - 1].row)
                    points[3] = {track.pointsEdgeLeft[track.pointsEdgeLeft.size() - 1].row,
                                    track.pointsEdgeLeft[track.pointsEdgeLeft.size() - 1].col+ CarParams->danger_anglesize};
                else
                    points[3] = {nearest_object.y, nearest_object.x + nearest_object.width};

                track.pointsEdgeLeft.resize((size_t)row/2); // 删除错误路线
                vector<mpoint> repair = Bezier(0.001, points);  // 重新规划车道线
                for (int i = 0; i < repair.size(); i++)
                    track.pointsEdgeLeft.push_back(repair[i]);
            }
        }
        else if (nearest_object.x + nearest_object.width > track.pointsEdgeLeft[row].col &&
            (nearest_object.y+nearest_object.height>ROWSIMAGE*ring_params->danger_pingbi_row)&&
                 (track.pointsEdgeRight[row].col > nearest_object.x ||CarParams->danger_col_debug)&&
                !cone_left) //[2] 障碍物靠右
        {
            if (nearest_object.type == LABEL_BLOCK){
                counterImmunity++;
                if (counterImmunity>CarParams->danger_blockcount 
                &&nearest_object.y+nearest_object.height>ROWSIMAGE*(ring_params->danger_block_pingbi)) // 黑色路障特殊处理
                {
                    curtailTracking(track, true); // 缩减优化车道线（双车道→单车道）
                    std::cout<<"黑色路障右"<<std::endl;
                }
                else{
                    curtailTracking(track, false); // 缩减优化车道线（双车道→单车道）
                    std::cout<<"block屏蔽计数"<<counterImmunity <<std::endl;

                }
            }
            else{
                vector<mpoint> points(4); // 三阶贝塞尔曲线
                points[0] = track.pointsEdgeRight[row / 2];
                points[1] = {nearest_object.y + nearest_object.height, max(nearest_object.x - 2*nearest_object.width-CarParams->danger_anglesize,0)};
                points[2] = {(nearest_object.y + nearest_object.height + nearest_object.y) / 2, 
                                max(nearest_object.x - 2*nearest_object.width-CarParams->danger_anglesize,0)};
                // points[3] = {nearest_object.y, nearest_object.x};

                if (nearest_object.y > track.pointsEdgeRight[track.pointsEdgeRight.size() - 1].row)
                    points[3] = {track.pointsEdgeRight[track.pointsEdgeRight.size() - 1].row,
                                track.pointsEdgeRight[track.pointsEdgeRight.size() - 1].col- CarParams->danger_anglesize};
                else
                    points[3] = {nearest_object.y, nearest_object.x};

                track.pointsEdgeRight.resize((size_t)row/2); // 删除错误路线
                vector<mpoint> repair = Bezier(0.001, points);   // 重新规划车道线
                for (int i = 0; i < repair.size(); i++)
                    track.pointsEdgeRight.push_back(repair[i]);
            }
        }

    }



    return enable;
}
bool Danger::left_cone_process(Findline &track, vector<PredictResult> & predict){
    vector<mpoint> points(3); // 三阶贝塞尔曲线
    points[0] = track.pointsEdgeLeft.front();//左边界起点
   
    points[2] = track.pointsEdgeRight[track.pointsEdgeRight.size()/2];
    points[1] = {(points[0].row+points[2].row)/2, (points[0].col+points[2].col)/2};//右边界中点
    track.pointsEdgeRight.resize(track.pointsEdgeRight.size()/2);
    track.endline_eight = points[1].row; 
    track.pointsEdgeLeft.clear(); // 删除错误路线
                // vector<mpoint> repair = Bezier(0.001, points);  // 重新规划车道线   
    add_points_between_terminal_points(track.pointsEdgeLeft, points[0], points[2]);   
                //距离最近的锥桶和上一帧相比的跳变距离
    gap_cone_distance = (nearest_object.height + nearest_object.y) - last_cone_row ;
}

bool Danger::right_cone_process(Findline &track, vector<PredictResult> & predict){
     vector<mpoint> points(3); // 三阶贝塞尔曲线
        points[0] = track.pointsEdgeRight.front();
        points[1] = {nearest_object.y + nearest_object.height, nearest_object.x };//框框左下角
        points[2] = track.pointsEdgeLeft[track.pointsEdgeLeft.size()/2];
        track.pointsEdgeLeft.resize(track.pointsEdgeLeft.size()/2);
         track.endline_eight = points[1].row; 
        track.pointsEdgeRight.clear(); // 删除错误路线
        add_points_between_terminal_points(track.pointsEdgeRight, points[0], points[2]);
        gap_cone_distance = (nearest_object.height + nearest_object.y) - last_cone_row ;
}

bool Danger::block_process(Findline &track, vector<PredictResult> & predict){

}
void Danger::drawImage(Mat &img)
{
    if (enable)
    {
        putText(img, "[2] DANGER - ENABLE", Point(COLSIMAGE / 2 - 30, 10), cv::FONT_HERSHEY_TRIPLEX, 0.5, cv::Scalar(0, 255, 0), 1, CV_AA);
        cv::Rect rect(nearest_object.x, nearest_object.y, nearest_object.width, nearest_object.height);
        cv::rectangle(img, rect, cv::Scalar(0, 0, 255), 1);
    }
}
void Danger::curtailTracking(Findline &track, bool left){
        if (left){ // 向左侧缩进
            if (track.pointsEdgeRight.size() > track.pointsEdgeLeft.size())
                track.pointsEdgeRight.resize(track.pointsEdgeLeft.size());

            for (int i = 0; i < track.pointsEdgeRight.size(); i++){
                track.pointsEdgeRight[i].col = max(0,
                (int)(track.pointsEdgeRight[i].col+ track.pointsEdgeLeft[i].col)/2 -ring_params->danger_block_angle);
            }
        }
        else{ // 向右侧缩进
            if (track.pointsEdgeRight.size() < track.pointsEdgeLeft.size())
                track.pointsEdgeLeft.resize(track.pointsEdgeRight.size());

            for (int i = 0; i < track.pointsEdgeLeft.size(); i++){
                track.pointsEdgeLeft[i].col = min((COLSIMAGE-1),
                (int)((track.pointsEdgeRight[i].col + track.pointsEdgeLeft[i].col) / 2 +ring_params->danger_block_angle));
            }
        }
    }
