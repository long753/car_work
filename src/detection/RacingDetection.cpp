#include "RacingDetection.h"
StopWatch race_danger;
int danger_side=0;
bool dangerDetected = false;  // spy识别到danger标志位
int propCounter = 0;          // 这个计数器用于记录识别到的 prop 数量
int propFrameCounter = 0;     // 检测到prop的帧计数器
int frameCounter = 0;         // 总帧计数器
bool propInProgress = false;  // 标志是否处于prop处理状态

// 将 prop1Type, prop2Type, prop3Type 映射为 TypeRace 类型
Racing::TypeRace Racing::mapPropType(int propType) {
    switch (propType) {
        case 1:
            return TypeRace::Safe;
        case 2:
            return TypeRace::Danger;
        case 3:
            return TypeRace::Spy;
        default:
            return TypeRace::None;
    }
}

bool Racing::process(Findline &track, vector<PredictResult> predicts)
{

    _index = 0;
    carStoping = false; // 停车标志
//指定
    if (CarParams->racing_key && !propInProgress) // 只有当没有正在处理的prop状态时才进行识别
    {
        bool propDetected = false; // 标记当前帧是否检测到 prop

        // 遍历预测结果，检查是否有 prop
        for (auto &predict : predicts) {
            if (predict.type == LABEL_PROP || predict.type == LABEL_SPY || predict.type == LABEL_SAFETY || predict.type == LABEL_DANGER){
                propFrameCounter++;
                propDetected = true;
                break; // 如果识别到prop就可以跳出循环
            }
        }

        // 增加帧计数器
        frameCounter++;

        // 检查在连续8帧中是否有3帧检测到prop
        if (frameCounter >= 8) {
            // 如果满足条件，进入处理逻辑
            if (propFrameCounter >= 3) {
                propCounter++;
                std::cout << "当前为第" << propCounter << "个小车" << std::endl;

                // 根据 propCounter 的值来决定 prop 的类型
                TypeRace targetType = TypeRace::None;
                switch (propCounter) {
                    case 1:
                        targetType = mapPropType(CarParams->prop1Type);
                        break;
                    case 2:
                        targetType = mapPropType(CarParams->prop2Type);
                        break;
                    case 3:
                        targetType = mapPropType(CarParams->prop3Type);
                        break;
                    default:
                        targetType = TypeRace::None;
                        break;
                }
                if (targetType != TypeRace::None) {
                    typeRace = targetType;
                    propInProgress = true; // 设置标志，表示进入了某个状态
                }
            }

            // 重置计数器
            frameCounter = 0;
            propFrameCounter = 0;
        }
    }

    switch (typeRace)
        {
        case TypeRace::None:// AI检测
            if (!CarParams->racing_key) { // 确保用 "!" 符号
                searchTypeRace(track, predicts); // 检索AI场景类型
            }
            break;
        //case TypeRace::Prop:          // AI检测
            //std::cout<<"识别到:prop"<<std::endl;
            //searchTypeRace(predicts); // 检索AI场景类型
            //break;

        case TypeRace::Safe: // 普通车辆
        {   race_step=1;
            //typeRace=TypeRace::Danger;
            std::cout<<"识别到safe"<<typeRace<<endl;
            //prop替换
            PredictResult predict = searchSign(predicts, CarParams->racing_key ? LABEL_PROP : LABEL_SAFETY);
            counterSes[0]++;
            if (predict.x > 0 && predict.y > 0) // 检测到有效的AI标志
            {
                // 检索小车靠近左/右方向
                sideLeft = false; // 靠左侧标志
                int row = track.pointsEdgeLeft[0].row - predict.y - predict.height;
                if (row < 0)
                {
                    _index = 1;
                    if (abs(track.pointsEdgeLeft[0].col - predict.x) > abs(track.pointsEdgeRight[0].col - predict.x - predict.width))
                        sideLeft = true;
                }
                else if (row >= track.pointsEdgeLeft.size() || row >= track.pointsEdgeRight.size())
                {
                    _index = 2;
                    if (abs(track.pointsEdgeLeft[track.pointsEdgeLeft.size() - 1].col - predict.x) > abs(track.pointsEdgeRight[track.pointsEdgeRight.size() - 1].col - predict.x - predict.width))
                        sideLeft = true;
                }
                else
                {
                    _index = 3;
                    if (abs(track.pointsEdgeLeft[row].col - predict.x) > abs(track.pointsEdgeRight[row].col - predict.x - predict.width))
                        sideLeft = true;
                }
             cout<<"safeleft:"<<sideLeft<<endl;
            // 重新规划车道线
            vector<mpoint> points(4); // 三阶贝塞尔曲线
            if (!sideLeft)
            {
                points[0] = track.pointsEdgeLeft[row / 2];
                points[1] = {predict.y + predict.height, min(predict.x + 2 * predict.width + 90,319)};
                points[2] = {(predict.y + predict.height + predict.y) / 2, min(predict.x + 2 * predict.width + 90,319)};
                if (predict.y > track.pointsEdgeLeft[track.pointsEdgeLeft.size() - 1].row)
                    points[3] = {track.pointsEdgeLeft[track.pointsEdgeLeft.size() - 1].row, min(track.pointsEdgeLeft[track.pointsEdgeLeft.size() - 1].col + CarParams->racing_anglesize + 90,319)};
                else
                    points[3] = {predict.y, min(predict.x + predict.width + 90,319)};

                track.pointsEdgeLeft.resize((size_t)row / 2); // 删除错误路线
                vector<mpoint> repair = Bezier(0.001, points); // 重新规划车道线
                for (int i = 0; i < repair.size(); i++)
                    track.pointsEdgeLeft.push_back(repair[i]);
            }
            else
            {
                points[0] = track.pointsEdgeRight[row / 2];
                points[1] = {predict.y + predict.height, predict.x - 2 * predict.width - 90};
                points[2] = {(predict.y + predict.height + predict.y) / 2, predict.x - 2 * predict.width - 90};
                if (predict.y > track.pointsEdgeRight[track.pointsEdgeRight.size() - 1].row)
                    points[3] = {track.pointsEdgeRight[track.pointsEdgeRight.size() - 1].row, track.pointsEdgeRight[track.pointsEdgeRight.size() - 1].col - CarParams->racing_anglesize - 90};
                else
                    points[3] = {predict.y, predict.x - predict.width - 90};

                track.pointsEdgeRight.resize((size_t)row / 2); // 删除错误路线
                vector<mpoint> repair = Bezier(0.001, points); // 重新规划车道线
                for (int i = 0; i < repair.size(); i++)
                    track.pointsEdgeRight.push_back(repair[i]);
            }

            counterSes[0] = 0;

            }
            else if (counterSes[0] > 50) // AI检测异常：退出该场景
            {
                std::cout<<"退出safe"<<std::endl;
                counterRec[0] = 0;
                counterSes[0] = 0;
                typeRace = TypeRace::None;
                propInProgress = false; // 重置标志，表示可以进行新的prop识别               
            }
            break;

        }

        case TypeRace::Spy: // 嫌疑车辆
        {   
            race_step=2;
            //typeRace=TypeRace::Danger;
            /**
             *嫌疑车辆逼停策略：绕行至车辆前方阻挡其运行
             */
                 // 在Spy状态下增加对Danger的检测
            if(!CarParams->racing_key){
                PredictResult dangerPredict = searchSign(predicts, LABEL_DANGER);
                if (dangerPredict.x > 0 && dangerPredict.y > 0) // 如果检测到Danger
                {
                    std::cout << "在Spy状态下检测到Danger，切换到Danger状态" << endl;
                    typeRace = TypeRace::Danger;
                    dangerDetected = true; // 设置标志位
                    counterRec[2] = 0;
                    counterSes[2] = 0;
                    break; // 立即跳出Spy状态，进入Danger处理
                }
            }
            std::cout<<"识别到spy"<<endl;
            if (stepSpy == StepSpy::Det) // AI检测嫌疑车辆所在赛道两侧
            {
                //prop替换
                PredictResult predict = searchSign(predicts, CarParams->racing_key ? LABEL_PROP : LABEL_SPY);
                counterSes[1]++;
                if (predict.x > 0 && predict.y > 0) // 检测到有效的AI标志
                {
                    
                    // 检索小车靠近左/右方向
                    sideLeft = true; // 靠左侧标志
                    int row = track.pointsEdgeLeft[0].row - predict.y - predict.height;
                    if (row < 0)
                    {
                        _index = 1;
                        if (abs(track.pointsEdgeLeft[0].col - predict.x) > abs(track.pointsEdgeRight[0].col - predict.x - predict.width))
                            sideLeft = false;
                    }
                    else if (row >= track.pointsEdgeLeft.size() || row >= track.pointsEdgeRight.size())
                    {
                        _index = 2;
                        if (abs(track.pointsEdgeLeft[track.pointsEdgeLeft.size() - 1].col - predict.x) > abs(track.pointsEdgeRight[track.pointsEdgeRight.size() - 1].col - predict.x - predict.width))
                            sideLeft = false;
                    }
                    else
                    {
                        _index = 3;
                        if (abs(track.pointsEdgeLeft[row].col - predict.x) > abs(track.pointsEdgeRight[row].col - predict.x - predict.width))
                            sideLeft = false;
                    }

                    counterSes[1] = 0;
                    counterRec[1]++;
                    if (counterRec[1] > 5)
                    {
                        counterRec[1] = 0;
                        counterSes[1] = 0;
                        stepSpy = StepSpy::Bypass;
                    }
                }
                else if (counterSes[1] > 20) // AI检测异常：退出该场景
                {
                    counterRec[1] = 0;
                    counterSes[1] = 0;
                    typeRace = TypeRace::None;
                }
            }
            else if (stepSpy == StepSpy::Bypass) // 车辆绕行阶段
            { 
                //prop替换
                PredictResult predict = searchSign(predicts, CarParams->racing_key ? LABEL_PROP : LABEL_SPY);
                int row = track.pointsEdgeLeft[0].row - predict.y - predict.height;
                counterSes[1]++;
                if (predict.x > 0 && predict.y > 0) // 检测到有效的AI标志
                  {  counterSes[1] = 0;
                     vector<mpoint> points(4); // 三阶贝塞尔曲线
                    if (sideLeft)
                    {
                        points[0] = track.pointsEdgeLeft[row / 2];
                        points[1] = {predict.y + predict.height, predict.x + 2 * predict.width + 90};
                        points[2] = {(predict.y + predict.height + predict.y) / 2, predict.x + 2 * predict.width + 90};
                        if (predict.y > track.pointsEdgeLeft[track.pointsEdgeLeft.size() - 1].row)
                            points[3] = {track.pointsEdgeLeft[track.pointsEdgeLeft.size() - 1].row, track.pointsEdgeLeft[track.pointsEdgeLeft.size() - 1].col + CarParams->racing_anglesize + 90};
                        else
                            points[3] = {predict.y, predict.x + predict.width + 90};

                        track.pointsEdgeLeft.resize((size_t)row / 2); // 删除错误路线
                        vector<mpoint> repair = Bezier(0.001, points); // 重新规划车道线
                        for (int i = 0; i < repair.size(); i++)
                            track.pointsEdgeLeft.push_back(repair[i]);
                    }
                else
                {
                    points[0] = track.pointsEdgeRight[row / 2];
                    points[1] = {predict.y + predict.height, predict.x - 2 * predict.width - 90};
                    points[2] = {(predict.y + predict.height + predict.y) / 2, predict.x - 2 * predict.width - 90};
                    if (predict.y > track.pointsEdgeRight[track.pointsEdgeRight.size() - 1].row)
                        points[3] = {track.pointsEdgeRight[track.pointsEdgeRight.size() - 1].row, track.pointsEdgeRight[track.pointsEdgeRight.size() - 1].col - CarParams->racing_anglesize - 90};
                    else
                        points[3] = {predict.y, predict.x - predict.width - 90};

                    track.pointsEdgeRight.resize((size_t)row / 2); // 删除错误路线
                    vector<mpoint> repair = Bezier(0.001, points); // 重新规划车道线
                     for (int i = 0; i < repair.size(); i++)
                        track.pointsEdgeRight.push_back(repair[i]);
                   }
                }        
                else if (counterSes[1] > 40) // 绕行完毕
                {
                    counterSes[1] = 0;
                    stepSpy = StepSpy::Inside;
                }
            }
            else if (stepSpy == StepSpy::Inside) // 车辆变道
            {
                curtailTracking(track, sideLeft); // 缩减优化车道线（双车道→单车道）
                counterSes[1]++;
                if (counterSes[1] > ai_params->racing_cruise_frame) // 变道完毕  20
                {
                    counterSes[1] = 0;
                    stepSpy = StepSpy::Resist;
                }
            }
            else if (stepSpy == StepSpy::Resist) // 停车阻挡逼停
            {
                carStoping = true;
                counterSes[1]++;
                if (counterSes[1] >ai_params->racing_spy_exit_time) // 停车逼停时间: 2.3s
                {
                    carStoping = false;
                    counterSes[1] = 0;
                    typeRace = TypeRace::None; // 完成，退出场景
                    racing_exit_status = true;
                    propInProgress = false; // 重置标志，表示可以进行新的prop识别
                    exit_timer.tic();
                }
            }

            break;
        }

        case TypeRace::Danger: // 危险车辆
        {  
           //typeRace=TypeRace::Spy;
            /**
             *恐怖车辆逼停策略：沿赛道左/右侧通行，强行撞击车辆逼停
             */
            std::cout<<"识别到Danger"<<typeRace<<endl;
        //prop替换
            PredictResult predict = searchSign(predicts, CarParams->racing_key ? LABEL_PROP : LABEL_DANGER);
            counterSes[2]++;
             if (predict.x > 0 && predict.y > 0) // 检测到有效的AI标志
             {counterSes[2] = 0;
             danger_side=1;
             int row = track.pointsEdgeLeft[0].row - predict.y - predict.height;
                if (row < 0)
                {
                    if (abs(track.pointsEdgeLeft[0].col - predict.x) > abs(track.pointsEdgeRight[0].col - predict.x - predict.width))
                       { sideLeft = false;
                        danger_side=2;}
                }
                else if (row >= track.pointsEdgeLeft.size() || row >= track.pointsEdgeRight.size())
                {
                    if (abs(track.pointsEdgeLeft[track.pointsEdgeLeft.size() - 1].col - predict.x) > abs(track.pointsEdgeRight[track.pointsEdgeRight.size() - 1].col - predict.x - predict.width))
                        {sideLeft = false;
                        danger_side=2;}
                }
                else
                {
                    if (abs(track.pointsEdgeLeft[row].col - predict.x) > abs(track.pointsEdgeRight[row].col - predict.x - predict.width))
                        {sideLeft = false;
                        danger_side=2;}
                }}
             if (1) // 检测到有效的AI标志
           // if (num_danger==0||race_danger.toc()>3000) // 检测到有效的AI标志
            {   //if(predict.x<=0)
                //{predict.x=1;}
                //if(predict.y<=0)
                //{predict.y=1;}
                // 检索小车靠近左/右方向
                int row = track.pointsEdgeLeft[0].row - predict.y - predict.height;
                if(danger_side==1)
                sideLeft = true; // 靠左侧标志
                else if(danger_side==2)sideLeft = false;
                cout<<"x:"<<predict.x<<endl;
                cout<<"y:"<<predict.y<<endl;
                predict.x=320-predict.x;
                vector<mpoint> points(4); // 三阶贝塞尔曲线
                if(sideLeft==true){
                    //safe右边
                // points[0] = track.pointsEdgeLeft[row / 2];
                // points[1] = {predict.y + predict.height, min(predict.x + 2 * predict.width + 90,319)};
                // points[2] = {(predict.y + predict.height + predict.y) / 2, min(predict.x + 2 * predict.width + 90,319)};
                // if (predict.y > track.pointsEdgeLeft[track.pointsEdgeLeft.size() - 1].row)
                //     points[3] = {track.pointsEdgeLeft[track.pointsEdgeLeft.size() - 1].row, min(track.pointsEdgeLeft[track.pointsEdgeLeft.size() - 1].col + CarParams->racing_anglesize + 90,319)};
                // else
                //     points[3] = {predict.y, min(predict.x + predict.width + 90,319)};

                // track.pointsEdgeLeft.resize((size_t)row / 2); // 删除错误路线
                // vector<mpoint> repair = Bezier(0.001, points); // 重新规划车道线
                // for (int i = 0; i < repair.size(); i++)
                //     track.pointsEdgeLeft.push_back(repair[i]);
                race_step=3;
                }
                else {
                    //safe左边
                // points[0] = track.pointsEdgeRight[row / 2];
                // points[1] = {predict.y + predict.height, predict.x - 2 * predict.width - 90};
                // points[2] = {(predict.y + predict.height + predict.y) / 2, predict.x - 2 * predict.width - 90};
                // if (predict.y > track.pointsEdgeRight[track.pointsEdgeRight.size() - 1].row)
                //     points[3] = {track.pointsEdgeRight[track.pointsEdgeRight.size() - 1].row, track.pointsEdgeRight[track.pointsEdgeRight.size() - 1].col - CarParams->racing_anglesize - 90};
                // else
                //     points[3] = {predict.y, predict.x - predict.width - 90};

                // track.pointsEdgeRight.resize((size_t)row / 2); // 删除错误路线
                // vector<mpoint> repair = Bezier(0.001, points); // 重新规划车道线
                // for (int i = 0; i < repair.size(); i++)
                //     track.pointsEdgeRight.push_back(repair[i]);
                race_step=4;
                
                }
            // vector<mpoint> points(3); // 三阶贝塞尔曲线
            // points[0] = {predict.y - 20, 0};
            // points[1] = {predict.y , max(0,predict.x -60)};
            // points[2] = {319,max(0,predict.x -20)};
            // // if (predict.y > track.pointsEdgeLeft[track.pointsEdgeLeft.size() - 1].row)
            // //     points[3] = {track.pointsEdgeLeft[track.pointsEdgeLeft.size() - 1].row, track.pointsEdgeLeft[track.pointsEdgeLeft.size() - 1].col + CarParams->racing_anglesize};
            // // else
            // //     points[3] = {predict.y, predict.x + predict.width};

            // track.pointsEdgeLeft.resize((size_t)(predict.y - 20)); // 删除错误路线
            // vector<mpoint> repair = Bezier(0.001, points); // 重新规划车道线
            // for (int i = 0; i < repair.size(); i++)
            //     track.pointsEdgeLeft.push_back(repair[i]);
                
               // curtailTracking(track, sideLeft); // 缩减优化车道线（双车道→单车道）
                //counterSes[2] = 0;
            }
            // else if (counterSes[2] > 20) // 退出该场景


            // if (counterSes[2] > (dangerDetected ? 25 : 20)) // 退出该场景
            // {   //num_danger=1;
            //    // race_danger.tic();
            //     counterSes[2] = 0;
            //     typeRace = TypeRace::None;
            // }

            
            break;
        }
        }

        if(racing_exit_status && exit_timer.toc()<ring_params->racing_exit_time ){ //屏蔽
            return true;
        }
        if (typeRace == TypeRace::None){
            racing_exit_status=false;
            propInProgress = false; // 重置标志，表示可以进行新的prop识别
            return false;
        }
            
        else{
            return true;

            }
    }


     void Racing::drawImage(Mat &img)
    {
        if (typeRace == TypeRace::Spy)
        {
            switch (stepSpy)
            {
            case StepSpy::Det:
                putText(img, "[4] RACE - SPY - Det", Point(COLSIMAGE / 2 - 50, 10), cv::FONT_HERSHEY_TRIPLEX, 0.3, cv::Scalar(0, 255, 0), 1, CV_AA);
                break;
            case StepSpy::Bypass:
                putText(img, "[4] RACE - SPY - Bypass", Point(COLSIMAGE / 2 - 50, 10), cv::FONT_HERSHEY_TRIPLEX, 0.3, cv::Scalar(0, 255, 0), 1, CV_AA);
                break;
            case StepSpy::Inside:
                putText(img, "[4] RACE - SPY - Inside", Point(COLSIMAGE / 2 - 50, 10), cv::FONT_HERSHEY_TRIPLEX, 0.3, cv::Scalar(0, 255, 0), 1, CV_AA);
                break;
            case StepSpy::Resist:
                putText(img, "[4] RACE - SPY - Resist", Point(COLSIMAGE / 2 - 50, 10), cv::FONT_HERSHEY_TRIPLEX, 0.3, cv::Scalar(0, 255, 0), 1, CV_AA);
                break;
            default:
                break;
            }
        }
        else if (typeRace == TypeRace::Danger)
            putText(img, "[4] RACE - DANGER", Point(COLSIMAGE / 2 - 30, 10), cv::FONT_HERSHEY_TRIPLEX, 0.3, cv::Scalar(0, 255, 0), 1, CV_AA);
        else if (typeRace == TypeRace::Safe)
            putText(img, "[4] RACE - Safe", Point(COLSIMAGE / 2 - 30, 10), cv::FONT_HERSHEY_TRIPLEX, 0.3, cv::Scalar(0, 255, 0), 1, CV_AA);

        putText(img, to_string(_index), Point(COLSIMAGE / 2 - 10, 40), cv::FONT_HERSHEY_TRIPLEX, 0.3, cv::Scalar(0, 255, 0), 1, CV_AA);
    }

 void Racing::searchTypeRace(Findline &track,vector<PredictResult> predicts)
    {
            
    bool foundDanger = false;
    
    // 检查是否存在危险车辆
    for (size_t i = 0; i < predicts.size(); i++)
    {
        if (predicts[i].type == LABEL_DANGER) // 危险车辆
        {   
            foundDanger = true;
            break;
        }
    }

    if (foundDanger)
    {
        // 如果检测到danger，直接进入Danger状态
        typeRace = TypeRace::Danger;
        counterRec[2] = 0;
        counterSes[2] = 0;
        return;
    }
        // 普通车辆AI连续帧检测
        for (size_t i = 0; i < predicts.size(); i++)
        {
            if (predicts[i].type == LABEL_SAFETY) // 普通车辆
            {
                counterRec[0]++;
                break;
            }
        }
        // 嫌疑车辆AI连续帧检测
        for (size_t i = 0; i < predicts.size(); i++)
        {
            if (predicts[i].type == LABEL_SPY) // 嫌疑车辆
            {
                counterRec[1]++;
                break;
            }
        }
        // 危险车辆AI连续帧检测
        for (size_t i = 0; i < predicts.size(); i++)
        {
            if (predicts[i].type == LABEL_DANGER) // 危险车辆
            { //danger_side=1;
            //  int row = track.pointsEdgeLeft[0].row - predicts[i].y - predicts[i].height;
            //     if (row < 0)
            //     {
            //         if (abs(track.pointsEdgeLeft[0].col - predicts[i].x) > abs(track.pointsEdgeRight[0].col - predicts[i].x - predicts[i].width))
            //            { sideLeft = false;
            //             danger_side=2;}
            //     }
            //     else if (row >= track.pointsEdgeLeft.size() || row >= track.pointsEdgeRight.size())
            //     {
            //         if (abs(track.pointsEdgeLeft[track.pointsEdgeLeft.size() - 1].col - predicts[i].x) > abs(track.pointsEdgeRight[track.pointsEdgeRight.size() - 1].col - predicts[i].x - predicts[i].width))
            //             {sideLeft = false;
            //             danger_side=2;}
            //     }
            //     else
            //     {
            //         if (abs(track.pointsEdgeLeft[row].col - predicts[i].x) > abs(track.pointsEdgeRight[row].col - predicts[i].x - predicts[i].width))
            //             {sideLeft = false;
            //             danger_side=2;}
            //     }
                counterRec[2]++;
                break;}
                
            }
                
        

        if (counterRec[0]) // 安全车辆场景检测
        {
            counterSes[0]++;
            if (counterRec[0] > 3 && counterSes[0] <= 8)
            {
                typeRace = TypeRace::Safe; // 场景类型
                counterRec[0] = 0;
                counterSes[0] = 0;
            }
            else if (counterSes[0] > 8)
            {
                counterRec[0] = 0;
                counterSes[0] = 0;
            }
        }
        if (counterRec[1]) // 嫌疑车辆场景检测
        {
            counterSes[1]++;
            if (counterRec[1] > 3 && counterSes[1] <= 8)
            {
                //danger误判
                if(counterRec[2] > 0)
                {
                    typeRace = TypeRace::Danger; // 场景类型
                    
                    counterRec[2] = 0;
                    counterSes[2] = 0;
                }
                // else if (counterSes[2] > 8)
                // {
                //     counterRec[2] = 0;
                //     counterSes[2] = 0;
                // }
                else
                {
                typeRace = TypeRace::Spy; // 场景类型
                counterRec[1] = 0;
                counterSes[1] = 0;
                stepSpy = StepSpy::Det;
                }
            }
            else if (counterSes[1] > 8)
            {
                counterRec[1] = 0;
                counterSes[1] = 0;
            }
        }
        if (counterRec[2]) // 危险车辆场景检测
        {
            counterSes[2]++;
            if (counterRec[2] > 3 && counterSes[2] <= 8)
            {
                typeRace = TypeRace::Danger; // 场景类型
                counterRec[2] = 0;
                counterSes[2] = 0;
            }
            else if (counterSes[2] > 8)
            {
                counterRec[2] = 0;
                counterSes[2] = 0;
            }
        }
    }


PredictResult Racing::searchSign(vector<PredictResult> predicts, int index)
    {
        PredictResult predict;
        predict.x = 0;
        predict.y = 0;
        predict.height = 0;
        predict.width = 0;
        // AI连续帧检测
        for (size_t i = 0; i < predicts.size(); i++)
        {
            if (predicts[i].type == index)
            {
                // 通过框大小过滤最佳目标
                if (predicts[i].height * predicts[i].width > predict.height * predict.width)
                {
                    predict = predicts[i];
                }
            }
        }
        return predict;
    }


void Racing::curtailTracking(Findline &track, bool left)
    {
        if (left) // 向左侧缩进
        {
            if (track.pointsEdgeRight.size() > track.pointsEdgeLeft.size())
                track.pointsEdgeRight.resize(track.pointsEdgeLeft.size());

             for (int i = 0; i < track.pointsEdgeRight.size(); i++){
                track.pointsEdgeRight[i].col = max(0,
                (int)(track.pointsEdgeRight[i].col*0.8 + track.pointsEdgeLeft[i].col)/2 -75);
            }
        }
        else // 向右侧缩进
        {
            if (track.pointsEdgeRight.size() < track.pointsEdgeLeft.size())
                track.pointsEdgeLeft.resize(track.pointsEdgeRight.size());

            for (int i = 0; i < track.pointsEdgeLeft.size(); i++){
                track.pointsEdgeLeft[i].col = min((COLSIMAGE-1),
                (int)((track.pointsEdgeRight[i].col + track.pointsEdgeLeft[i].col*1.5) / 2 +75));
            }
        }
    }
