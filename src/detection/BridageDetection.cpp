#include "BridageDetection.h"

bool Bridge::process(Findline &track, vector<PredictResult> predict)
    {
        if (!ai_params->bridge_change_scheme){
            if (bridgeEnable) // 坡道处理，直接降低前瞻，见motion.json
            {
                // if (track.pointsEdgeLeft.size() > ROWSIMAGE / 2 && track.pointsEdgeRight.size() > ROWSIMAGE / 2) // 切行，防止错误前瞻引发转向
                // {
                //     track.pointsEdgeLeft.resize(track.pointsEdgeLeft.size() / 2);
                //     track.pointsEdgeRight.resize(track.pointsEdgeRight.size() / 2);
                // }
                counterSession++;
                std::cout << "session" << counterSession <<"\n";
                if (counterSession > CarParams->bridge_count_exit){
                    counterRec = 0;
                    counterSession = 0;
                    bridgeEnable = false;
                    std::cout<<"****退出桥****"<<std::endl;
                    return false;
                }
                return true;
            }
            else // 检测坡道
            {
                for (int i = 0; i < predict.size(); i++)
                {   
                    if (predict[i].type == LABEL_BRIDGE){
                        counterRec_all++;
                    }
                    if (predict[i].type == LABEL_BRIDGE && 
                        predict[i].score > CarParams->bridge_score && 
                        (predict[i].y + predict[i].height) > ROWSIMAGE * ai_params->bridge_block_row)
                    {
                        counterRec++;
                        std::cout<<"识别坡道:"<<counterRec<<std::endl;
                        break;
                    }
                    if(counterRec_all>20&&counterRec<CarParams->bridge_count_enter){
                        counterRec = 0;
                        counterRec_all=0;
                        //std::cout<<"坡道误判"<<std::endl;
                    }
                }
                if (counterRec>=CarParams->bridge_count_enter)
                {
                    counterRec = 0;
                    bridgeEnable = true; // 检测到桥标志
                    std::cout<<"进入桥状态"<<std::endl;
                    return true;
                }

                return false;
            }
        }


        else{//备用方案
            if (bridgeEnable) // 坡道处理，直接降低前瞻，见motion.json
            {
                counterSession++;
                std::cout << "session" << counterSession <<"\n";
                if (counterSession > CarParams->bridge_count_exit){
                    counterRec = 0;
                    counterSession = 0;
                    bridgeEnable = false;
                    std::cout<<"****退出桥****"<<std::endl;
                    return false;
                }
                return true;
            }
            else // 检测坡道
            {
                for (int i = 0; i < predict.size(); i++)
                {
                    if (predict[i].type == LABEL_BRIDGE &&
                     predict[i].score > CarParams->bridge_score && 
                     (predict[i].y + predict[i].height) > ROWSIMAGE * ai_params->bridge_block_row)
                    {
                        counterRec++;
                        std::cout<<"识别坡道:"<<counterRec<<std::endl;
                        break;
                    }
                    else{
                        counterRec = 0;
                    }
                }
                if (counterRec >= ai_params->bridge_count_enter_changed)
                {
                    counterRec = 0;
                    bridgeEnable = true; // 检测到桥标志
                    std::cout<<"进入桥状态"<<std::endl;
                    return true;
                }

                return false;
            }
        }
    }


void Bridge::drawImage(Mat &image,Findline track)
    {
        // 赛道边缘
        for (int i = 0; i < track.pointsEdgeLeft.size(); i++)
        {
            circle(image, Point(track.pointsEdgeLeft[i].col, track.pointsEdgeLeft[i].row), 1,
                   Scalar(0, 255, 0), -1); // 绿色点
        }
        for (int i = 0; i < track.pointsEdgeRight.size(); i++)
        {
            circle(image, Point(track.pointsEdgeRight[i].col, track.pointsEdgeRight[i].row), 1,
                   Scalar(0, 255, 255), -1); // 黄色点
        }

        if (bridgeEnable)
            putText(image, "[1] BRIDGE - ENABLE", Point(COLSIMAGE / 2 - 30, 10), cv::FONT_HERSHEY_TRIPLEX, 0.3, cv::Scalar(0, 255, 0), 1, CV_AA);
    }
