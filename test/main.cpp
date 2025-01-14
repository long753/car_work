#include "../src/SmartCar.hpp"
#include <memory>
#include <vector>
#include <cmath>
#include <iostream>
#include <array>
#include <algorithm>
#include <signal.h>
#include "../include/stop_watch.hpp"

void callbackSignal(int signum)
{
  smartcar.uart->carControl(0, PWMSERVOMID); // 智能车停止运动
  cout << "====System Exit!!!  -->  CarStopping! " << signum << endl;
  exit(signum);
}

std::array<int,240> midline{0};
float err_blue[20];
int endline_eight = 240;
float lastError = 0;
float caculate_err(){
  int16_t i, sum;
  lastError = err_blue[0];
  sum = 0;
  for (i = 19; i >= 1; i--)
  err_blue[i] = err_blue[i - 1]; // 偏差滤波
  err_blue[0] = 0.0;
  if (endline_eight > CarParams->qianzhan){
    for (i = endline_eight; i <endline_eight+8 ; i++){ // 摄像头前瞻修改位置， 通过修改i的范围来控制前瞻
      sum++;
      err_blue[0] += (float)(midline[i] - 160); // 74
    }
    err_blue[0] = err_blue[0] / sum;
  }
  else {
   err_blue[0] = (float)(midline[CarParams->qianzhan+8] - 160);
  }

  if ((lastError * err_blue[0] < 0 )&& (fabs(lastError - err_blue[0]) > 80))//原来是80
  {

    err_blue[0] = lastError;
  }

  return err_blue[0];
}
std::array<float, 4> errorFiFo;
std::array<float, 4> errorDtFiFo;
void servo_control(float error)
{
  for (size_t i = 0; i < 3; i++)
  {
    errorFiFo[i] = errorFiFo[i + 1];
    errorDtFiFo[i] = errorDtFiFo[i + 1];
  }
  errorFiFo[3] = (error);
  errorDtFiFo[3] = (errorFiFo[3] - errorFiFo[2]);
  float tmp_d = CarParams->turnD;
  float turnP = CarParams->turnP;
  float turnP2 = 0;
  int pwmDiff = errorFiFo[3] * turnP + 
              errorFiFo[3] * fabs(errorFiFo.back()) * turnP2 + 
              errorDtFiFo[3] * tmp_d;
  auto servoPwm = (uint16_t)(PWMSERVOMID - pwmDiff); 
  smartcar.uart->carControl(CarParams->speedtest, servoPwm); 
}

int main()
{
  signal(SIGINT, callbackSignal); // 程序退出信号
  smartcar.model_init();
  smartcar.imagecapture.camera_init();
  smartcar.uart_connect();

  if(CarParams->debug){
    smartcar.icarShow.run();
  }
    smartcar.ai_process_mul();// AI推理
  while (1)
  { 
    if (!smartcar.frame_process())
    {
      continue;
    }
    std::array<int,240> leftpoint;
    std::array<int,240> rightpoint;
    midline = {0};
    smartcar.findline.search_line(smartcar.imageBinary);
    smartcar.ring->circle_search(smartcar.findline);
    
    leftpoint.fill(4);
    rightpoint.fill(315);
    endline_eight = 240;
    for(auto p : smartcar.findline.pointsEdgeLeft){
      leftpoint[p.row] = p.col;
      endline_eight = min(p.row,endline_eight);
    };
    for(auto p : smartcar.findline.pointsEdgeRight){
      rightpoint[p.row] = p.col;
      endline_eight = min(p.row,endline_eight);
    };
      midline.fill(160);
    for(int i = 238 ;i >=0 ; i--){
      if((leftpoint[i]==4)&&(rightpoint[i]==315)){
        midline[i]=midline[i+1];
      }
      else {
        midline[i] = (leftpoint[i]+rightpoint[i])/2;
      }
    }

    if (CarParams->debug){
      auto draw_line_task {[](cv::Mat m,int endline_eight,std::array<int,240> midline,std::vector<mpoint>pointsLeft,std::vector<mpoint>pointsRight,cv::Point gp
      ,cv::Point gp2 ,cv::Point gp3,cv::Point gp4){
      smartcar.findline.drawImage(m,pointsLeft,pointsRight);        // 图像显示赛道线识别结果
      for (size_t i = endline_eight; i < midline.size(); i++){
        circle(m, Point(midline[i], i), 3,
              Scalar(0, 0, 255), -1);                          // 红色点
      }
      circle(m,gp,3,Scalar(255, 255, 0), -1);
       circle(m,gp2,3,Scalar(255, 0, 0), -1);

      circle(m,gp3,3,Scalar(255, 0, 170), -1);
      circle(m,gp4,3,Scalar(170, 0, 255), -1);
        smartcar.icarShow.debug_window1.push(std::move(m));
      }
      };
      thread_pool.commit(draw_line_task,smartcar.imageCorrect.clone(),endline_eight,
      midline,smartcar.findline.pointsEdgeLeft,smartcar.findline.pointsEdgeRight,
      smartcar.ring->corner_up_point,smartcar.ring->corner_mid_point,
      smartcar.ring->corner_down_point,smartcar.ring->corner_edge);

    }
    if (!midline.empty()){
      auto e = caculate_err();
      servo_control(e);
    }
  }
}
