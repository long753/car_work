#include "Control.h"
#include "Findline.h"
Control::Control() { servoPwm = PWMSERVOMID; }

void Control::servo_control(float error,Findline &findline,Scene roadScence) {
  for (size_t i = 0; i < 3; i++)
  {
    errorFiFo[i] = errorFiFo[i + 1];
    errorDtFiFo[i] = errorDtFiFo[i + 1];
  }
  errorFiFo[3] = (error);
  errorDtFiFo[3] = (errorFiFo[3] - errorFiFo[2]);
  

    if(abs(errorFiFo[3])<60&&findline.line_type== LineType::STRAIGHT){
      turnP=0.7*turnP;
      
    }
     if(abs(errorFiFo[3])<30&&findline.line_type== LineType::STRAIGHT
     ){
     // turnP=0.5*turnP;
      speed_up=1;
    }
    else {speed_up=0;}
   // turnP=1.5*turnP;
  //float tmp_d=fuzzy.Fuzzy_Kd(errorFiFo[3],errorDtFiFo[3]); 
  //float turnP = fuzzy.Fuzzy_Kp(errorFiFo[3],errorDtFiFo[3]); 
  float turnP2 = 0;
  //  cout<< "error" <<errorFiFo[3]<<endl;
   //if(abs(errorFiFo[3])>60){
   // turnP=1.5*turnP;
  //  tmp_d=10*tmp_d;}
  //   tmp_d=1.5*tmp_d;
  // }
  // else if(abs(errorFiFo[3])>20){
  //   turnP=1.2*turnP;
  //   tmp_d=1.2*tmp_d;
  // }
  // else {turnP=0.5*turnP;}
  int pwmDiff = errorFiFo[3] * turnP + errorFiFo[3] * fabs(errorFiFo.back()) * turnP2 + errorDtFiFo[3] * turnD;
  if(roadScence == Scene::DangerScene){
      turnP = CarParams->danger_p;
      turnD = CarParams->danger_d;
      pwmDiff = errorFiFo[3] * turnP + errorDtFiFo[3] * turnD;
  }
  this->servoPwm = (uint16_t)(PWMSERVOMID - pwmDiff);
  if (CarParams->PWM_show) {
    //std::cout<<"err1:"<<this->servoPwm << "\n";
  }
  // cout<<"PWM_sent   "<< servoPwm<<endl;
}

float Control::error_cal(Findline & findline) {
  int16_t i, sum;
  lastError = err_blur[0];
  sum = 0;

  for (i = 19; i >= 1; i--){
    err_blur[i] = err_blur[i - 1]; // 偏差滤波
    
  }
  err_blur[0] = 0.0;
 if (findline.endline_eight > this->qianzhan) // 反向
 {
    for (i = findline.endline_eight; i <findline.endline_eight+8 ; i++) // 摄像头前瞻修改位置， 通过修改i的范围来控制前瞻
    {
      sum++;
      err_blur[0] += (float)(findline.midline[i] - 160); // 74
      
    }
    err_blur[0] = err_blur[0] / sum;
 }
 else
 {
   err_blur[0] = (float)(findline.midline[this->qianzhan+8] - 160);
 
  }

  // if ((lastError * err_blur[0] < 0 )&& (fabs(lastError - err_blur[0]) > 80))//原来是80
  // {

  //   err_blur[0] = lastError;
  // }

  return err_blur[0];
 
}



void Control::motor_control(Scene scene) {
 
}
