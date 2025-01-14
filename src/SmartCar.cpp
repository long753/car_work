#include "SmartCar.hpp"
#include "config/CarConfig.hpp"
#include "include/stop_watch.hpp"
#include <chrono>
#include <filesystem>
#include <iostream>
#include <memory>
#include <mutex>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>
#include <string>
#include <thread>
#include "./detection/RacingDetection.h"
//bool turn_angel=0;
StopWatch time2;

float straight_speed=CarParams->speedHigh;
float max_speed=CarParams->accelerate_speed;
float min_speed=CarParams->speedHigh;
//float flag_straight=0;
SmartCar::SmartCar() {
  if (CarParams->ring) {
    ring = std::make_unique<Ring>();
  }
  if (CarParams->cross) {
    cross = std::make_unique<Cross>();
  }
  if (CarParams->danger) {
    danger = std::make_unique<Danger>();
  }
  if (CarParams->rescue) {
    rescue = std::make_unique<Rescue>();
  }
  if (CarParams->racing) {
    racing = std::make_unique<Racing>();
  }
  if(CarParams->bridge){
    bridge = std::make_unique<Bridge>();
  }
  if (CarParams->parking) {
    parking = std::make_unique<Parking>();
  }
  if (CarParams->bridge){
    bridge = std::make_unique<Bridge>();
  }
}
// 元素状态重置

bool SmartCar::uart_connect() {
  // USB转串口的设备名为 / dev/ttyUSB0
  uart = std::make_shared<Uart>("/dev/ttyUSB0");
  if (uart == nullptr) {
    std::cerr << "Create Uart-Driver Error!" << std::endl;
    return false;
  }
  // 串口初始化，打开串口设备及配置串口数据格式
  int ret = uart->open();
  if (ret != 0) {
    std::cerr << "Uart Open failed!" << std::endl;
    return false;
  }
  std::cout << "Uart Open Success" << std::endl;
  uart->startReceive();
  return true;
}

void SmartCar::wait_sub_key_press() {
  while (!uart->keypress) {
    cv::waitKey(30);
  }
  uart->keypress = false;
}

bool SmartCar::model_init() {
  detection = std::make_shared<Detection>("../res/model/yolov3_mobilenet_v1");
  detection->score = CarParams->score;
  // ai_process = std::make_shared<AiProcess>();
  // ai_process->ai_process_start();
  return true;
}

void SmartCar::ai_process_mul(){
  auto ai_process_task{
    [](std::shared_ptr<Detection> detection){
      cv::Mat m;
      while(smartcar.ai_runable){
      //auto start =   std::chrono::steady_clock::now();
      if(!smartcar.ai_picture_update.load()){
        continue;
      }
      m = std::move(smartcar.image_for_predict);
      auto start =  std::chrono::steady_clock::now();
      detection->inference(m);
      if (CarParams->inference_time_view){
        auto end =   std::chrono::steady_clock::now();
        auto elapsed= end - start;
        std::cout << "Elapsed time1: " << elapsed.count()/1000000  << "ms\n";
      }
      smartcar.ai_picture_update = false;
      //std::copy(detection->results.begin(),detection->results.end(),smartcar.predict_result.begin());
      smartcar.predict_result = detection->results;
      if (CarParams->debug){
        auto draw_ai_task{
          [](cv::Mat m,std::vector<PredictResult> results){
            icarShow.ai_draw_result(m,results);
            if(icarShow.debug_image2_mutex.try_lock()){
            icarShow.debug_image2 = std::move(m);
            icarShow.debug_image2_mutex.unlock();
        }
          }
        };
        thread_pool.commit(draw_ai_task,std::move(m),detection->results);
      }

      }
    }
  };
  
  predict_thread= std::jthread(ai_process_task,this->detection);
  // ai_process->commit_ai_task(this->image_for_predict);
}

// 帧处理
bool SmartCar::frame_process() {
  if(this->imagecapture.image_update_flag.load()){
  imageCorrect = std::move(this->imagecapture.rgb_image);
    this->imagecapture.image_update_flag.store(false);

  if(imageCorrect.empty()){
    return false;
  }
  imageBinary =  this->imagecapture.image_process.image_binary(imageCorrect); // Gray  
  if(!this->ai_picture_update.load()){
    this->image_for_predict = imageCorrect.clone();
    this->ai_picture_update.store(true);
  }
  
  return true;
  }
  else{
    return false;
  }

}
SmartCar::~SmartCar(){
  ai_runable.store(false);
}

void SmartCar::set_Scene(Scene s) { scene = s; }
std::string SmartCar::get_Scene_string(Scene s) {
  switch (s) {
  case Scene::NormalScene:
    return "Normal";
  case Scene::CrossScene:
    return "Crossroad";
  case Scene::RingScene:
    return "Ring";
  case Scene::BridgeScene:
    return "Bridge";
  case Scene::DangerScene:
    return "Danger";
  case Scene::RescueScene:
    return "Rescue";
  case Scene::RacingScene:
    return "Racing";
  case Scene::BlocksScene:
    return "Blocks";
  case Scene::ParkingScene:
    return "Parking";
  default:
    return "Error";
  }
}

bool SmartCar::find_line_init() {
  findline.rowCutUp = CarParams->rowCutUp;
  findline.rowCutBottom = CarParams->rowCutBottom;
  return true;
}

// 运动控制
void SmartCar::motion_control() {
    control.turnD = CarParams->turnD;
    control.turnP = CarParams->turnP;
   if ((scene == Scene::RescueScene && rescue->carStoping)  || //删除了|| parking->park
          racing->carStoping) {
            control.motorSpeed = 0;
    }     
    else if (scene == Scene::RescueScene ){//rescue
      findline.line_type = LineType::STRAIGHT; //强制不使用单边
      if (rescue->carExitting){//倒车
        control.motorSpeed = -1 * ring_params->rescue_exit_speed;
      }
      else if (rescue->step==Rescue::Step::Cruise){
        control.motorSpeed = CarParams->rescue_Cruise_speed;
      }
      else{
        control.motorSpeed = CarParams->speedDown;
      }
    }
    else if (scene == Scene::DangerScene) {//danger
    
    //test
    //   // 在Danger场景下使用陀螺仪
    //   float gyroscope_angle = uart->angle_yaw.load();
    //   float some_threshold = 30.0; // 假设设定的阈值为10度
      

    //   if (gyroscope_angle > some_threshold) {
    // // 当角度超过10度，向左调整方向
    //     control.servoPwm = some_pwm_value_left;
    //   } else if (gyroscope_angle < -some_threshold) {
    //   // 当角度小于-10度，向右调整方向
    //       control.servoPwm = some_pwm_value_right;
    //   } else {
    // // 角度在阈值内，不调整方向
    //       control.servoPwm = pwmservomid;
    //   }
    //   gyroscope_angle =0;




      if (danger->dangerstep == DangerStep::Cruise){
        control.motorSpeed = CarParams->danger_speed;
      }
      findline.line_type = LineType::STRAIGHT;
    }// 减速  
    else if (scene == Scene::BridgeScene){
      control.motorSpeed = CarParams->speedHigh; //上桥加速

    } // 坡道速度
    else if (scene == Scene::NormalScene){
      //now_speed=CarParams->speedHigh;
      control.motorSpeed = straight_speed;
      // cout<<"speed:"<<control.motorSpeed<<"straight_speed:"<<straight_speed<<endl;
      //now_speed=CarParams->speedHigh;
      if (CarParams->accelerate){
        float correlationLeft = calculateCorrelation(findline.pointsEdgeLeft);
        float correlationRight = calculateCorrelation(findline.pointsEdgeRight);
        cout<<"corleft:"<<correlationLeft<<endl;
        cout<<"corright:"<<correlationRight<<endl;
        if (fabs(correlationLeft)>0.97&&fabs(correlationRight)>0.97){
        // findline.zuodandiao();
        // findline.youdandiao();
        // if(findline.zuo_num>5&&findline.you_num>5){ //判断直道
         // control.motorSpeed = CarParams->accelerate_speed;
          straight_speed=min(straight_speed+0.1,(double)max_speed);
          cout<<"finish_acc"<<endl;

        }
        else{
           straight_speed=max(straight_speed-0.1,(double)min_speed);

        }
      }
    } else if(scene == Scene::RingScene){    
     control.motorSpeed = CarParams->speedLow;      //圆环单边
     if(ring->ringType == RingType::RingLeft){
      control.turnP = CarParams->ring_p;
     control.turnD = CarParams->ring_d;
     }
     
     findline.line_type=LineType::STRAIGHT;
    }
    else if (scene == Scene::RacingScene){
      control.motorSpeed = CarParams->racing_speed; //减速
      
    }

    findline.midline_calculate(); //中线计算
    control.servo_control(control.error_cal(findline),findline,scene);
    // if(control.speed_up==1)
    // {control.motorSpeed=6.2;}
    // if(smartcar.findline.line_type != LineType::STRAIGHT){
    //   control.motorSpeed = CarParams->speedLow;
    // }



       uart->carControl(control.motorSpeed, control.servoPwm); // 串口通信控制车辆
    
    if (scene == Scene::RacingScene && racing->race_step==3) {

            // 在Danger场景下使用陀螺仪
            if(!SmartCar::turn_angel)
            {
              SmartCar::gyroscope_angle = uart->angle_yaw.load();
              //float some_threshold = 30.0; // 假设设定的阈值为10度
              SmartCar::turn_angel=1;
            }
            float gyroscope_angle_current= uart->angle_yaw.load();
            cout<<"danger zuo"<<endl;
            cout<<"start_angel_zuo"<<gyroscope_angle<<endl;
            cout<<"current_angel_zuo"<<gyroscope_angle_current<<endl;
            uart->carControl(control.motorSpeed,990);
            if(gyroscope_angle_current>gyroscope_angle+5){
              if(gyroscope_angle_current-gyroscope_angle>ring_params->racing_danger_angle_left){
                //出状态
                racing->typeRace  = racing->TypeRace::None;
                racing->counterSes[2] = 0;
                SmartCar::turn_angel=0;
                std::cout<<"left racing_danger finish"<<std::endl;
              }
            }
            if(gyroscope_angle_current<gyroscope_angle-5){
              if(360-gyroscope_angle+gyroscope_angle_current>ring_params->racing_danger_angle_left){
                //出状态
                racing->typeRace  = racing->TypeRace::None;
                racing->counterSes[2] = 0;
                SmartCar::turn_angel=0;
                std::cout<<"left racing_danger finish"<<std::endl;
              }
            }

            // timer_smart.tic();
            // while(timer_smart.toc()<500);
    }     
    if (scene == Scene::RacingScene && racing->race_step==4) {

            // 在Danger场景下使用陀螺仪
            if(!SmartCar::turn_angel)
            {
              SmartCar::gyroscope_angle = uart->angle_yaw.load();
              //float some_threshold = 30.0; // 假设设定的阈值为10度
              SmartCar::turn_angel=1;
            }
            float gyroscope_angle_current= uart->angle_yaw.load();
            cout<<"danger you"<<endl;
            cout<<"start_angel_you"<<gyroscope_angle<<endl;
            cout<<"current_angel_you"<<gyroscope_angle_current<<endl;
            uart->carControl(control.motorSpeed,780); 

            if(gyroscope_angle_current>gyroscope_angle+5){
              if(360-gyroscope_angle_current+gyroscope_angle>ring_params->racing_danger_angle_right){
                //出状态
                racing->typeRace  = racing->TypeRace::None;
                racing->counterSes[2] = 0;
                SmartCar::turn_angel=0;
                std::cout<<"right racing_danger finish"<<std::endl;
              }
            }
            if(gyroscope_angle_current<gyroscope_angle){
              if(gyroscope_angle-gyroscope_angle_current>ring_params->racing_danger_angle_right){
                //出状态
                racing->typeRace  = racing->TypeRace::None;
                racing->counterSes[2] = 0;
                SmartCar::turn_angel=0;
                std::cout<<"right racing_danger finish"<<std::endl;
              }
            }

            // timer_smart.tic();
            // while(timer_smart.toc()<500);
    }     
    if (scene == Scene::RacingScene && racing->race_step==1) {
            // control.turnP = 2;
            // control.turnD =3;
            // uart->carControl(control.motorSpeed,ring_params->right_enter_pwm);
            // timer_smart.tic();
            // while(timer_smart.toc()<10);
            // uart->carControl(control.motorSpeed,PWMSERVOMID);
            // timer_smart.tic();
            // while(timer_smart.toc()<10);
            // uart->carControl(control.motorSpeed,ring_params->left_enter_pwm);
            // timer_smart.tic();
            // while(timer_smart.toc()<10);
    }     

   
}

void SmartCar::out_border_inspect() {
  int i = 0, j = 0, count_black = 0;
  for (i = 208; i > 205; i--) {
    for (j = 130; j >= 30; j--) {
      if (imageBinary.at<uchar>(i, j) == 0) {
        count_black = count_black + 1;
      }
    }
  }
  if (count_black > 350) {
    findline.loseflag = true;
  } else {
    findline.loseflag = false;
  }
}
