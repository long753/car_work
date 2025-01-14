#ifndef INCLUDE_SMARTCAR_HPP
#define INCLUDE_SMARTCAR_HPP


#include "config/CarConfig.hpp"
#include "control/Control.h"
#include "detection/BridageDetection.h"
#include "detection/DangerDetection.h"
#include "detection/ParkingDetection.h"
#include "detection/RacingDetection.h"
#include "detection/RescueDetection.h"
#include "driver/uart.hpp"
#include "driver/camera_driver.h"
#include "element/CrossElement.h"
#include "element/Findline.h"
#include "element/RingElement.h"
#include "include/common.hpp"
#include "include/detection.hpp"
#include "utils/IcarShow.h"
#include "utils/SingletonHolder.hpp"
#include "utils/ThreadUtil.hpp"
//#include "utils/Logger.hpp"
#include "vision/ImageProcess.h"
#include "vision/ImageCapture.h"
#include <chrono>
#include <memory>
#include <thread>
#include <utility>
#include <atomic>



using namespace cv;


class SmartCar : public SingletonHolder<SmartCar> {
public:
  uint16_t counterRunBegin = 1; // 智能车启动计数器：等待摄像头图像帧稳定
  uint16_t counterOutTrackA = 0;        // 车辆冲出赛道计数器A
  uint16_t counterOutTrackB = 0;        // 车辆冲出赛道计数器B
  Scene scene = Scene::NormalScene;     // 初始化场景：常规道路
  Scene sceneLast = Scene::NormalScene; // 记录上一次场景状态
  ImageCapture imagecapture;
  std::shared_ptr<Uart> uart = nullptr; // 初始化串口驱动
  // std::jthread predict_thread; //AI多线程
  std::atomic<bool> ai_runable{true};
  std::atomic<bool> ai_using_picture{false};
  std::atomic<bool> ai_picture_update{false};
  std::atomic<bool> ai_result_using {false};
  // ImageProcess imageprocess;
  Findline findline;
  StopWatch timer_smart;
  std::shared_ptr<Detection> detection = nullptr;

  cv::Mat imageCorrect;
  cv::Mat imageBinary;
  cv::Mat image_for_predict;

  std::mutex camera_mutex;
  std::mutex predict_mutex;
  int detection_nums;
  float gyroscope_angle;
  bool turn_angel;
  //float now_speed;

  std::vector<PredictResult> predict_result;
  std::jthread predict_thread;
  Control control;
  

  std::unique_ptr<Ring> ring = nullptr;
  std::unique_ptr<Cross> cross = nullptr;
  std::unique_ptr<Bridge> bridge = nullptr;
  std::unique_ptr<Danger> danger = nullptr;
  std::unique_ptr<Parking> parking = nullptr;
  std::unique_ptr<Racing> racing = nullptr;
  std::unique_ptr<Rescue> rescue = nullptr;

  int Time = 0;

  SmartCar();
  bool uart_connect();
  bool model_init();

  void ai_process_mul();
  template <typename... T> bool Scene_check(T &&...t) {
    return ((t == scene) || ...);
  }
  void set_Scene(Scene s);
  std::string get_Scene_string(Scene s);
  bool find_line_init();
  template<class F>
  void show_debug_window(F && _element_draw){
    if(CarParams->debug){

  auto debug_task {
    [this](auto && f1,auto && f2,cv::Mat img) mutable{
       cv::cvtColor(img,img,cv::COLOR_GRAY2BGR);
      std::invoke(f1,img);
      std::invoke(f2,img);
      cv::line(img,Point(0,control.qianzhan),Point(319,control.qianzhan),Scalar(170,150,120));
       if(icarShow.debug_image1_mutex.try_lock()){
          icarShow.debug_image1 = std::move(img);
          icarShow.debug_image1_mutex.unlock();
      }
    }
  };
  thread_pool.commit(debug_task,
  findline.get_findline_draw_task(),std::forward<F>(_element_draw),imageBinary.clone());
}
  }
 
  bool frame_process();
  void motion_control();
  void wait_sub_key_press();
  void out_border_inspect();
  ~SmartCar();
};

inline static auto &smartcar = SmartCar::get_instance();

#endif