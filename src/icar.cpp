#include "SmartCar.hpp"
#include "utils/algorithm.h"
#include <signal.h>
using namespace std;
using namespace cv;

void callbackSignal(int signum) {
  smartcar.uart->carControl(0, PWMSERVOMID); // 智能车停止运动

  cout << "====System Exit!!!  -->  CarStopping! " << signum << endl;
  exit(signum);
}

int main() {
  signal(SIGINT, callbackSignal); // 程序退出信号
  // std::cout<<"abs***********************-----------:"<<mymod((20-350),360)<<std::endl;
  smartcar.model_init();
  smartcar.uart_connect();
  smartcar.imagecapture.camera_init();
  smartcar.imagecapture.run();
  smartcar.ai_process_mul(); // AI推理
  smartcar.control.qianzhan = CarParams->qianzhan;
  if (CarParams->debug) {
    icarShow.show_init();
    icarShow.run();
  }
  if (CarParams->key_start_end) {
    while (1) {
      smartcar.uart->dataTransform(); // 调用函数，内部状态被更新
      if (smartcar.uart->lastReceivedKey == 1)
        break; // 检查更新后的键值
    }
  } else {
    std::cout << "---key starting is not working---" << std::endl;
  }

  while (1) {
    // 时间记录
    auto start = std::chrono::steady_clock::now();
    if (smartcar.uart->lastReceivedKey == 2)
      break;
    if (!smartcar.frame_process()) {
      continue;
    }
    // 赛道线识别
    smartcar.findline.search_line(smartcar.imageBinary);
    // smartcar.uart->angle_yaw 访问角度值                                                                                                                                                 
    // std::cout <<"angle_yaw:"<<smartcar.uart->angle_yaw<<endl;
    smartcar.findline.you_dan_diao_width();
    smartcar.findline.zuo_dan_diao_width();
    if (smartcar.findline.you_dan_diao_width() == 1) {
      // cout<<endl<<"you"<<endl;
    }
    if (smartcar.findline.zuo_dan_diao_width() == 1) {
      // cout<<"zuo"<<endl;
    }

    

    if (smartcar.Scene_check(Scene::NormalScene, Scene::RingScene) &&
        CarParams->ring) {
      smartcar.ring->circle_search(smartcar.findline,
                                   smartcar.uart->angle_yaw.load());
      if (smartcar.ring->ringStep != RingStep::None) { // 环岛
        smartcar.scene = Scene::RingScene;
      } else {
        smartcar.scene = Scene::NormalScene;
      }
    }
    // if (smartcar.Scene_check(Scene::NormalScene, Scene::CrossScene) &&
    //     CarParams->cross) {
    //   smartcar.cross->cross_search(smartcar.findline);
    //   if (smartcar.cross->crossroadType != CrossroadType::None) { // 十字
    //     smartcar.scene = Scene::CrossScene;
    //   } else {
    //     smartcar.scene = Scene::NormalScene;
    //   }
    // }

    if (smartcar.Scene_check(Scene::NormalScene, Scene::RescueScene) // 救援区
        && CarParams->rescue) {
      smartcar.rescue->angle_ = smartcar.uart->angle_yaw.load();
      if (smartcar.rescue->process(smartcar.findline,
                                   smartcar.predict_result)) {
        smartcar.set_Scene(Scene::RescueScene);
        smartcar.control.qianzhan = CarParams->rescue_qianzhan;
        if (smartcar.rescue->carStoping) {
          smartcar.uart->carControl(
              0, smartcar.control.servoPwm); // 串口通信控制车辆
        }

      } else {
        smartcar.set_Scene(Scene::NormalScene);
        smartcar.control.qianzhan = CarParams->qianzhan;
      }
    }

    if (smartcar.Scene_check(Scene::NormalScene, Scene::RacingScene) &&
        CarParams->racing) { // 追车
      if (smartcar.racing->process(smartcar.findline,
                                   smartcar.predict_result)) {
        smartcar.set_Scene(Scene::RacingScene);
        smartcar.control.racing_speed=CarParams->racing_speed;
      } else {
        smartcar.set_Scene(Scene::NormalScene);
      }
    }

    if (smartcar.Scene_check(Scene::NormalScene, Scene::DangerScene) &&
        CarParams->danger) { // 危险区
      if (smartcar.danger->process(smartcar.findline,
                                   smartcar.predict_result)) {
        smartcar.scene = Scene::DangerScene;
        smartcar.control.qianzhan = CarParams->danger_qianzhan;

      } else {
        smartcar.scene = Scene::NormalScene;
        smartcar.control.qianzhan = CarParams->qianzhan;
        smartcar.danger->dangerstep = DangerStep::None;
      }
    }
    if (smartcar.Scene_check(Scene::NormalScene, Scene::BridgeScene) &&
        CarParams->bridge) { // 桥
      if (smartcar.bridge->process(smartcar.findline,smartcar.predict_result)) {
        smartcar.set_Scene(Scene::BridgeScene);
        if (smartcar.bridge->bridgeEnable) {
          smartcar.control.qianzhan = CarParams->bridge_qianzhan;
        }
      } else {
        smartcar.set_Scene(Scene::NormalScene);
        smartcar.control.qianzhan = CarParams->qianzhan;
      }
    }
    if (CarParams->parking&&smartcar.Scene_check(Scene::NormalScene, Scene::ParkingScene)) { // parking
      if (smartcar.parking->process(smartcar.predict_result)) {
        smartcar.set_Scene(Scene::ParkingScene);
        if (smartcar.parking->countExit >
            CarParams->parking_count) { // 停车计数大于停车计数就停车 默认20
            std::cout <<"nowtime" << smartcar.parking->parking_timer.toc() << "\n";
          if (smartcar.parking->parking_timer.toc() > CarParams->parking_time) {
            smartcar.uart->carControl(0, PWMSERVOMID);
            sleep(1);
            std::cout << "System Exit !!!" << "\n";
            exit(0);
          }

          
        }
      }
    }

    // 速度控制
    smartcar.motion_control();
    icarShow.now_scene = smartcar.get_Scene_string(smartcar.scene);
    if (CarParams->debug) {
      switch (smartcar.scene) {
      case Scene::NormalScene:
        smartcar.show_debug_window([](cv::Mat img) {});
        break;
      case Scene::RingScene:
        smartcar.show_debug_window(smartcar.ring->get_draw_task());
        break;
      case Scene::DangerScene:
        smartcar.show_debug_window(smartcar.danger->get_draw_task());
        break;
      case Scene::RescueScene:
        smartcar.show_debug_window(smartcar.rescue->get_draw_task());
        break;
      case Scene::RacingScene:
        smartcar.show_debug_window(smartcar.racing->get_draw_task());
        break;
      // case Scene::CrossScene:
      //   smartcar.show_debug_window(smartcar.cross->get_draw_task());
      //   break;
      case Scene::BridgeScene:
        break;
      default:
        break;
      }
      auto end = std::chrono::steady_clock::now();
      auto elapsed = end - start;
      // std::cout << "Elapsed time: " << elapsed.count()/1000000  << "ms\n";
    }
  }
}
