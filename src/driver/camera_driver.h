#pragma once

#include <cstddef>
#include <memory>
#include <opencv2/core/base.hpp>
#define VIDEO_DEV "/dev/video0"
#define IMAGEWIDTH 320
#define IMAGEHEIGHT 240
#define FRAME_NUM 3
#include "camera_decoder.h"
#include "../include/json.hpp"
#include <chrono>
#include <cstdio>
#include <errno.h>
#include <fcntl.h>
#include <linux/videodev2.h>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <stdlib.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/time.h>
#include <unistd.h>
#include <cstring>
#include <atomic>
#include "../utils/ThreadUtil.hpp"


struct Buffer{
  void *start;
  unsigned int length;
  long long int timestamp;
};


class Camera {
public:
  struct Params {
    unsigned short bin_threshold;
    unsigned short AUTO_WHITE_BALANCE;
    unsigned short EXPOSURE_AUTO;
    unsigned short WHITE_BALANCE_TEMPERATURE; // 4600 # 白平衡
    unsigned short EXPOSURE_ABSOLUTE;         // 100 #157 #曝光绝对值
    short BRIGHTNESS;                         // 0
    unsigned short CONTRAST;                  // 64
    unsigned short SATURATION;                // 60 # 饱和度
    short HUE;                                // 0 #色度
    unsigned short SHARPNESS;                 // 2 #锐化，清晰度
    unsigned short BACKLIGHT_COMPENSATION;    // 1 # 背光补偿
    unsigned short GAMMA;                     // 100 #gamma值
    NLOHMANN_DEFINE_TYPE_INTRUSIVE(Params, bin_threshold, AUTO_WHITE_BALANCE,
                                   EXPOSURE_AUTO, WHITE_BALANCE_TEMPERATURE,
                                   EXPOSURE_ABSOLUTE, BRIGHTNESS, CONTRAST,
                                   SATURATION, HUE, SHARPNESS,
                                   BACKLIGHT_COMPENSATION, GAMMA

    );
  };
  Params params;
  int fd;
  struct v4l2_buffer buf;
  Buffer *buffers;
  unsigned int n_buffers;
  std::shared_ptr<Decoder> decoder = nullptr;

  struct v4l2_capability cap;
  struct v4l2_fmtdesc fmtdesc;
  struct v4l2_format fmt;
  struct v4l2_streamparm stream_para;




  Camera();
  ~Camera();
  bool v4l2Init();
  int v4l2_set_control_args();
  int v4l2_show_control_args();
  bool v4l2_mem_ops();
  bool v4l2_frame_preprocess();
  bool v4l2_frame_process(cv::Mat &output);
  void load_params();
  void v4l2_release();

};