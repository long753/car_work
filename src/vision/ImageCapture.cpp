#include "ImageCapture.h"
#include <iostream>
bool ImageCapture::camera_init() {
  std::string indexCapture = "/dev/video0";

// #define VIDEO
#ifdef VIDEO
  if (CarParams->debug) {
    indexCapture = "/home/edgeboard/work/res/samples/sample.mp4";
  }
#endif
  capture = VideoCapture(indexCapture, cv::CAP_V4L2);

  if (!capture.isOpened()) {
    std::cerr << "can not open video device " << std::endl;
    capture = VideoCapture("/dev/video1", cv::CAP_V4L2);
    // return false;
  }
  // }else{
  //   capture = VideoCapture("/dev/video1", cv::CAP_V4L2);
  // }

  capture.set(cv::CAP_PROP_FRAME_WIDTH, COLSIMAGE);
  capture.set(cv::CAP_PROP_FRAME_HEIGHT, ROWSIMAGE);
  capture.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));
  capture.set(cv::CAP_PROP_FPS, 90);
  // capture.set(cv::CAP_PROP_EXPOSURE,50);
  //capture.set(cv::CAP_PROP_AUTO_EXPOSURE,3);
  // capture.set(cv::CAP_PROP_BRIGHTNESS,0);
  // capture.set(cv::CAP_PROP_CONTRAST,8);


  return true;
}

void ImageCapture::raw_image_catch() {
   camera_thread = std::jthread{[this]() {
    while (capture_flag.load()) {
      Mat frame;
      if (!capture.read(frame)) {
        continue;
        // std::cerr << "no video frame" << std::endl;
      }
      if(image_update_flag.load() == false){
      //image_process.image_correct(frame);
      if(ring_params->video_rec){
        video_record<<frame;
      }
      rgb_image = std::move(frame);
      image_update_flag.store(true);
      }
    }
  }};
 
}

void ImageCapture::image_show() {

}



void ImageCapture::run(){
  raw_image_catch();
  
}


ImageCapture::~ImageCapture(){
  capture_flag.store(false);
  capture.release();
  
}
