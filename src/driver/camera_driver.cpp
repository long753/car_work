#include "camera_driver.h"
#include <fstream>
#include <memory>
#include <opencv2/core/mat.hpp>
#include <opencv2/imgcodecs.hpp>
#include <thread>
#include <future>
#include <vector>



Camera::Camera() {
  buffers = nullptr;
  fd = 0;
  n_buffers = 0;

  decoder = std::make_shared<Decoder>();
  load_params();
}
Camera::~Camera() 
{ 
  this->v4l2_release(); 
}
bool Camera::v4l2Init() {
  struct v4l2_capability cap;
  struct v4l2_fmtdesc fmtdesc;
  struct v4l2_format fmt;
  struct v4l2_streamparm stream_para;

  if ((fd = open(VIDEO_DEV, O_RDWR)) == -1) {
    printf("Error opening V4L interface\n");
    return false;
  }

  //查询设备属性
  if (ioctl(fd, VIDIOC_QUERYCAP, &cap) == -1) {
    printf("Error opening device %s: unable to query device.\n", VIDEO_DEV);
    return false;
  } else {
    printf("driver:\t\t%s\n", cap.driver);
    printf("card:\t\t%s\n", cap.card);
    printf("bus_info:\t%s\n", cap.bus_info);
    printf("version:\t%d\n", cap.version);
    printf("capabilities:\t%x\n", cap.capabilities);

    if ((cap.capabilities & V4L2_CAP_VIDEO_CAPTURE) == V4L2_CAP_VIDEO_CAPTURE) {
      printf("Device %s: supports capture.\n", VIDEO_DEV);
    }

    if ((cap.capabilities & V4L2_CAP_STREAMING) == V4L2_CAP_STREAMING) {
      printf("Device %s: supports streaming.\n", VIDEO_DEV);
    }
  }

  //显示所有支持帧格式
  fmtdesc.index = 0;
  fmtdesc.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  printf("Support format:\n");
  while (ioctl(fd, VIDIOC_ENUM_FMT, &fmtdesc) != -1) {
    printf("\t%d.%s\n", fmtdesc.index + 1, fmtdesc.description);
    fmtdesc.index++;
  }

  //检查是否支持某帧格式
  struct v4l2_format fmt_test;
  fmt_test.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  fmt_test.fmt.pix.pixelformat = V4L2_PIX_FMT_RGB32;
  if (ioctl(fd, VIDIOC_TRY_FMT, &fmt_test) == -1) {
    printf("not support format RGB32!\n");
  } else {
    printf("support format RGB32\n");
  }

  //查看及设置当前格式
  printf("set fmt...\n");
  fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_MJPEG; // jpg格式
  // fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_YUYV;//yuv格式

  fmt.fmt.pix.height = IMAGEHEIGHT;
  fmt.fmt.pix.width = IMAGEWIDTH;
  fmt.fmt.pix.field = V4L2_FIELD_INTERLACED;
  printf("fmt.type:\t\t%d\n", fmt.type);
  printf("pix.pixelformat:\t%c%c%c%c\n", fmt.fmt.pix.pixelformat & 0xFF,
         (fmt.fmt.pix.pixelformat >> 8) & 0xFF,
         (fmt.fmt.pix.pixelformat >> 16) & 0xFF,
         (fmt.fmt.pix.pixelformat >> 24) & 0xFF);
  printf("pix.height:\t\t%d\n", fmt.fmt.pix.height);
  printf("pix.width:\t\t%d\n", fmt.fmt.pix.width);
  printf("pix.field:\t\t%d\n", fmt.fmt.pix.field);
  if (ioctl(fd, VIDIOC_S_FMT, &fmt) == -1) {
    printf("Unable to set format\n");
    return false;
  }

  printf("get fmt...\n");
  if (ioctl(fd, VIDIOC_G_FMT, &fmt) == -1) {
    printf("Unable to get format\n");
    return false;
  }
  {
    printf("fmt.type:\t\t%d\n", fmt.type);
    printf("pix.pixelformat:\t%c%c%c%c\n", fmt.fmt.pix.pixelformat & 0xFF,
           (fmt.fmt.pix.pixelformat >> 8) & 0xFF,
           (fmt.fmt.pix.pixelformat >> 16) & 0xFF,
           (fmt.fmt.pix.pixelformat >> 24) & 0xFF);
    printf("pix.height:\t\t%d\n", fmt.fmt.pix.height);
    printf("pix.width:\t\t%d\n", fmt.fmt.pix.width);
    printf("pix.field:\t\t%d\n", fmt.fmt.pix.field);
  }

  //设置及查看帧速率，这里只能是30帧，就是1秒采集30张图
  memset(&stream_para, 0, sizeof(struct v4l2_streamparm));
  stream_para.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  stream_para.parm.capture.timeperframe.denominator = 30; //帧率
  stream_para.parm.capture.timeperframe.numerator = 1;

  if (ioctl(fd, VIDIOC_S_PARM, &stream_para) == -1) {
    printf("Unable to set frame rate\n");
    return false;
  }

  if (ioctl(fd, VIDIOC_G_PARM, &stream_para) == -1) {
    printf("Unable to get frame rate\n");
    return false;
  }
  else
  {
    printf("numerator:%d\ndenominator:%d\nfps:%.2f\n",
           stream_para.parm.capture.timeperframe.numerator,
           stream_para.parm.capture.timeperframe.denominator,
           (float)stream_para.parm.capture.timeperframe.denominator /
               stream_para.parm.capture.timeperframe.numerator);
  }
  v4l2_set_control_args();
  v4l2_show_control_args();
  return true;
}

int Camera::v4l2_set_control_args() {
  struct v4l2_control *v4l2_control_arg = new v4l2_control();
  v4l2_control_arg->id = V4L2_CID_AUTO_WHITE_BALANCE;
  v4l2_control_arg->value = params.AUTO_WHITE_BALANCE;
  if (-1 == ioctl(fd, VIDIOC_S_CTRL, v4l2_control_arg)) // VIDIOC_S_CTRL
  {
    perror("ioctl: set V4L2_CID_AUTO_WHITE_BALANCE ERROR");
    exit(EXIT_FAILURE);
  }
  printf("自动白平衡：%d\n", v4l2_control_arg->value);

  v4l2_control_arg->id = V4L2_CID_EXPOSURE_AUTO;
  v4l2_control_arg->value = params.EXPOSURE_AUTO;
  if (-1 == ioctl(fd, VIDIOC_S_CTRL, v4l2_control_arg)) // VIDIOC_S_CTRL
  {
    perror("ioctl: set V4L2_CID_EXPOSURE_AUTO ERROR");
    exit(EXIT_FAILURE);
  }
  printf("自动曝光：%d\n", v4l2_control_arg->value);
  
    v4l2_control_arg->id = V4L2_CID_EXPOSURE_ABSOLUTE;
    v4l2_control_arg->value = params.EXPOSURE_ABSOLUTE;
    if (-1 == ioctl(fd, VIDIOC_S_CTRL, v4l2_control_arg)) // VIDIOC_S_CTRL
    {
      perror("ioctl: set V4L2_CID_EXPOSURE_ABSOLUTE ERROR");
      exit(EXIT_FAILURE);
    }
    printf("曝光绝对值：%d\n", v4l2_control_arg->value);
  
  v4l2_control_arg->id = V4L2_CID_BRIGHTNESS;
  v4l2_control_arg->value = params.BRIGHTNESS;
  if (-1 == ioctl(fd, VIDIOC_S_CTRL, v4l2_control_arg)) // VIDIOC_S_CTRL
  {
    perror("ioctl: set V4L2_CID_BRIGHTNESS ERROR");
    exit(EXIT_FAILURE);
  }
  printf("亮度：%d\n", v4l2_control_arg->value);

  v4l2_control_arg->id = V4L2_CID_CONTRAST;
  v4l2_control_arg->value = params.CONTRAST;
  if (-1 == ioctl(fd, VIDIOC_S_CTRL, v4l2_control_arg)) // VIDIOC_S_CTRL
  {
    perror("ioctl: set V4L2_CID_CONTRAST ERROR");
    exit(EXIT_FAILURE);
  }
  printf("对比度：%d\n", v4l2_control_arg->value);

  v4l2_control_arg->id = V4L2_CID_SATURATION;
  v4l2_control_arg->value = params.SATURATION;
  if (-1 == ioctl(fd, VIDIOC_S_CTRL, v4l2_control_arg)) // VIDIOC_S_CTRL
  {
    perror("ioctl: set V4L2_CID_SATURATION ERROR");
    exit(EXIT_FAILURE);
  }
  printf("饱和度：%d\n", v4l2_control_arg->value);

  v4l2_control_arg->id = V4L2_CID_HUE;
  v4l2_control_arg->value = params.HUE;
  if (-1 == ioctl(fd, VIDIOC_S_CTRL, v4l2_control_arg)) // VIDIOC_S_CTRL
  {
    perror("ioctl: set V4L2_CID_HUE ERROR");
    exit(EXIT_FAILURE);
  }

  printf("色度：%d\n", v4l2_control_arg->value);

  v4l2_control_arg->id = V4L2_CID_SHARPNESS;
  v4l2_control_arg->value = params.SHARPNESS;
  if (-1 == ioctl(fd, VIDIOC_S_CTRL, v4l2_control_arg)) // VIDIOC_S_CTRL
  {
    perror("ioctl: set V4L2_CID_SHARPNESS ERROR");
    exit(EXIT_FAILURE);
  }
  printf("锐度：%d\n", v4l2_control_arg->value);

  v4l2_control_arg->id = V4L2_CID_BACKLIGHT_COMPENSATION;
  v4l2_control_arg->value = params.BACKLIGHT_COMPENSATION;
  if (-1 == ioctl(fd, VIDIOC_S_CTRL, v4l2_control_arg)) // VIDIOC_S_CTRL
  {
    perror("ioctl: set V4L2_CID_BACKLIGHT_COMPENSATION ERROR");
    exit(EXIT_FAILURE);
  }
  printf("背光补偿：%d\n", v4l2_control_arg->value);

  v4l2_control_arg->id = V4L2_CID_GAMMA;
  v4l2_control_arg->value = params.GAMMA;
  if (-1 == ioctl(fd, VIDIOC_S_CTRL, v4l2_control_arg)) // VIDIOC_S_CTRL
  {
    perror("ioctl: set V4L2_CID_GAMMA ERROR");
    exit(EXIT_FAILURE);
  }
  printf("伽马值：%d\n", v4l2_control_arg->value);

  free(v4l2_control_arg);
  return 0;
}
int Camera::v4l2_show_control_args() {
  //查看参数控制能力
  struct v4l2_control *v4l2_control_arg = new v4l2_control();
  struct v4l2_queryctrl *Setting = new v4l2_queryctrl();
  v4l2_control_arg->id = V4L2_CID_AUTO_WHITE_BALANCE;
  if (-1 == ioctl(fd, VIDIOC_G_CTRL, v4l2_control_arg)) // VIDIOC_S_CTRL
  {
    perror("ioctl: get V4L2_CID_AUTO_WHITE_BALANCE ERROR");
    exit(EXIT_FAILURE);
  }
  Setting->id = V4L2_CID_AUTO_WHITE_BALANCE;
  if (-1 == ioctl(fd, VIDIOC_QUERYCTRL, Setting)) // VIDIOC_S_CTRL
  {
    perror("ioctl: get VIDIOC_QUERYCTRL ERROR");
    exit(EXIT_FAILURE);
  }
  printf("min:%d, max:%d, step:%d ", Setting->minimum, Setting->maximum,
         Setting->step);
  printf("自动白平衡：%d\n", v4l2_control_arg->value);

  v4l2_control_arg->id = V4L2_CID_EXPOSURE_AUTO;
  if (-1 == ioctl(fd, VIDIOC_G_CTRL, v4l2_control_arg)) // VIDIOC_S_CTRL
  {
    perror("ioctl: get V4L2_CID_EXPOSURE_AUTO ERROR");
    exit(EXIT_FAILURE);
  }
  Setting->id = V4L2_CID_EXPOSURE_AUTO;
  if (-1 == ioctl(fd, VIDIOC_QUERYCTRL, Setting)) // VIDIOC_S_CTRL
  {
    perror("ioctl: get VIDIOC_QUERYCTRL ERROR");
    exit(EXIT_FAILURE);
  }
  printf("min:%d, max:%d, step:%d ", Setting->minimum, Setting->maximum,
         Setting->step);
  printf("自动曝光：%d\n", v4l2_control_arg->value);

  v4l2_control_arg->id = V4L2_CID_WHITE_BALANCE_TEMPERATURE;
  if (-1 == ioctl(fd, VIDIOC_G_CTRL, v4l2_control_arg)) // VIDIOC_S_CTRL
  {
    perror("ioctl: get V4L2_CID_WHITE_BALANCE_TEMPERATURE ERROR");
    exit(EXIT_FAILURE);
  }
  Setting->id = V4L2_CID_WHITE_BALANCE_TEMPERATURE;
  if (-1 == ioctl(fd, VIDIOC_QUERYCTRL, Setting)) // VIDIOC_S_CTRL
  {
    perror("ioctl: get VIDIOC_QUERYCTRL ERROR");
    exit(EXIT_FAILURE);
  }
  printf("min:%d, max:%d, step:%d ", Setting->minimum, Setting->maximum,
         Setting->step);

  printf("白平衡：%d\n", v4l2_control_arg->value);

  v4l2_control_arg->id = V4L2_CID_EXPOSURE_ABSOLUTE;
  if (-1 == ioctl(fd, VIDIOC_G_CTRL, v4l2_control_arg)) // VIDIOC_S_CTRL
  {
    perror("ioctl: get V4L2_CID_EXPOSURE_ABSOLUTE ERROR");
    exit(EXIT_FAILURE);
  }
  Setting->id = V4L2_CID_EXPOSURE_ABSOLUTE;
  if (-1 == ioctl(fd, VIDIOC_QUERYCTRL, Setting)) // VIDIOC_S_CTRL
  {
    perror("ioctl: get VIDIOC_QUERYCTRL ERROR");
    exit(EXIT_FAILURE);
  }
  printf("min:%d, max:%d, step:%d ", Setting->minimum, Setting->maximum,
         Setting->step);
  printf("曝光绝对值：%d\n", v4l2_control_arg->value);

  v4l2_control_arg->id = V4L2_CID_BRIGHTNESS;
  if (-1 == ioctl(fd, VIDIOC_G_CTRL, v4l2_control_arg)) // VIDIOC_S_CTRL
  {
    perror("ioctl: get V4L2_CID_BRIGHTNESS ERROR");
    exit(EXIT_FAILURE);
  }
  Setting->id = V4L2_CID_BRIGHTNESS;
  if (-1 == ioctl(fd, VIDIOC_QUERYCTRL, Setting)) // VIDIOC_S_CTRL
  {
    perror("ioctl: get VIDIOC_QUERYCTRL ERROR");
    exit(EXIT_FAILURE);
  }
  printf("min:%d, max:%d, step:%d ", Setting->minimum, Setting->maximum,
         Setting->step);
  printf("亮度：%d\n", v4l2_control_arg->value);

  v4l2_control_arg->id = V4L2_CID_CONTRAST;
  if (-1 == ioctl(fd, VIDIOC_G_CTRL, v4l2_control_arg)) // VIDIOC_S_CTRL
  {
    perror("ioctl: get V4L2_CID_CONTRAST ERROR");
    exit(EXIT_FAILURE);
  }
  Setting->id = V4L2_CID_CONTRAST;
  if (-1 == ioctl(fd, VIDIOC_QUERYCTRL, Setting)) // VIDIOC_S_CTRL
  {
    perror("ioctl: get VIDIOC_QUERYCTRL ERROR");
    exit(EXIT_FAILURE);
  }
  printf("min:%d, max:%d, step:%d ", Setting->minimum, Setting->maximum,
         Setting->step);
  printf("对比度：%d\n", v4l2_control_arg->value);

  v4l2_control_arg->id = V4L2_CID_SATURATION;
  if (-1 == ioctl(fd, VIDIOC_G_CTRL, v4l2_control_arg)) // VIDIOC_S_CTRL
  {
    perror("ioctl: get V4L2_CID_SATURATION ERROR");
    exit(EXIT_FAILURE);
  }
  Setting->id = V4L2_CID_SATURATION;
  if (-1 == ioctl(fd, VIDIOC_QUERYCTRL, Setting)) // VIDIOC_S_CTRL
  {
    perror("ioctl: get VIDIOC_QUERYCTRL ERROR");
    exit(EXIT_FAILURE);
  }
  printf("min:%d, max:%d, step:%d ", Setting->minimum, Setting->maximum,
         Setting->step);
  printf("饱和度：%d\n", v4l2_control_arg->value);

  v4l2_control_arg->id = V4L2_CID_HUE;
  if (-1 == ioctl(fd, VIDIOC_G_CTRL, v4l2_control_arg)) // VIDIOC_S_CTRL
  {
    perror("ioctl: get V4L2_CID_HUE ERROR");
    exit(EXIT_FAILURE);
  }
  Setting->id = V4L2_CID_HUE;
  if (-1 == ioctl(fd, VIDIOC_QUERYCTRL, Setting)) // VIDIOC_S_CTRL
  {
    perror("ioctl: get VIDIOC_QUERYCTRL ERROR");
    exit(EXIT_FAILURE);
  }
  printf("min:%d, max:%d, step:%d ", Setting->minimum, Setting->maximum,
         Setting->step);
  printf("色度：%d\n", v4l2_control_arg->value);

  v4l2_control_arg->id = V4L2_CID_SHARPNESS;
  if (-1 == ioctl(fd, VIDIOC_G_CTRL, v4l2_control_arg)) // VIDIOC_S_CTRL
  {
    perror("ioctl: get V4L2_CID_SHARPNESS ERROR");
    exit(EXIT_FAILURE);
  }
  Setting->id = V4L2_CID_SHARPNESS;
  if (-1 == ioctl(fd, VIDIOC_QUERYCTRL, Setting)) // VIDIOC_S_CTRL
  {
    perror("ioctl: get VIDIOC_QUERYCTRL ERROR");
    exit(EXIT_FAILURE);
  }
  printf("min:%d, max:%d, step:%d ", Setting->minimum, Setting->maximum,
         Setting->step);
  printf("锐度：%d\n", v4l2_control_arg->value);

  v4l2_control_arg->id = V4L2_CID_BACKLIGHT_COMPENSATION;
  if (-1 == ioctl(fd, VIDIOC_G_CTRL, v4l2_control_arg)) // VIDIOC_S_CTRL
  {
    perror("ioctl: get V4L2_CID_BACKLIGHT_COMPENSATION ERROR");
    exit(EXIT_FAILURE);
  }
  Setting->id = V4L2_CID_BACKLIGHT_COMPENSATION;
  if (-1 == ioctl(fd, VIDIOC_QUERYCTRL, Setting)) // VIDIOC_S_CTRL
  {
    perror("ioctl: get VIDIOC_QUERYCTRL ERROR");
    exit(EXIT_FAILURE);
  }
  printf("min:%d, max:%d, step:%d ", Setting->minimum, Setting->maximum,
         Setting->step);
  printf("背光补偿：%d\n", v4l2_control_arg->value);

  v4l2_control_arg->id = V4L2_CID_GAMMA;
  if (-1 == ioctl(fd, VIDIOC_G_CTRL, v4l2_control_arg)) // VIDIOC_S_CTRL
  {
    perror("ioctl: get V4L2_CID_GAMMA ERROR");
    exit(EXIT_FAILURE);
  }
  Setting->id = V4L2_CID_GAMMA;
  if (-1 == ioctl(fd, VIDIOC_QUERYCTRL, Setting)) // VIDIOC_S_CTRL
  {
    perror("ioctl: get VIDIOC_QUERYCTRL ERROR");
    exit(EXIT_FAILURE);
  }
  printf("min:%d, max:%d, step:%d ", Setting->minimum, Setting->maximum,
         Setting->step);
  printf("伽马值：%d\n", v4l2_control_arg->value);

  delete v4l2_control_arg;
  delete Setting;
  return 0;
}

bool Camera::v4l2_mem_ops() {
  unsigned int n_buffers;
  struct v4l2_requestbuffers req;

  //申请帧缓冲
  req.count = FRAME_NUM;
  req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  req.memory = V4L2_MEMORY_MMAP;
  if (ioctl(fd, VIDIOC_REQBUFS, &req) == -1) {
    printf("request for buffers error\n");
    return false;
  }

  // 申请用户空间的地址列
  //buffers = (Buffer *)malloc(req.count * sizeof(*buffers));
  buffers = (Buffer *)malloc(req.count * sizeof(Buffer));

  // buffers = new Buffer();

  if (!buffers) {
    printf("out of memory!\n");
    return false;
  }

  // 进行内存映射
  for (n_buffers = 0; n_buffers < FRAME_NUM; n_buffers++) {
    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_MMAP;
    buf.index = n_buffers;
    //查询->申请内核缓存区
    if (ioctl(fd, VIDIOC_QUERYBUF, &buf) == -1) {
      printf("query buffer error\n");
      return false;
    }
    //映射
    buffers[n_buffers].length = buf.length;
    buffers[n_buffers].start = mmap(NULL, buf.length, PROT_READ | PROT_WRITE,
                                    MAP_SHARED, fd, buf.m.offset);
    if (buffers[n_buffers].start == MAP_FAILED) {
      printf("buffer map error\n");
      return false;
    }
  }

  if (decoder == nullptr) {
    printf("ERROR in init decoder \n");
    return false;
  } 
  return true;
}
bool Camera::v4l2_frame_preprocess() {
  enum v4l2_buf_type type;

  //入队和开启采集
  for (n_buffers = 0; n_buffers < FRAME_NUM; n_buffers++) {
    buf.index = n_buffers;
    ioctl(fd, VIDIOC_QBUF, &buf);
  }
  type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  ioctl(fd, VIDIOC_STREAMON, &type);

  n_buffers = 0;

  buf.index = n_buffers;
  ioctl(fd, VIDIOC_DQBUF, &buf); //出队 因此要在读取缓冲区前，先调用VIDIOC_DQBUG
                                 //ioctl 通知驱动程序不要使用此缓冲区，

  printf("buf_len:%d\n", buffers[n_buffers].length);
  //处理数据只是简单写入文件，名字以loop的次数和帧缓冲数目有关
  printf("grab image data OK\n");

  //入队循环
  ioctl(fd, VIDIOC_QBUF, &buf);
  decoder->preDecode((uint8_t *)buffers[n_buffers].start,
                     buffers[n_buffers].length);

  n_buffers++;
  printf("process image successed \n");
  return true;
}
bool Camera::v4l2_frame_process(cv::Mat &output) {

  buf.index = n_buffers;
  
  ioctl(fd, VIDIOC_DQBUF, &buf); //出队
  decoder->decode(output, (unsigned char *)buffers[n_buffers].start,
                  buffers[n_buffers].length);
  ioctl(fd, VIDIOC_QBUF, &buf); //入队
  n_buffers = (n_buffers+1) & FRAME_NUM;
  return true;
}



void Camera::v4l2_release() {
  enum v4l2_buf_type type;
  //关闭流
  type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  if (fd != 0) {
    ioctl(fd, VIDIOC_STREAMOFF, &type);
  }

  //关闭内存映射
  if (buffers != nullptr) {
    for (n_buffers = 0; n_buffers < FRAME_NUM; n_buffers++) {
      munmap(buffers[n_buffers].start, buffers[n_buffers].length);
    }
    //释放自己申请的内存

    free(buffers);
  }
  //关闭设备
  if (fd != 0) {
    close(fd);
  }
}

void Camera::load_params() {
  std::string jsonPath = "../src/config/camera.json";
  std::ifstream config_is(jsonPath);
  if (!config_is.good()) {
    std::cout << "Error: Params file path:[" << jsonPath << "] not find .\n";
    exit(-1);
  }
  nlohmann::json js_value;
  config_is >> js_value;

  try {
    params = js_value.get<Params>();
  } catch (const nlohmann::detail::exception &e) {
    std::cerr << "Json Params Parse failed :" << e.what() << '\n';
    exit(-1);
  }
}