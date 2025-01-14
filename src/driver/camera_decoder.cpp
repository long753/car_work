#include "camera_decoder.h"
#include <chrono>
#include <memory>
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>

Decoder::Decoder() {

  avPkt = av_packet_alloc();
  mjpegCodec = avcodec_find_decoder(AV_CODEC_ID_MJPEG);
  if (mjpegCodec == nullptr) {
    std::cerr << "ERROR unsupported codec!" << std::endl;
  }
  codecCtx = avcodec_alloc_context3(mjpegCodec);
  if (codecCtx == nullptr) {
    std::cerr << "ERROR codecCtx!" << std::endl;
  }
  avcodec_open2(codecCtx, mjpegCodec, nullptr);
  originFrame = av_frame_alloc();
  BGRFrame = av_frame_alloc();
}
Decoder::~Decoder() {
  if (colorSpcCvtCxt != nullptr) {
    sws_freeContext(colorSpcCvtCxt);
  }

  if (BGRFrame) {
    av_frame_free(&BGRFrame);
  }
  if (originFrame) {
    av_frame_free(&originFrame);
  }
  if (avPkt) {
    av_packet_unref(avPkt);
  }
  if (avPkt) {
    av_packet_free(&avPkt);
  }
  if (codecCtx) {
    avcodec_free_context(&codecCtx);
  }
}

AVPixelFormat Decoder::deprecatedImprove(AVPixelFormat pixFormat) {
  switch (pixFormat) {
  case AV_PIX_FMT_YUVJ420P:
    return AV_PIX_FMT_YUV420P;
  case AV_PIX_FMT_YUVJ422P:
    return AV_PIX_FMT_YUV422P;
  case AV_PIX_FMT_YUVJ444P:
    return AV_PIX_FMT_YUV444P;
  case AV_PIX_FMT_YUVJ440P:
    return AV_PIX_FMT_YUV440P;
  default:
    return pixFormat;
  }
}

AVPixelFormat Decoder::getValidPixelFormat(AVPixelFormat pixFormat) {
  AVPixelFormat validFormat = deprecatedImprove(pixFormat);

  // 检查是否为有效的像素格式
  if (!av_pix_fmt_desc_get(validFormat)) {
    // 打印错误信息或采取其他适当的错误处理措施
    av_log(NULL, AV_LOG_ERROR, "Invalid pixel format: %s\n",
           av_get_pix_fmt_name(pixFormat));
    // 返回默认的有效格式或其他适当的处理
    validFormat = AV_PIX_FMT_NONE;
  }

  return validFormat;
}

bool Decoder::preDecode(uint8_t *data, size_t length) {
  avPkt->size = length;
  avPkt->data = data;
  auto response = avcodec_send_packet(codecCtx, avPkt);
  if (response < 0) {
    printf("send data package to codec failed \n");
    return false;
  }
  response = avcodec_receive_frame(codecCtx, originFrame);
  if (response < 0) {
    printf("recv origin frame from codec failed \n");
    return false;
  }
  if (!setDecoder(originFrame->width, originFrame->height,
                  (AVPixelFormat)originFrame->format)) {
    return false;
  }
  printf("set decoder success!\n");
  return true;
}

bool Decoder::setDecoder(int width, int height, AVPixelFormat originFormat) {

  av_image_alloc(BGRFrame->data, BGRFrame->linesize, width, height,
                 AV_PIX_FMT_BGR24, 1);
  auto Format = deprecatedImprove(originFormat);
  Format = getValidPixelFormat(Format);
  colorSpcCvtCxt =
      sws_getContext(width, height, Format, width, height, AV_PIX_FMT_BGR24,
                     SWS_FAST_BILINEAR, NULL, NULL, NULL);
  return true;
}

bool Decoder::decode(cv::Mat &decodedMat, uint8_t *data, size_t length) {
  // auto t1 = std::chrono::steady_clock::now();
  av_packet_unref(avPkt);
  // fill data package
  avPkt->size = length;
  avPkt->data = data;

  // decode
  auto response = avcodec_send_packet(codecCtx, avPkt);
  if (response < 0) {
    printf("send data package to codec failed \n");
    return false;
  }
  response = avcodec_receive_frame(codecCtx, originFrame);
  if (response < 0 && response != 0) {
    printf("recv origin frame from codec failed :%d \n", response);
    return false;
  }
  sws_scale(colorSpcCvtCxt, originFrame->data, originFrame->linesize, 0,
            originFrame->height, BGRFrame->data, BGRFrame->linesize);

  decodedMat = cv::Mat(BGRFrame->height, BGRFrame->width, CV_8UC3,
                       BGRFrame->data[0]);
  return true;
}



cv::Mat Decoder::decode(uint8_t *data, size_t length) {
  // auto t1 = std::chrono::steady_clock::now();
  av_packet_unref(avPkt);
  // fill data package
  avPkt->size = length;
  avPkt->data = data;

  // decode
  auto response = avcodec_send_packet(codecCtx, avPkt);
  if (response < 0) {
    printf("send data package to codec failed \n");
  }
  response = avcodec_receive_frame(codecCtx, originFrame);
    if (response < 0 && response != 0) {
    printf("recv origin frame from codec failed :%d \n", response);
  }
  sws_scale(colorSpcCvtCxt, originFrame->data, originFrame->linesize, 0,
            originFrame->height, BGRFrame->data, BGRFrame->linesize);
  return cv::Mat(originFrame->height, originFrame->width, CV_8UC3,
                       BGRFrame->data[0]);

}
