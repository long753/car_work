#pragma once
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include <chrono>

extern "C"{
#include <libavcodec/avcodec.h>
#include <libavformat/avformat.h>
#include <libavfilter/avfilter.h>
#include <libswscale/swscale.h>
#include <libavutil/imgutils.h>
}
#include <memory>

class Decoder
{
public:
AVPacket *avPkt;
const AVCodec *mjpegCodec =  nullptr;
AVCodecContext *codecCtx = nullptr;
AVFrame *originFrame = nullptr;
AVFrame  *BGRFrame = nullptr;
SwsContext *colorSpcCvtCxt = nullptr;

Decoder();
~Decoder();
AVPixelFormat deprecatedImprove(AVPixelFormat pixFormat);

AVPixelFormat getValidPixelFormat(AVPixelFormat pixFormat);

bool preDecode(uint8_t *data, size_t length);

bool setDecoder(int width, int height, AVPixelFormat originFormat);

bool decode(cv::Mat &decodedMat, uint8_t *data, size_t length);
cv::Mat decode(uint8_t *data, size_t length); // 多线程使用

};


