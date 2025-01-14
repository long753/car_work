#pragma once 
#include "../include/common.hpp"

using namespace cv;
using namespace std;
class ImageProcess{
public:

ImageProcess();
Mat image_binary(Mat &frame);
void image_correcte_init(void);
Mat path_search(Mat &imageBinary); //输出完整赛道图像（可行使区域）     
Mat image_correct(const Mat &image);

private:
bool correctionEnable = false; // 图像矫正使能：初始化完成
cv::Mat cameraMatrix;			   // 摄像机内参矩阵
cv::Mat distCoeffs;				   // 相机的畸变矩阵
cv::Size sizeImage; // 图像的尺寸
cv::Mat mapx;
cv::Mat mapy;
cv::Mat rotMatrix;

};