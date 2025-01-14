#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <stdint.h>
#include "../src/SmartCar.hpp"
#include <iostream>
using namespace cv;

struct Ffl_old
{

    int16_t midline[240];
    int16_t leftline[240];
    int16_t rightline[240];
    int16_t leftlineflag[240];
    int16_t rightlineflag[240];

    int16_t leftstartpoint;
    int16_t rightstartpoint;
    int16_t endline;
    int8_t loseflag;
    float err[60];

    int16_t leftstartrow;
    int16_t rightstartrow;
    int16_t toprow;

    int16_t L_infec_row;
    int16_t R_infec_row;
    int16_t L_infec_flag;
    int16_t R_infec_flag;
    int16_t fork_toprow;
    int16_t fan_L_infec_row;
    int16_t fan_R_infec_row;
    int16_t fan_L_infec_flag;
    int16_t fan_R_infec_flag;
    int16_t fan_L_infec_col;
    int16_t fan_R_infec_col;

};
Ffl_old fl_old;
int lastError;
void caculate_err(void)
{
    int16_t i, sum;
    // static float sum;
    lastError = fl_old.err[0];  
    sum = 0;
    for (i = 59; i >= 1; i--)  
    fl_old.err[i] = fl_old.err[i - 1]; //偏差滤波
    fl_old.err[0] = 0.0;

    if (77 > fl_old.endline + 2) //<58
    {
        for (i = 75; i > fl_old.endline && i > 75 - 4; i--) // 摄像头前瞻修改位置， 通过修改i的范围来控制前瞻
        {
            sum++;
            fl_old.err[0] += (float)(fl_old.midline[i] - 87);//74
        }
        fl_old.err[0] = fl_old.err[0] / sum;
    }
    else
        fl_old.err[0] = (float)(fl_old.midline[fl_old.endline + 2] - 87);


    if (lastError * fl_old.err[0] < 0 && fabs(lastError - fl_old.err[0]) > 40)
    {

    }
    
}

void common_findline(cv::Mat &picture)
    {
        int16_t i = 0, j = 0;
        // 参数初始化
        fl_old.midline[239] = 160;
        fl_old.leftline[239] = 0;
        fl_old.rightline[239] = 319;
        fl_old.leftstartpoint = 0;
        fl_old.rightstartpoint = 0;

        for (i = 118; i > 0; i--)
        {
            fl_old.midline[i] = 160;
            fl_old.leftline[i] = 0;
            fl_old.rightline[i] = 319;
            fl_old.leftlineflag[i] = 0;
            fl_old.rightlineflag[i] = 0;

            fl_old.endline = 0;
            // 寻找左跳变点
            if (fl_old.midline[i + 1] > 80)
                j = fl_old.midline[i + 1] > 155 ? 155 : fl_old.midline[i + 1];
            else
                j = fl_old.midline[i + 1] < 4 ? 4 : fl_old.midline[i + 1]; // 先找到中间位置，从中间向左边开始找
            for (; j > 3; j--)
            {
                if (picture.at<uchar>(i, j) == 255 && 
                picture.at<uchar>(i, j - 1) == 255 && 
                picture.at<uchar>(i, j - 2) == 0 && 
                picture.at<uchar>(i, j - 3) == 0) // 黑黑白白
                {
                    if (fl_old.leftstartpoint == 0)
                        fl_old.leftstartpoint = i;
                    fl_old.leftline[i] = j - 1;
                    fl_old.leftlineflag[i] = 1;

                    break;
                }
            }
            if (fl_old.midline[i + 1] > 80)
                j = fl_old.midline[i + 1] > 155 ? 155 : fl_old.midline[i + 1];
            else
                j = fl_old.midline[i + 1] < 4 ? 4 : fl_old.midline[i + 1];
            for (j = fl_old.midline[i + 1]; j < 156; j++)
            {
                if (picture.at<uchar>(i, j) == 255 
                && picture.at<uchar>(i, j + 1) == 255 
                && picture.at<uchar>(i, j + 2) == 0 
                && picture.at<uchar>(i, j + 3) == 0) // 白白黑黑
                {
                    if (fl_old.rightstartpoint == 0)
                        fl_old.rightstartpoint = i;
                    fl_old.rightline[i] = j + 1;
                    fl_old.rightlineflag[i] = 1;
                    break;
                }
            }

            fl_old.midline[i] = (fl_old.leftline[i] + fl_old.rightline[i]) / 2;
            // 边线都没找到
            if (fl_old.leftlineflag[i] == 0 && fl_old.rightlineflag[i] == 0)
                fl_old.midline[i] = fl_old.midline[i + 1];
            fl_old.midline[i] = fl_old.midline[i] > 300 ? 300 : fl_old.midline[i];
            fl_old.midline[i] = fl_old.midline[i] < 0 ? 0 : fl_old.midline[i];
            // 找顶点
            if (fl_old.midline[i + 1] > 80)
                j = fl_old.midline[i + 1] > 155 ? 155 : fl_old.midline[i + 1];
            else
                j = fl_old.midline[i + 1] < 4 ? 4 : fl_old.midline[i + 1];
            if (picture.at<uchar>(i - 1, j) == 0 && (picture.at<uchar>(i - 1, j - 2) == 0 || picture.at<uchar>(i - 1, j + 2) == 0))
            {
                fl_old.endline = i;
                break;
            }
        }
for (size_t i = 0; i < 240; i++) {
        // std::cout<<fl_old.leftline[i] << " " << i<<std::endl;
}
}

void draw_image(Mat & img,int16_t left[] , int16_t right[],int16_t midline[]){
        for (size_t i = 0; i < 240; i++) {
            
      circle(img, Point(left[i], i), 3,
             Scalar(0, 255, 0), -1); // 绿色点
    }
    for (size_t i = 0; i < 240; i++) {
      circle(img, Point(right[i], i), 3,
             Scalar(0, 255, 255), -1); // 黄色点
    }

    for (size_t i = 0; i < 240; i++) {
      circle(img, Point(midline[i], i), 3,
             Scalar(0, 0, 255), -1); // 红色点
    }
}


int main(){
smartcar.camera_init();
while(1)
{
    if(!smartcar.frame_process()){
        continue;
    }
    cv::Mat imgb;
    common_findline(smartcar.imageBinary);
    cv::cvtColor(smartcar.imageBinary,imgb,CV_GRAY2BGR);
    draw_image(imgb ,fl_old.leftline,fl_old.rightline,fl_old.midline);
    imshow("222",imgb);
    cv::waitKey(30);
}

}
