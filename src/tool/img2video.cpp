/**
 * @file img2video.cpp
 * @author Leo
 * @brief 图像合成视频（mp4）
 * @version 0.1
 * @date 2023-02-21
 *
 * @copyright Copyright (c) 2023
 *
 */
#include <fstream>
#include <iostream>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;

int main()
{
    Mat img;
    VideoCapture vc;
    
    if (!vc.open(cv::CAP_V4L2)) {
        cerr << "Error: Could not open camera." << endl;
        return -1;
    }

    vc.set(cv::CAP_PROP_FPS, 2);
    vc.set(cv::CAP_PROP_FRAME_HEIGHT, 240);
    vc.set(cv::CAP_PROP_FRAME_WIDTH, 320);
    vc.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M','J','P','G'));

    bool capturing = false; 
    string img_name;
    int key = 0;
    int index = 6000;

    while (1){
        if (!vc.read(img)) {
            cerr << "Error: Could not read frame from camera." << endl;
            break;
        }

        if (img.empty()) {
            cerr << "Error: Captured image is empty." << endl;
            break;
        }

        imshow("pic", img);
        key = waitKey(0);
        std::cout << "key: " << key << endl;

        if (key == 0x20) { // 20 is space
            img_name = "../res/samples/img/safe" + to_string(index) + ".jpg";
            capturing = true;
        }

        if (capturing) {
            if (!imwrite(img_name, img)) {
                cout << "Error: Failed to write image " << img_name << endl;
            } else {
                cout << "Saved image: " << img_name << endl;
                std::cout << index << endl;
                index++;  // 更新图像序号
                capturing = false;
            }
        }
    }

    vc.release();
    return 0;
}
