#include "ImageProcess.h"

ImageProcess::ImageProcess() {
  sizeImage.width = 320;
  sizeImage.height = 240;

  mapx = Mat(sizeImage, CV_32FC1); // 经过矫正后的X坐标重映射参数
  mapy = Mat(sizeImage, CV_32FC1); // 经过矫正后的Y坐标重映射参数
  rotMatrix = Mat::eye(3, 3, CV_32F); // 内参矩阵与畸变矩阵之间的旋转矩阵
  if (correctionEnable) {
  image_correcte_init();
  }
}

Mat ImageProcess::image_binary(Mat &frame) {
  Mat imageGray, imageBinary;
  cvtColor(frame, imageGray, COLOR_BGR2GRAY);             // RGB转灰度图

  cv::Mat tmp;
  // Mat image_blur;
  // blur(imageGray,image_blur,Size(3,3));
  // Mat image_midblur;
  // medianBlur(image_blur,image_midblur,3);
  // double th1 = threshold(imageGray,tmp,0,255,THRESH_OTSU);
  // imageGray.setTo(th1,imageGray<=th1);

  threshold(imageGray, imageBinary, 0, 255, THRESH_OTSU); // OTSU二值化方法
  return imageBinary;
}

void ImageProcess::image_correcte_init() {
  cameraMatrix = Mat(3, 3, CV_32FC1, Scalar::all(0)); // 摄像机内参矩阵
  distCoeffs = Mat(1, 5, CV_32FC1, Scalar::all(0));   // 相机的畸变矩阵
  FileStorage file;
  if (file.open("../res/calibration/valid/calibration.xml",
                FileStorage::READ)) // 读取本地保存的标定文件
  {
    file["cameraMatrix"] >> cameraMatrix;
    file["distCoeffs"] >> distCoeffs;
    cout << "相机矫正参数初始化成功!" << endl;
    correctionEnable = true;
    initUndistortRectifyMap(cameraMatrix, distCoeffs, rotMatrix, cameraMatrix,
                            sizeImage, CV_32FC1, mapx, mapy);

  } else {
    cerr << "打开相机矫正参数失败!!!" << endl;
    correctionEnable = false;
  }
}

Mat ImageProcess::image_correct(const Mat &image) {
  if (correctionEnable) {
    Mat imageCorrect = image.clone();
    remap(image, imageCorrect, mapx, mapy, INTER_LINEAR);
    return imageCorrect;
  } else {
    return image;
  }
}

Mat ImageProcess::path_search(Mat &imageBinary) {
  Point pointFloodFill = cv::Point(60, 220); // 左:()  右：(60,220)
  Mat imagePath = Mat::zeros(imageBinary.size(), CV_8UC3);

  if (imageBinary.at<uchar>(220, 160) > 128) {
    pointFloodFill.x = 220;
    pointFloodFill.y = 160;
  } else if (imageBinary.at<uchar>(220, 60) > 128) {
    pointFloodFill.x = 220;
    pointFloodFill.y = 60;
  } else if (imageBinary.at<uchar>(220, 260) > 128) {
    pointFloodFill.x = 220;
    pointFloodFill.y = 260;
  }

  vector<vector<Point>> points; // 赛道轮廓搜索
  cv::findContours(imageBinary, points, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

  // 绘制轮廓

  int indexAreaMax = 0;
  double areaMax = 0;

  for (size_t i = 0; i < points.size(); i++) // 遍历
  {
    // 计算面积和周长
    //  double length = arcLength(points[i], true);
    double area = contourArea(points[i]);
    if (area > areaMax) {
      areaMax = area;
      indexAreaMax = i;
    }
  }
  cv::drawContours(imagePath, points, indexAreaMax, Scalar(238, 238, 175), 1);
  // imshow("imageContours", imagePath);

findMark:
  for (size_t i = 0; i < points[indexAreaMax].size(); i++) {
    if (points[indexAreaMax][i].x == pointFloodFill.y &&
        points[indexAreaMax][i].y == pointFloodFill.x) {
      cout << "-------------------------------Counter!" << endl;
      if (pointFloodFill.y < 160) {
        pointFloodFill.y += 5;
        pointFloodFill.x += 5;
        goto findMark;
      } else {
        pointFloodFill.y -= 5;
        pointFloodFill.x += 5;
        goto findMark;
      }

      break;
    }
  }
  cv::floodFill(imagePath, cv::Point(pointFloodFill.y, pointFloodFill.x),
                Scalar(0, 0, 255));
  cv::circle(imagePath, Point(pointFloodFill.y, pointFloodFill.x), 2,
             Scalar(100, 100, 100), -1); // 显示采样点
  return imagePath;
}
