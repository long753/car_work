#pragma once
#include "opencv2/core.hpp"
#include "opencv2/imgproc.hpp"
#include <ctime>
#include <iostream>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>
#include "../utils/SingletonHolder.hpp"

using namespace cv;
using namespace std;
#define COLSIMAGE 320    // 图像的列数
#define ROWSIMAGE 240    // 图像的行数
#define COLSIMAGEIPM 320 // IPM图像的列数
#define ROWSIMAGEIPM 400 // IPM图像的行数
class Mapping : public SingletonHolder<Mapping> {
public:
  /**
   * @brief IPM默认Size初始化
   *
   * @param origSize 输入原始图像Size
   * @param dstSize 输出图像Size
   */
  Mapping();

  /**
   * @brief IPM初始化
   *
   * @param origSize 输入原始图像Size
   * @param dstSize 输出图像Size
   */
  Mapping(const cv::Size &origSize, const cv::Size &dstSize);
  //cv::Mat IPM_MATRIX = (cv::Mat_<double>(3,3)<< 1.237582,0.468356,-227.95903,-0.000000,-0.174215,61.370571,-0.000000,0.003661,-0.289688);
  /**
   * @brief 单应性反透视变换
   *
   * @param _inputImg 原始域图像
   * @param _dstImg 矫正域图像
   * @param _borderMode 矫正模式
   */
  void homographyInv(const Mat &_inputImg, Mat &_dstImg, int _borderMode);

  /**
   * @brief 单应性反透视变换
   *
   * @param _point 矫正域坐标
   * @return Point2d 原始域坐标
   */
  Point2d homographyInv(const Point2d &_point);

  /**
   * @brief 单应性反透视变换
   *
   * @param _point 矫正域坐标
   * @return Point3d 原始域坐标
   */
  Point3d homographyInv(const Point3d &_point);

  /**
   * @brief 单应性透视变换
   *
   * @param _point 原始域坐标
   * @return Point2d 矫正域坐标
   */
  Point2d homography(const Point2d &_point);

  /**
   * @brief 单应性透视变换
   *
   * @param _point 原始域坐标
   * @param _H 转换矩阵
   * @return Point2d 矫正域坐标
   */
  Point2d homography(const Point2d &_point, const Mat &_H);

  /**
   * @brief 单应性透视变换
   *
   * @param _point 原始域坐标
   * @return Point3d 矫正域坐标
   */
  Point3d homography(const Point3d &_point);

  /**
   * @brief 单应性透视变换
   *
   * @param _point 原始域坐标
   * @param _H 转换矩阵
   * @return Point3d
   */
  Point3d homography(const Point3d &_point, const cv::Mat &_H);
  /**
   * @brief 单应性透视变换
   *
   * @param _inputImg 原始域图像
   * @param _dstImg 矫正域图像
   */
  void homography(const Mat &_inputImg, Mat &_dstImg);

  cv::Mat getH() const;
  cv::Mat getHinv() const;
  void getPoints(vector<Point2f> &_origPts, vector<Point2f> &_ipmPts);

  /**
   * @brief 绘制掩膜外框
   *
   * @param _points
   * @param _img
   */
  void drawBorder(const std::vector<cv::Point2f> &_points, cv::Mat &_img) const;

private:
  // Sizes
  cv::Size m_origSize;
  cv::Size m_dstSize;

  // Points
  std::vector<cv::Point2f> m_origPoints;
  std::vector<cv::Point2f> m_dstPoints;

  // Homography
  cv::Mat m_H;
  cv::Mat m_H_inv;

  // Maps
  cv::Mat m_mapX, m_mapY;
  cv::Mat m_invMapX, m_invMapY;

  void createMaps();
};

inline static auto &ipm = Mapping::get_instance();