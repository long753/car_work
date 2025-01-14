#pragma once
#include <vector>
#include <cmath>
#include <string>
#include <opencv2/highgui.hpp> //OpenCV终端部署
#include <opencv2/opencv.hpp>  //OpenCV终端部署
#include "../vision/mapping.h"
using namespace std;

struct  mpoint
{
    int row = 0;
    int col = 0;
    float slope = 0.0f;
    mpoint()= default;
    mpoint(int row , int col): row(row),col(col){} 
    mpoint(cv::Point p ): row(p.y),col(p.x){} 
    template<typename T>
    mpoint operator+( T && m){
        return {m.row+row,m.col+col};
    }
    operator cv::Point(){return {col,row};}
};
int calAnglediff(int currentAngle, int previousAngle);
double average(vector<int> vec);
double sigma(vector<int> vec);
double sigma(vector<mpoint> vec);
int factorial(int x);
std::vector<mpoint> Bezier(double dt, vector<mpoint> input);
std::string formatDoble2String(double val, int fixed);
double distanceForPoint2Line(mpoint a, mpoint b, mpoint p);
double distanceForPoints(mpoint a, mpoint b);
int getMiddleValue(std::vector<int> vec);
void add_points_between_terminal_points(std::vector<mpoint> &liner_p, mpoint start_p, mpoint end_p);
void add_points_between_terminal_points(std::vector<cv::Point> &liner_p, mpoint start_p, mpoint end_p);
double neighbour_points_inner_product_cal(int index , std::vector<cv::Point> points);
double neighbour_points_angle(int index,std::vector<cv::Point> points);
double points_distance_IPM(cv::Point a, cv::Point b);
float calculateCorrelation(const vector<mpoint>& points);
float calculateCorrelation_ring(const vector<mpoint>& points);
float calculateCorrelation_ring_two(const vector<mpoint>& points);

