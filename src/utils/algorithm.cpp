#include "algorithm.h"


float calculateCorrelation(const vector<mpoint>& points) {
    int n = points.size();
    if (n == 0) return 0.0;

    double sumX = 0.0, sumY = 0.0, sumXY = 0.0;
    double sumX2 = 0.0, sumY2 = 0.0;

    for (const auto& point : points) {
        double x = point.row;
        double y = point.col;

        sumX += x;
        sumY += y;
        sumXY += x * y;
        sumX2 += x * x;
        sumY2 += y * y;
    }

    double numerator = n * sumXY - sumX * sumY;
    double denominator = std::sqrt((n * sumX2 - sumX * sumX) * (n * sumY2 - sumY * sumY));

    return (denominator == 0) ? 0 : numerator / denominator;
}
float calculateCorrelation_ring(const vector<mpoint>& points) {
    int n = points.size();
    if (n == 0) return 0.0;
    int begin=0;
    double sumX = 0.0, sumY = 0.0, sumXY = 0.0;
    double sumX2 = 0.0, sumY2 = 0.0;
    for(int i=0;i<points.size();i++)
    {//cout<<"point["<<i<<"]:"<<points[i].col<<endl;
        if(points[i].col>6&&points[i].col<314)
        {begin=i;
            break;
        }
    }
    if(begin>130){
        return 0;
    }
    n=n-begin;
    cout<<"begin:"<<begin<<endl;
    for (int i=begin;i<points.size();i++) {
        double x = points[i].row;
        double y = points[i].col;
        if(x>50)
        {
        sumX += x;
        sumY += y;
        sumXY += x * y;
        sumX2 += x * x;
        sumY2 += y * y;
        }
        else{n=n-1;}
    }

    double numerator = n * sumXY - sumX * sumY;
    double denominator = std::sqrt((n * sumX2 - sumX * sumX) * (n * sumY2 - sumY * sumY));

    return (denominator == 0) ? 0 : numerator / denominator;
}
float calculateCorrelation_ring_two(const vector<mpoint>& points) {
    int n = points.size();
    if (n == 0) return 0.0;
    int begin=0;
    double sumX = 0.0, sumY = 0.0, sumXY = 0.0;
    double sumX2 = 0.0, sumY2 = 0.0;
    // for(int i=0;i<points.size();i++)
    // {//cout<<"point["<<i<<"]:"<<points[i].col<<endl;
    //     if(points[i].col>6&&points[i].col<314)
    //     {begin=i;
    //         break;
    //     }
    // }
    // if(begin>130){
    //     return 0;
    // }
    n=n-begin;
    //cout<<"begin:"<<begin<<endl;
    for (int i=begin;i<points.size();i++) {
        double x = points[i].row;
        double y = points[i].col;
        if(x>50)
        {
        sumX += x;
        sumY += y;
        sumXY += x * y;
        sumX2 += x * x;
        sumY2 += y * y;
        }
        else{n=n-1;}
    }

    double numerator = n * sumXY - sumX * sumY;
    double denominator = std::sqrt((n * sumX2 - sumX * sumX) * (n * sumY2 - sumY * sumY));

    return (denominator == 0) ? 0 : numerator / denominator;
}

int calAnglediff(int currentAngle, int previousAngle) {
    int diff = (currentAngle - previousAngle+360 ) % 360;
    if (diff > 180) {
        diff -= 360;
    }
    return abs(diff);
}
/**
 * 最小二乘 补线要从下往上补线
*/

void add_points_between_terminal_points(vector<mpoint> &liner_p, mpoint start_p, mpoint end_p)
{
    float k = (float)(end_p.row-start_p.row)/((float)(end_p.col-start_p.col)+0.001f);
    float b = end_p.row - k*(end_p.col);
    for(int i = start_p.row; i>= end_p.row;i--)
    {
        liner_p.emplace_back(i,(int)((i-b)/k));
    }
}

void add_points_between_terminal_points(vector<cv::Point> &liner_p, mpoint start_p, mpoint end_p)
{
    float k = (float)(end_p.row-start_p.row)/((float)(end_p.col-start_p.col)+0.001f);
    float b = end_p.row - k*(end_p.col);
    for(int i = start_p.row; i>= end_p.row;i--)
    {
        liner_p.emplace_back((int)((i-b)/k),i);
    }
}

/**
 * @brief 相邻点组成的向量的内积的cos值计算
 *
 * @param index  points 想要计算的点的索引 点集合
 * @return double
 */

double neighbour_points_inner_product_cal(int index , std::vector<cv::Point> points){
    int up_points_index = (index + 8 )< (points.size()-1) ? (index + 8 ) : (points.size()-1);
    int down_points_index = (index - 8 ) > 0 ? (index - 8 ) : 0;
    float inner_pro = ((points[up_points_index].x-points[index].x)*(points[down_points_index].x-points[index].x))+
    ((points[up_points_index].y-points[index].y)*(points[down_points_index].y-points[index].y));
    double m =sqrt(pow(points[up_points_index].x-points[index].x,2)+pow(points[up_points_index].y-points[index].y,2))*
    sqrt(pow(points[down_points_index].x-points[index].x,2)+pow(points[down_points_index].y-points[index].y,2));
    return inner_pro/m;
}
/** 
* @brief 相邻点组成的向量的角度计算
* @param index 当前点的索引
* @param points 点集合
* @return double 角度值 单位度
*/
double neighbour_points_angle(int index,std::vector<cv::Point> points){
    int up_points_index = (index + 10 )< (points.size()-1) ? (index + 10 ) : (points.size()-1);
    int down_points_index = (index - 10 ) > 0 ? (index - 10 ) : 0;

    cv::Point A = points[up_points_index];
    cv::Point B = points[index];
    cv::Point C = points[down_points_index];

    A = ipm.homography(A);
    B = ipm.homography(B);
    C = ipm.homography(C);

    double BA = sqrt((A.x-B.x)*(A.x-B.x)+(A.y-B.y)*(A.y-B.y));
    double BC = sqrt((C.x-B.x)*(C.x-B.x)+(C.y-B.y)*(C.y-B.y));

    double product_BA_BC = (A.x-B.x)*(C.x-B.x)+(A.y-B.y)*(C.y-B.y);
    return acos(product_BA_BC*1.00/(BA*BC))*57.3; //转化为度
}


double points_distance_IPM(cv::Point a, cv::Point b){
   cv::Point A = ipm.homography(a);
   cv::Point B = ipm.homography(b);

   return sqrt((A.x-B.x)*(A.x-B.x)+(A.y-B.y)*(A.y-B.y));
}
/**
 * @brief int集合平均值计算
 *
 * @param arr 输入数据集合
 * @return double
 */

double average(vector<int> vec)
{
    if (vec.size() < 1)
        return -1;

    double sum = 0;
    for (int i = 0; i < vec.size(); i++)
    {
        sum += vec[i];
    }

    return (double)sum / vec.size();
}

/**
 * @brief int集合数据方差计算
 *
 * @param vec Int集合
 * @return double
 */
double sigma(vector<int> vec)
{
    if (vec.size() < 1)
        return 0;

    double aver = average(vec); // 集合平均值
    double sigma = 0;
    for (int i = 0; i < vec.size(); i++)
    {
        sigma += (vec[i] - aver) * (vec[i] - aver);
    }
    sigma /= (double)vec.size();
    return sigma;
}

/**
 * @brief 赛道点集的方差计算
 *
 * @param vec
 * @return double
 */
double sigma(vector<mpoint> vec)
{
    if (vec.size() < 1)
        return 0;

    double sum = 0;
    for (int i = 0; i < vec.size(); i++)
    {
        sum += vec[i].col;
    }
    double aver = (double)sum / vec.size(); // 集合平均值

    double sigma = 0;
    for (int i = 0; i < vec.size(); i++)
    {
        sigma += (vec[i].col - aver) * (vec[i].col - aver);
    }
    sigma /= (double)vec.size();
    return sigma;
}

/**
 * @brief 阶乘计算
 *
 * @param x
 * @return int
 */
int factorial(int x)
{
    int f = 1;
    for (int i = 1; i <= x; i++)
    {
        f *= i;
    }
    return f;
}

/**
 * @brief 贝塞尔曲线
 *
 * @param dt
 * @param input
 * @return vector<POINT>
 */
vector<mpoint> Bezier(double dt, vector<mpoint> input)
{
    vector<mpoint> output;

    double t = 0;
    while (t <= 1)
    {
        mpoint p;
        double x_sum = 0.0;
        double y_sum = 0.0;
        int i = 0;
        int n = input.size() - 1;
        while (i <= n)
        {
            double k =
                factorial(n) / (factorial(i) * factorial(n - i)) * pow(t, i) * pow(1 - t, n - i);
            x_sum += k * input[i].row;
            y_sum += k * input[i].col;
            i++;
        }
        p.row = x_sum;
        p.col = y_sum;
        output.push_back(p);
        t += dt;
    }
    return output;
}


std::string formatDoble2String(double val, int fixed)
{
    auto str = std::to_string(val);
    return str.substr(0, str.find(".") + fixed + 1);
}


/**
 * @brief 点到直线的距离计算
 *
 * @param a 直线的起点
 * @param b 直线的终点
 * @param p 目标点
 * @return double
 */
double distanceForPoint2Line(mpoint a, mpoint b, mpoint p)
{
    int d = 0; // 距离

    double ab_distance =
        sqrt((a.row - b.row) * (a.row - b.row) + (a.col - b.col) * (a.col - b.col));
    double ap_distance =
        sqrt((a.row - p.row) * (a.row - p.row) + (a.col - p.col) * (a.col - p.col));
    double bp_distance =
        sqrt((p.row - b.row) * (p.row - b.row) + (p.col - b.col) * (p.col - b.col));

    double half = (ab_distance + ap_distance + bp_distance) / 2;
    double area = sqrt(half * (half - ab_distance) * (half - ap_distance) * (half - bp_distance));

    return (2 * area / ab_distance);
}

/**
 * @brief 两点之间的距离
 *
 * @param a
 * @param b
 * @return double
 */
double distanceForPoints(mpoint a, mpoint b)
{
    return sqrt((a.row - b.row) * (a.row - b.row) + (a.col - b.col) * (a.col - b.col));
}

int getMiddleValue(vector<int> vec) {
    if (vec.size() < 1)
      return -1;
    if (vec.size() == 1)
      return vec[0];

    int len = vec.size();
    while (len > 0) {
      bool sort = true; // 是否进行排序操作标志
      for (int i = 0; i < len - 1; ++i) {
        if (vec[i] > vec[i + 1]) {
          swap(vec[i], vec[i + 1]);
          sort = false;
        }
      }
      if (sort) // 排序完成
        break;

      --len;
    }

    return vec[(int)vec.size() / 2];
}

