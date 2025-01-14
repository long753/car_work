#include "IcarShow.h"


IcarShow::IcarShow()
{  // 显式初始化 mapp
    
}
void IcarShow::show_init(){
    int size =3 ;
    show_enable.store(true);
    cv::namedWindow("icar", WINDOW_NORMAL);                // 图像名称
    cv::resizeWindow("icar", COLSIMAGE * size, ROWSIMAGE); // 分辨率
    imgShow = cv::Mat::zeros(ROWSIMAGE, COLSIMAGE * size, CV_8UC3);
    enable = true;
    sizeWindow = size;
}

/**
 * @brief 设置新窗口属性
 *
 * @param index 窗口序号
 * @param img 显示图像
 * @param name 窗口名称
 */
void IcarShow::set_new_window(int index, Mat img, string name){
        // 数据溢出保护

        Mat imgDraw = img.clone();
        // 图像缩放
        if (imgDraw.cols != COLSIMAGE || imgDraw.rows != ROWSIMAGE)
        {
            float fx = COLSIMAGE / imgDraw.cols;
            float fy = ROWSIMAGE / imgDraw.rows;
            if (fx <= fy)
                resize(imgDraw, imgDraw, Size(COLSIMAGE, ROWSIMAGE), fx, fx);
            else
                resize(imgDraw, imgDraw, Size(COLSIMAGE, ROWSIMAGE), fy, fy);
        }
        // 限制图片标题长度
        string text = "[" + to_string(index + 1) + "] ";
        if (name.length() > 15)
            text = text + name.substr(0, 15);
        else
            text = text + name;

        putText(imgDraw, text, Point(10, 20), cv::FONT_HERSHEY_TRIPLEX, 0.5, cv::Scalar(0, 0, 255), 0.5, CV_AA);

        Rect placeImg = cvRect(COLSIMAGE * index, 0, COLSIMAGE, ROWSIMAGE);
        imgDraw.copyTo(imgShow(placeImg));
        
    }

/**
 * @brief 融合后的图像显示
 *
 */
void IcarShow::display()
{
    while(show_enable.load()){
        cv::Mat window1;     
        //debug_window1.try_pop(window1);
        {
            std::lock_guard<std::mutex> _lock(debug_image1_mutex);
            window1 = debug_image1.clone();
        }
        if(!window1.empty()){
            set_new_window(0,window1,now_scene);
            #define DISABLE_IPM
            #ifndef DISABLE_IPM
            cv::Mat window3;
            ipm.homography(window1, window3);
            set_new_window(2, window3, "IPM");
            #endif
        }
        #ifndef DISABLE_AI
        cv::Mat window2;    
        {
            std::lock_guard<std::mutex> _lock(debug_image2_mutex);
            window2 = debug_image2.clone();
        }

        if(!window2.empty()){
             set_new_window(1,window2,"AI");
        }
        #endif
        
        imshow("icar", imgShow);      
        cv::waitKey(10);

    }
        
}

void IcarShow::run(){
    debug_window_thread = std::thread(&IcarShow::display,this);
}


IcarShow::~IcarShow(){
    show_enable.store(false);
    if(debug_window_thread.joinable()){
        debug_window_thread.join();
    }
}

void IcarShow::ai_draw_result(Mat &img,std::vector<PredictResult> results)
{
    for (size_t i = 0; i < results.size(); i++)
    {
        PredictResult result = results[i];

        auto score = std::to_string(result.score);
        int pointY = result.y - 20;
        if (pointY < 0)
            pointY = 0;
        cv::Rect rectText(result.x, pointY, result.width, 20);
        cv::rectangle(img, rectText, getCvcolor(result.type), -1);
        std::string label_name = result.label + " [" + score.substr(0, score.find(".") + 3) + "]";
        cv::Rect rect(result.x, result.y, result.width, result.height);
        cv::rectangle(img, rect, getCvcolor(result.type), 1);
        cv::putText(img, label_name, Point(result.x, result.y), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(0, 0, 254), 1);
    }
}

/**
 * @brief 获取Opencv颜色
 *
 * @param index 序号
 * @return cv::Scalar
 */
cv::Scalar IcarShow::getCvcolor(int index)
{
    switch (index)
    {
    case 0:
        return cv::Scalar(0, 255, 0); // 绿
        break;
    case 1:
        return cv::Scalar(255, 255, 0); // 天空蓝
        break;
    case 2:
        return cv::Scalar(0, 0, 255); // 大红
        break;
    case 3:
        return cv::Scalar(0, 250, 250); // 大黄
        break;
    case 4:
        return cv::Scalar(250, 0, 250); // 粉色
        break;
    case 5:
        return cv::Scalar(0, 102, 255); // 橙黄
        break;
    case 6:
        return cv::Scalar(255, 0, 0); // 深蓝
        break;
    case 7:
        return cv::Scalar(255, 255, 255); // 大白
        break;
    case 8:
        return cv::Scalar(247, 43, 113);
        break;
    case 9:
        return cv::Scalar(40, 241, 245);
        break;
    case 10:
        return cv::Scalar(237, 226, 19);
        break;
    case 11:
        return cv::Scalar(245, 117, 233);
        break;
    case 12:
        return cv::Scalar(55, 13, 19);
        break;
    case 13:
        return cv::Scalar(255, 255, 255);
        break;
    case 14:
        return cv::Scalar(237, 226, 19);
        break;
    case 15:
        return cv::Scalar(0, 255, 0);
        break;
    default:
        return cv::Scalar(255, 0, 0);
        break;
    }
}

