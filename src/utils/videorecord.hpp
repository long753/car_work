#pragma once 
#include "../include/common.hpp"
#include "../include/detection.hpp"
#include <chrono>
#include "SingletonHolder.hpp"
#include <filesystem>
#include <string>

class VideoRecord:public SingletonHolder<VideoRecord>{

public:
    std::filesystem::path m_video_base = "../log/";
    int frame_fps = 30;
    int frame_width = 320;
    int frame_height = 240;
    cv::VideoWriter writer;
   
    VideoRecord(){
        auto now = std::chrono::system_clock::now();
        std::time_t now_c = std::chrono::system_clock::to_time_t(now);
        std::stringstream ss;
         ss <<std::put_time(std::localtime(&now_c), "%Y-%m-%d-%H-%M-%S");
    std::filesystem::path m_video;
    m_video += m_video_base;
    m_video+= ss.str();
    m_video+=".mp4";

        writer = cv::VideoWriter(m_video.c_str(), CV_FOURCC('a', 'v', 'c', '1'),
                         frame_fps, Size(frame_width, frame_height), true);
    }

    auto & operator<<(cv::Mat img){
        return (this->writer)<<img;
    }
   
    ~VideoRecord(){
        writer.release();
        cv::waitKey(10);
    }
};

inline static auto & video_record = VideoRecord::get_instance();