#include "AiProcess.h"

AiProcess::AiProcess(){

// for(int i = 0; i< detection_num ;i++){
//     auto _d = std::make_shared<Detection>(CarParams->model);
//     _d->score = CarParams->score;
//     Detections.emplace_back(_d);
// }
// this->thread_free_nums = detection_num;

}

void AiProcess::ai_process_start(){
    // for(int i = 0; i< detection_num;i++){
    //     auto ai_task {
    //     [this](std::shared_ptr<Detection> detection){
    //         while(!this->stop_flag.load()){
    //         cv::Mat img;
    //         {
    //             std::unique_lock<std::mutex> lk(ai_mutex);
    //             ai_cond.wait(lk,[this](){
    //                 return this->stop_flag.load()
    //                 || !this->image_queue.empty();
    //             });
    //             if(this->image_queue.empty()){
    //                 return;
    //             }
    //             img= std::move(this->image_queue.front());
    //             this->image_queue.pop();
    //         }
    //         this->thread_free_nums--;
    //         detection->inference(img);
    //         this->thread_free_nums++;
    //         {
    //             std::scoped_lock _lk(result_mutex);
    //             this->results = detection->results;
    //         }

    //     auto draw_ai_task{
    //     [](cv::Mat m,std::vector<PredictResult> results){
    //       icarShow.ai_draw_result(m,results);
    //       icarShow.debug_window2.push(std::move(m));
    //     }
    //     };
    //     thread_pool.commit(draw_ai_task,std::move(img),detection->results);
    //     }     
    //     }
    // };
    //     this->threads.emplace_back(std::jthread(ai_task,this->Detections[i]));
    // }
}


void AiProcess::commit_ai_task(cv::Mat img){
    if(this->thread_free_nums !=0){
        if(ai_mutex.try_lock())
        {
            // std::scoped_lock _lk(ai_mutex);
            this->image_queue.emplace(img.clone());
            ai_mutex.unlock();
            this->ai_cond.notify_all();
        }
    }
}

AiProcess::~AiProcess(){
    this->stop_flag = true;
    this->ai_cond.notify_all();
}