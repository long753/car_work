#ifndef INCLUDE_THREAD_UTILS_HPP
#define INCLUDE_THREAD_UTILS_HPP
#include <condition_variable>
#include <memory>
#include <mutex>
#include <queue>
#include <thread>
#include <future>
#include <atomic>
#include <queue>
#include <vector>
#include "SingletonHolder.hpp"

template <typename T> class ThreadSafeQueue {
private:
  struct Node {
    std::shared_ptr<T> data;
    std::shared_ptr<Node> next;
    
  };

  std::mutex head_mutex;
  std::mutex tail_mutex;
  std::condition_variable data_cond;
  std::shared_ptr<Node> head;
  std::shared_ptr<Node> tail;
  std::atomic<int> _size;
  std::shared_ptr<Node> get_tail() {
    std::lock_guard<std::mutex> tail_lock(tail_mutex);
    return tail;
  }

  std::shared_ptr<Node> pop_head() {
    std::shared_ptr<Node> old_head = std::move(head);
    head = std::move(old_head->next);
    _size.store(_size.load()-1);
    return old_head;
  }

  std::unique_lock<std::mutex> wait_for_data() {
    std::unique_lock<std::mutex> head_lock(head_mutex);
    data_cond.wait(head_lock, [&] { return head.get() != get_tail().get(); });
    return head_lock;
  }

  std::shared_ptr<Node> wait_pop_head() {
    std::unique_lock<std::mutex> head_lock(wait_for_data());
    return pop_head();
  }

  std::shared_ptr<Node> wait_pop_head(T &value) {
    std::unique_lock<std::mutex> head_lock(wait_for_data());
    value = std::move(*head->data);
    return pop_head();
  }

  std::shared_ptr<Node> try_pop_head() {
    std::lock_guard<std::mutex> head_lock(head_mutex);
    if (head.get() == get_tail().get()) {
      return std::shared_ptr<Node>();
    }
    return pop_head();
  }

  std::shared_ptr<Node> try_pop_head(T &value) {
    std::lock_guard<std::mutex> head_lock(head_mutex);
    if (head.get() == get_tail().get()) {
      return std::shared_ptr<Node>();
    }
    value = std::move(*head->data);
    return pop_head();
  }

public:
  ThreadSafeQueue() {
    head = std::shared_ptr<Node>(new Node);
    _size.store(0);
    tail = head;
  }

  ThreadSafeQueue(const ThreadSafeQueue &other) = delete;
  ThreadSafeQueue &operator=(const ThreadSafeQueue &other) = delete;

  std::shared_ptr<T> wait_and_pop()
    {
        std::shared_ptr<Node> const old_head = wait_pop_head();
        return old_head->data;
    }

    void wait_and_pop(T& value)  
    {
        std::shared_ptr<Node> const old_head = wait_pop_head(value);
    }


    std::shared_ptr<T> try_pop()
    {
        std::shared_ptr<Node> old_head = try_pop_head();
        return old_head ? old_head->data : std::shared_ptr<T>();
    }
    bool try_pop(T& value)
    {
        std::shared_ptr<Node> const old_head = try_pop_head(value);
        return old_head ? true: false;
    }

  void push(T new_value) {
    std::shared_ptr<T> new_data { std::make_shared<T>(std::move(new_value)) };
    std::shared_ptr<Node> p(new Node);
   { std::lock_guard<std::mutex> tail_lock(tail_mutex);
    tail->data = new_data;
    tail->next = std::move(p);
    tail = tail->next;
    _size.store(_size.load()+1);
   }
   data_cond.notify_one();
  }

  bool empty() {
    std::lock_guard<std::mutex> head_lock(head_mutex);
    return (head.get() == get_tail().get());
  }

  size_t size(){
    return _size.load();
  }
};

class ThreadPool: public SingletonHolder<ThreadPool>{

using Task = std::packaged_task<void()>;
public:
std::atomic<int> thread_num;
std::queue<Task> tasks;
std::vector<std::thread> threads;
std::atomic<bool>  stop_flag;
template<class _F ,class ...Args>
auto commit(_F && Fx , Args && ... Ax)->
std::future<decltype(std::forward<_F>(Fx)(std::forward<Args>(Ax)...))>
{
  using RetType = decltype(std::forward<_F>(Fx)(std::forward<Args>(Ax)...));
  if(stop_flag.load()){
    return std::future<RetType>{};
  }
    auto task = std::make_shared<std::packaged_task<RetType()>>(
      std::bind(std::forward<_F>(Fx),std::forward<Args>(Ax)...));
    std::future<RetType>ret = task->get_future();
    {
      std::lock_guard<std::mutex> cv_mt(_mutex);
      tasks.emplace([task]{(*task)();});
    }
    _cond.notify_all();
    return ret;
  
}
int thread_count(){
  return thread_num;
}

private:

friend ThreadPool& SingletonHolder<ThreadPool>::get_instance();

ThreadPool(unsigned int num = 2 ) : stop_flag(false){
  thread_num = num;
  start();
}

void start(){
for(int i = 0; i< thread_num ;i++){
  threads.emplace_back([this](){
    while(!this->stop_flag.load()){
      Task task;
      {
        std::unique_lock<std::mutex> cv_mt(_mutex);
        this->_cond.wait(cv_mt,[this](){
          return this->stop_flag.load() || !this->tasks.empty();
        });
        if(this->tasks.empty()){
          return; 
        }

        task = std::move(this->tasks.front());
        this->tasks.pop();

      }
      this->thread_num--;
      task();
      this->thread_num++;
    }
  });
}


}
void stop(){
  stop_flag.store(true);
  _cond.notify_all();
  for(auto & t : threads){
    if(t.joinable()){
      t.join();
    }
  }
}


std::mutex _mutex;
std::condition_variable _cond;


};


inline static auto &thread_pool = ThreadPool::get_instance();


#endif
