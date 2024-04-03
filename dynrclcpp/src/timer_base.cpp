#include <thread>
#include <atomic>
#include <functional>
#include <memory>

#include "dynrclcpp/timer_base.hpp"


// namespace dynrclcpp{


std::shared_ptr<Timer> 
Timer::create(
std::function<void()> callback,  
const std::chrono::duration<long, 
std::ratio<1l, 1l>>&  duration)
{
    return std::shared_ptr<Timer>(new Timer(std::move(callback), duration));
    
}

template<typename Duration>
Timer::Timer(std::function<void()> callback, const Duration& duration){
    callback_ = std::move(callback);
    timerThread_ = std::thread(&Timer::timerFunction<Duration>, this, duration);
}

Timer::~Timer(){
    stopFlag_ = true;
    if(timerThread_.joinable()){
        timerThread_.join();
    }
}

void Timer::stop(){
    stopFlag_ = true;
}


template <typename Duration>
void Timer::timerFunction(const Duration& duration)
{
    stopFlag_ = false;
    while(!stopFlag_){
        std::this_thread::sleep_for(duration);
        callback_();
    }        
}

// }