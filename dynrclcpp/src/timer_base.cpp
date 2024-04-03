#include <thread>
#include <atomic>
#include <functional>
#include <memory>

#include "dynrclcpp/timer_base.hpp"



Timer::~Timer(){
    stopFlag_ = true;
    if(timerThread_.joinable()){
        timerThread_.join();
    }
}

void Timer::stop(){
    stopFlag_ = true;
}
