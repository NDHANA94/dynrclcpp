/* -----------------------------------------------------------------------------
  Copyright 2024 Nipun Dhananjaya Weerakkodi

  Licensed under the Apache License, Version 2.0 (the "License");
  you may not use this file except in compliance with the License.
  You may obtain a copy of the License at

      http://www.apache.org/licenses/LICENSE-2.0

  Unless required by applicable law or agreed to in writing, software
  distributed under the License is distributed on an "AS IS" BASIS,
  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  See the License for the specific language governing permissions and
  limitations under the License.
-----------------------------------------------------------------------------*/ 

#ifndef _DYN_TIMER_BASE_HPP__
#define _DYN_TIMER_BASE_HPP__

#include <thread>
#include <atomic>
#include <chrono>
#include <functional>


namespace dynrclcpp{

class Timer : public std::enable_shared_from_this<Timer>{
public:
    template <typename Rep, typename Period>
    static std::shared_ptr<Timer> 
    create(std::function<void()> callback,  
    const std::chrono::duration<Rep, Period>& duration){
        return std::shared_ptr<Timer>(new Timer(std::move(callback), duration));
    }

    ~Timer();

    void stop();

private:
    template <typename Rep, typename Period>
    Timer(std::function<void()> callback, const std::chrono::duration<Rep, Period>& duration){
        callback_ = std::move(callback);
        timerThread_ = std::thread(&Timer::timerFunction<Rep, Period>, this, duration);
    }

    template <typename Rep, typename Period>
    void timerFunction(const std::chrono::duration<Rep, Period>& duration)
    {
        stopFlag_ = false;
        while(true){
            std::this_thread::sleep_for(duration);
            if(!stopFlag_) callback_();
            else break;
        }        
    }

    std::function<void()> callback_;
    std::thread timerThread_;
    std::atomic<bool> stopFlag_{false};

};
    


} //dynrclcpp

#endif //_DYN_TIMER_BASE_HPP__