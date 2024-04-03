#ifndef _DYN_TIMER_HPP__
#define _DYN_TIMER_HPP__

#include <thread>
#include <atomic>
#include <chrono>
#include <functional>



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
    




#endif //_DYN_TIMER_HPP__