#ifndef _DYN_TIMER_HPP__
#define _DYN_TIMER_HPP__

#include <thread>
#include <atomic>
#include <chrono>
#include <functional>

// namespace dynrclcpp
// {


class Timer : public std::enable_shared_from_this<Timer>{
public:
    static std::shared_ptr<Timer> 
    create(std::function<void()> callback,  
    const std::chrono::duration<long, 
    std::ratio<1l, 1l>>& duration);

    ~Timer();

    void stop();

private:
    template <typename Duration>
    Timer(std::function<void()> callback, const Duration& duration); 

    template <typename Duration>
    void timerFunction(const Duration& duration);

    std::function<void()> callback_;
    std::thread timerThread_;
    std::atomic<bool> stopFlag_{false};

};
    
// } // namespace dynrclcpp



#endif //_DYN_TIMER_HPP__