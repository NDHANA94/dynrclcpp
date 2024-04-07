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

#include <thread>
#include <atomic>
#include <functional>
#include <memory>

#include "dynrclcpp/timer_base.hpp"

namespace dynrclcpp{

Timer::~Timer(){
    stopFlag_ = true;
    if(timerThread_.joinable()){
        timerThread_.join();
    }
}

void Timer::stop(){
    stopFlag_ = true;
}

} //dynrclcpp