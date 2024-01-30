/**
 * @Author: Xia Yunkai
 * @Date:   2024-01-30 21:36:19
 * @Last Modified by:   Xia Yunkai
 * @Last Modified time: 2024-01-30 21:42:19
 */
#include <stdint.h>

#ifndef __TIME_H__
#define __TIME_H__
#include <thread>
#include <chrono>
namespace common
{
    inline double GetTimeNow()
    {
        auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch());
        return double(0.001 * ms.count());
    }
    inline void SleepSecond(const float dt)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(int(dt * 1000)));
    }
} // namespace common
#endif /* __TIME_H__ */
