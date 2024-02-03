/**
 * @Author: Xia Yunkai
 * @Date:   2024-01-31 20:04:49
 * @Last Modified by:   Xia Yunkai
 * @Last Modified time: 2024-02-03 21:15:23
 */

#ifndef __X_ROBOTICS_CALCULATIONS_H__
#define __X_ROBOTICS_CALCULATIONS_H__
#include "data_types.h"
namespace common
{
    inline float DegToRad(float deg) { return kPi * deg / 180.0f; }
    // 弧度转角度
    inline float RadToDeg(float rad) { return 180.0f * rad / kPi; }

    template <typename T>
    inline int Sign(const T val)
    {
        return (T(0) < val) - (val < T(0));
    }

    inline float NormalizeAngleDeg(float deg)
    {
        const float degPi = 180.0f;
        while (deg > degPi)
        {
            deg -= 2. * degPi;
        }
        while (deg < -degPi)
        {
            deg += 2. * degPi;
        }
        return deg;
    }

    float NormalizeAngleRad(float rad)
    {
        const float radPi = kPi;
        while (rad > radPi)
        {
            rad -= 2. * radPi;
        }
        while (rad < -radPi)
        {
            rad += 2. * radPi;
        }
        return rad;
    }

    inline float FastSigmoidf(float x) { return x / (1 + fabs(x)); }

    inline float Normf(float x, float y) { return sqrtf(powf(x, 2) + powf(y, 2)); }

    template <typename T>
    inline bool IsZero(T value)
    {
        if (std::fabs(value) <= kEPS)
        {
            return true;
        }
        else
        {
            return false;
        }
    }
    template <typename T>
    inline bool IsLarge(T value_1, T value_2)
    {
        if (value_1 > value_2 && fabs(value_1 - value_2) > kEPS)
        {
            return true;
        }
        else
        {
            return false;
        }
    }

    // 判断前一个double是否小于后一个double
    template <typename T>
    inline bool IsSmall(T value_1, T value_2)
    {
        if (value_1 < value_2 && fabs(value_1 - value_2) > kEPS)
        {
            return true;
        }
        else
        {
            return false;
        }
    }

    // 判断前一个double是否等于后一个double
    template <typename T>
    inline bool IsEqual(T value_1, T value_2)
    {
        if (std::fabs(value_1 - value_2) <= kEPS)
        {
            return true;
        }
        else
        {
            return false;
        }
    }
    template <typename T>
    inline T Max(T value_1, T value_2)
    {
        if (value_1 > value_2)
        {
            return value_1;
        }
        else
        {
            return value_2;
        }
    }
    template <typename T>
    inline T Min(T value_1, T value_2)
    {
        if (value_1 < value_2)
        {
            return value_1;
        }
        else
        {
            return value_2;
        }
    }

    template <typename T>
    T Clamp(const T value, const T min, const T max)
    {
        if (value > max)
        {
            return max;
        }
        if (value < min)
        {
            return min;
        }
        return value;
    }

} // namespace common

#endif /* __CALCULATIONS_H__ */
