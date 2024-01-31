/**
 * @Author: Xia Yunkai
 * @Date:   2024-01-31 20:04:49
 * @Last Modified by:   Xia Yunkai
 * @Last Modified time: 2024-01-31 21:39:55
 */

#ifndef __X_ROBOTICS_CALCULATIONS_H__
#define __X_ROBOTICS_CALCULATIONS_H__
#include "data_types.h"
namespace common
{
    constexpr float DegToRad(float deg) { return kPi * deg / 180.0f; }
    // 弧度转角度
    constexpr float RadToDeg(float rad) { return 180.0f * rad / kPi; }

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

    inline float FastSigmoid(float x) { return x / (1 + fabs(x)); }

    inline float Norm(float x, float y) { return sqrtf(powf(x, 2) + powf(y, 2)); }

    inline bool IsZero(float value)
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

    inline bool IsLarge(float value_1, float value_2)
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
    inline bool IsSmall(float value_1, float value_2)
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
    inline bool IsEqual(float value_1, float value_2)
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

    inline float Max(float value_1, float value_2)
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

    inline float Min(float value_1, float value_2)
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
    inline float Clamp(float value, float min, float max)
    {
        if (value < min)
        {
            return min;
        }
        else if (value > max)
        {
            return max;
        }
        else
        {
            return value;
        }
    }

} // namespace common

#endif /* __CALCULATIONS_H__ */
