/**
 * @Author: Xia Yunkai
 * @Date:   2024-01-31 20:10:47
 * @Last Modified by:   Xia Yunkai
 * @Last Modified time: 2024-01-31 20:23:30
 */
#include <stdint.h>

#ifndef __X_ROBOTICS_POLY_ROOTS_H__
#define __X_ROBOTICS_POLY_ROOTS_H__
#include <unsupported/Eigen/Polynomials>
#include "data_types.h"

// 多项式求根
namespace common
{
    /**
     * @brief 二次多项式
     *  b*t^2 + c * t + d = 0
     * @param b 二次项系数
     * @param c 一次项系数
     * @param d 常数项系数
     * @return Vec_f
     */
    inline Vec_f Quad(float b, float c, float d)
    {
        Vec_f dts;
        float p = c * c - 4 * b * d;
        if (p < 0)
        {
            return dts;
        }
        else
        {
            dts.push_back((-c - sqrt(p)) / (2 * b));
            dts.push_back((-c + sqrt(p)) / (2 * b));
            return dts;
        }
    }

} // namespace common

#endif /* __POLY_ROOTS_H__ */
