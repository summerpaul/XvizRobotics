/**
 * @Author: Xia Yunkai
 * @Date:   2024-01-31 20:04:49
 * @Last Modified by:   Xia Yunkai
 * @Last Modified time: 2024-01-31 20:10:23
 */

#ifndef __X_ROBOTICS_CALCULATIONS_H__
#define __X_ROBOTICS_CALCULATIONS_H__
#include "data_types.h"
namespace common
{
    template <typename T>
    inline int Sign(const T val)
    {
        return (T(0) < val) - (val < T(0));
    }


    float NormalizeAngle(float angle)
    {
        
    }



} // namespace common

#endif /* __CALCULATIONS_H__ */
