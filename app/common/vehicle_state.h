/**
 * @Author: Xia Yunkai
 * @Date:   2024-02-10 20:21:42
 * @Last Modified by:   Xia Yunkai
 * @Last Modified time: 2024-02-10 20:25:59
 */
#include <stdint.h>

#ifndef __VEHICLE_STATE_H__
#define __VEHICLE_STATE_H__
#include "data_types.h"
namespace common
{
    struct VehicleState
    {
        float m_timeStamp = 0.0f;
        Vec2f m_position = Vec2f(0.0f, 0.0f);
        float m_heading = 0.0f;
        float m_curvature = 0.0f;
        float m_velocity = 0.0f;
        float m_acceleration = 0.0f;
        float m_steering = 0.0f;
        Vec3f ToXYYaw() const
        {
            return Vec3f(m_position.x, m_position.y, m_heading);
        }
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };

} // namespace common

#endif /* __VEHICLE_STATE_H__ */
