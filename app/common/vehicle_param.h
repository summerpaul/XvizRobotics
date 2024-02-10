/**
 * @Author: Xia Yunkai
 * @Date:   2024-02-10 20:07:28
 * @Last Modified by:   Xia Yunkai
 * @Last Modified time: 2024-02-10 20:19:37
 */
#include <stdint.h>

#ifndef __VEHICLE_PARAM_H__
#define __VEHICLE_PARAM_H__

namespace common
{

    class VehicleParam
    {
    public:
        inline float Width() const { return m_width; }
        inline float Length() const { return m_length; }
        inline float WheelBase() const { return m_wheelBase; }
        inline float FrontSuspension() const { return m_frontSuspension; }
        inline float RearSuspension() const { return m_rearSuspension; }
        inline float MaxSteeringAngle() const { return m_maxSteeringAngle; }
        inline float MaxLongitudinalAcc() const { return m_maxLongitudinalAcc; }
        inline float MaxLateralAcc() const { return m_maxLateralAcc; }
        inline float DCR() const { return m_dCr; }

        inline void setWidth(float width) { m_width = width; }
        inline void setLength(float len) { m_length = len; }
        inline void setWheelBase(float base) { m_wheelBase = base; }
        inline void setFrontSuspension(float susp) { m_frontSuspension = susp; }
        inline void setRearSuspension(float susp) { m_rearSuspension = susp; }
        inline void setMaxSteeringAngle(float angle) { m_maxSteeringAngle = angle; }
        inline void setMaxLongitudinalAcc(float acc) { m_maxLongitudinalAcc = acc; }
        inline void setMaxLateralAcc(float acc) { m_maxLateralAcc = acc; }
        inline void setDCR(float dcr) { m_dCr = dcr; }

    private:
        float m_width = 1.90f;
        float m_length = 4.88f;
        float m_wheelBase = 2.85f;
        float m_frontSuspension = 0.93f;
        float m_rearSuspension = 1.10f;
        float m_maxSteeringAngle = 45.0f;

        float m_maxLongitudinalAcc = 2.0f;
        float m_maxLateralAcc = 2.0f;
        float m_dCr = 1.015f; // length between geometry center and rear axle
    };
} // namesp

#endif /* __VEHICLE_PARAM_H__ */
