/**
 * @Author: Xia Yunkai
 * @Date:   2024-01-30 09:28:13
 * @Last Modified by:   Xia Yunkai
 * @Last Modified time: 2024-02-05 18:16:40
 */
#include <stdint.h>

#ifndef __GRID_MAP_H__
#define __GRID_MAP_H__
#include "common/data_types.h"
#include "common/rigid_transform2f.h"
#include <memory>
#include <mutex>
#include <iostream>
namespace map
{
    using namespace common;
    const int8_t OCC = 100;    // 栅格占据
    const int8_t FREE = 0;     // 栅格未被占据
    const int8_t UNKNOWM = -1; // 栅格状态未知

    class GridMap
    {
    public:
        using Ptr = std::shared_ptr<GridMap>;

        bool Init(const Rigid2f &map_origin, const Vec2i &map_size, const char *data, float resolution)
        {
            std::lock_guard<std::mutex> lock(m_DataMutex);
            if (m_initialized)
            {
                return true;
            }

            if (resolution <= 0)
            {
                return false;
            }

            m_resolution = resolution;
            m_resolution_inv = 1.0f / resolution;
            m_origin = map_origin;
            m_size = map_size;
            const int data_size = map_size.x() * map_size.y();
            m_data = new char[data_size];
            memcpy(m_data, data, data_size * sizeof(char));
            m_initialized = true;
            return true;
        }

        float GetResolution() const { return m_resolution; }

        float GetResolutionInv() const { return m_resolution_inv; }

        const Rigid2f &GetOrigin() const { return m_origin; }

        const Vec2i &GetSize() const { return m_size; }

        const char *GetData() const { return m_data; }

        const int GetDataSize() const { return m_size.x() * m_size.y(); }

        int GetWidth() const { return GetSize().x(); }

        int GetHeight() const { return GetSize().y(); }

        bool IsVerify(const Vec2i &pn) const
        {
            return pn.x() >= 0 && pn.x() < GetWidth() && pn.y() >= 0 && pn.y() < GetHeight();
        }

        bool Query(const Vec2i &pn) const
        {
            if (!IsVerify(pn))
            {
                return false;
            }
            return (!IsOccupied(pn));
        }

        int GetIndex(const Vec2i &pn) const
        {
            return pn.y() * GetWidth() + pn.x();
        }

        void SetOccupied(const Vec2i &pn)
        {
            int index = GetIndex(pn);
            SetOccupied(index);
        }

        void SetOccupied(size_t index)
        {
            std::lock_guard<std::mutex> lock(m_DataMutex);
            if (index >= GetWidth() * GetHeight())
            {
                return;
            }
            m_data[index] = OCC;
        }

        bool IsOccupied(const Vec2i &pn) const
        {
            int index = GetIndex(pn);
            return IsOccupied(index);
        }

        bool IsOccupied(const Vec2f &pos) const
        {
            return IsOccupied(FloatToInt(pos));
        }

        bool IsOccupied(size_t index) const
        {
            if (index >= GetWidth() * GetHeight())
            {
                return false;
            }
            return m_data[index] == OCC;
        }

        Vec2f IntToFloat(const Vec2i &pn) const
        {
            Vec2f pt = (pn.template cast<float>() + Vec2f::Constant(0.5)) * m_resolution;
            return m_origin * pt;
        }

        Vec2i FloatToInt(const Vec2f &pt) const
        {
            Vec2f point = m_origin.inverse() * pt;
            Vec2i pn;
            pn[0] = std::round((point[0]) * m_resolution_inv - 0.5);
            pn[1] = std::round((point[1]) * m_resolution_inv - 0.5);
            return pn;
        }

    protected:
        Rigid2f m_origin;
        Vec2i m_size;
        float m_resolution;
        float m_resolution_inv;
        char *m_data;
        std::mutex m_DataMutex;
        bool m_initialized = false;

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };

}

#endif /* __GRID_MAP_H__ */
