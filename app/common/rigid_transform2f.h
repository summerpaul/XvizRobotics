/**
 * @Author: Xia Yunkai
 * @Date:   2024-02-03 18:52:39
 * @Last Modified by:   Xia Yunkai
 * @Last Modified time: 2024-02-03 21:06:54
 */
#include <stdint.h>

#ifndef __RIGID_TRANSFORM_H__
#define __RIGID_TRANSFORM_H__

#include "data_types.h"
#include "calculations.h"
namespace common
{
    class Rigid2f
    {
    public:
        Rigid2f() : m_translation(Vec2f::Zero()), m_rotation(0.0f) {}
        Rigid2f(const Vec2f &translation, float angle) : m_translation(translation), m_rotation(angle) {}
        Rigid2f(const Vec2f &translation, const Rotation2f &rotation) : m_translation(translation), m_rotation(rotation) {}
        Rigid2f(float x, float y, float angle) : m_translation(x, y), m_rotation(angle) {}
        Rigid2f(const Rigid2f &other) : m_translation(other.m_translation), m_rotation(other.m_rotation) {}
        Rigid2f &operator=(const Rigid2f &other)
        {
            m_translation = other.m_translation;
            m_rotation = other.m_rotation;
            return *this;
        }

        static Rigid2f Rotation(float angle) { return Rigid2f(Vec2f::Zero(), Rotation2f(angle)); }

        static Rigid2f Rotation(const Rotation2f &rotation) { return Rigid2f(Vec2f::Zero(), rotation); }

        static Rigid2f Translation(const Vec2f &translation) { return Rigid2f(translation, Rotation2f::Identity()); }

        static Rigid2f Identity() { return Rigid2f(Vec2f::Zero(), Rotation2f::Identity()); }

        const Vec2f &translation() const { return m_translation; }

        Rotation2f rotation() const { return m_rotation; }

        Rigid2f inverse() const
        {
            const Rotation2f rotation = m_rotation.inverse();
            const Vec2f translation = -(rotation * m_translation);
            return Rigid2f(translation, rotation);
        }

    private:
        Vec2f m_translation;
        Rotation2f m_rotation;
    };

    Rigid2f operator*(const Rigid2f &lhs,
                      const Rigid2f &rhs)
    {
        return Rigid2f(
            lhs.rotation() * rhs.translation() + lhs.translation(),
            lhs.rotation() * rhs.rotation());
    }

    Vec2f operator*(const Rigid2f &rigid, const Vec2f &point)
    {
        return rigid.rotation() * point + rigid.translation();
    }
}

#endif /* __RIGID_TRANSFORM_H__ */
