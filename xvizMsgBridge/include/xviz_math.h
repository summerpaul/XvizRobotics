/**
 * @Author: Xia Yunkai
 * @Date:   2024-01-01 03:13:39
 * @Last Modified by:   Xia Yunkai
 * @Last Modified time: 2024-01-19 16:41:08
 */

#ifndef __XVIZ_MATH_H__
#define __XVIZ_MATH_H__

#include <math.h>
#include "data_types.h"
#include <iostream>
namespace xviz
{
    template <typename T>
    inline int Sign(T num)
    {
        if (num < 0)
            return -1;
        else if (num > 0)
            return 1;
        else
            return 0;
    }

    // 角度归一化 [-180, 180]
    template <typename T>
    T NormalizeAngleDeg(T deg)
    {
        const T degPi = T(180.0);
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
    // 弧度归一化 [-pi, pi]
    template <typename T>
    T NormalizeAngleRad(T rad)
    {
        const T radPi = T(M_PI);
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

    inline float Dot(const Vec2f &a, const Vec2f &b)
    {
        return a.x * b.x + a.y * b.y;
    }

    /// Perform the dot product on two vectors.
    inline float Dot(const Vec3f &a, const Vec3f &b)
    {
        return a.x * b.x + a.y * b.y + a.z * b.z;
    }

    /// Perform the cross product on two vectors. In 2D this produces a scalar.
    inline float Cross(const Vec2f &a, const Vec2f &b)
    {
        return a.x * b.y - a.y * b.x;
    }

    /// Perform the cross product on a vector and a scalar. In 2D this produces
    /// a vector.
    inline Vec2f Cross(const Vec2f &a, float s)
    {
        return Vec2f(s * a.y, -s * a.x);
    }

    /// Perform the cross product on a scalar and a vector. In 2D this produces
    /// a vector.
    inline Vec2f Cross(float s, const Vec2f &a)
    {
        return Vec2f(-s * a.y, s * a.x);
    }

    /// Perform the cross product on two vectors.
    inline Vec3f Cross(const Vec3f &a, const Vec3f &b)
    {
        return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x};
    }

    struct Mat22
    {

        Mat22() = default;

        Mat22(const Vec2f &c1, const Vec2f &c2)
        {
            ex = c1;
            ey = c2;
        }

        Mat22(float a11, float a12, float a21, float a22)
        {
            ex.x = a11;
            ex.y = a21;
            ey.x = a12;
            ey.y = a22;
        }
        void Set(const Vec2f &c1, const Vec2f &c2)
        {
            ex = c1;
            ey = c2;
        }

        void SetIdentity()
        {
            ex.x = 1.0f;
            ey.x = 0.0f;
            ex.y = 0.0f;
            ey.y = 1.0f;
        }

        void SetZero()
        {
            ex.x = 0.0f;
            ey.x = 0.0f;
            ex.y = 0.0f;
            ey.y = 0.0f;
        }

        [[nodiscard]] Mat22 GetInverse() const
        {
            float a = ex.x, b = ey.x, c = ex.y, d = ey.y;
            Mat22 B;
            float det = a * d - b * c;
            if (det != 0.0f)
            {
                det = 1.0f / det;
            }
            B.ex.x = det * d;
            B.ey.x = -det * b;
            B.ex.y = -det * c;
            B.ey.y = det * a;
            return B;
        }

        [[nodiscard]] Vec2f Solve(const Vec2f &b) const
        {
            float a11 = ex.x, a12 = ey.x, a21 = ex.y, a22 = ey.y;
            float det = a11 * a22 - a12 * a21;
            if (det != 0.0f)
            {
                det = 1.0f / det;
            }
            Vec2f x;
            x.x = det * (a22 * b.x - a12 * b.y);
            x.y = det * (a11 * b.y - a21 * b.x);
            return x;
        }

        Vec2f ex, ey;
    };

    struct Mat33
    {
        Mat33() = default;

        Mat33(const Vec3f &c1, const Vec3f &c2, const Vec3f &c3)
        {
            ex = c1;
            ey = c2;
            ez = c3;
        }

        void SetZero()
        {
            ex.SetZero();
            ey.SetZero();
            ez.SetZero();
        }

        [[nodiscard]] Vec3f Solve33(const Vec3f &b) const
        {
            float det = Dot(ex, Cross(ey, ez));
            if (det != 0.0f)
            {
                det = 1.0f / det;
            }
            Vec3f x;
            x.x = det * Dot(b, Cross(ey, ez));
            x.y = det * Dot(ex, Cross(b, ez));
            x.z = det * Dot(ex, Cross(ey, b));
            return x;
        }

        [[nodiscard]] Vec2f Solve22(const Vec2f &b) const
        {
            float a11 = ex.x, a12 = ey.x, a21 = ex.y, a22 = ey.y;
            float det = a11 * a22 - a12 * a21;
            if (det != 0.0f)
            {
                det = 1.0f / det;
            }
            Vec2f x;
            x.x = det * (a22 * b.x - a12 * b.y);
            x.y = det * (a11 * b.y - a21 * b.x);
            return x;
        }

        void GetInverse22(Mat33 *M) const
        {
            float a = ex.x, b = ey.x, c = ex.y, d = ey.y;
            float det = a * d - b * c;
            if (det != 0.0f)
            {
                det = 1.0f / det;
            }

            M->ex.x = det * d;
            M->ey.x = -det * b;
            M->ex.z = 0.0f;
            M->ex.y = -det * c;
            M->ey.y = det * a;
            M->ey.z = 0.0f;
            M->ez.x = 0.0f;
            M->ez.y = 0.0f;
            M->ez.z = 0.0f;
        }

        void GetSymInverse33(Mat33 *M) const
        {
            float det = Dot(ex, Cross(ey, ez));
            if (det != 0.0f)
            {
                det = 1.0f / det;
            }

            float a11 = ex.x, a12 = ey.x, a13 = ez.x;
            float a22 = ey.y, a23 = ez.y;
            float a33 = ez.z;

            M->ex.x = det * (a22 * a33 - a23 * a23);
            M->ex.y = det * (a13 * a23 - a12 * a33);
            M->ex.z = det * (a12 * a23 - a13 * a22);

            M->ey.x = M->ex.y;
            M->ey.y = det * (a11 * a33 - a13 * a13);
            M->ey.z = det * (a13 * a12 - a11 * a23);

            M->ez.x = M->ex.z;
            M->ez.y = M->ey.z;
            M->ez.z = det * (a11 * a22 - a12 * a12);
        }

        Vec3f ex, ey, ez;
    };

    /// Multiply a matrix times a vector. If a rotation matrix is provided,
    /// then this transforms the vector from one frame to another.
    inline Vec2f Mul(const Mat22 &A, const Vec2f &v)
    {
        return {A.ex.x * v.x + A.ey.x * v.y, A.ex.y * v.x + A.ey.y * v.y};
    }

    /// Multiply a matrix transpose times a vector. If a rotation matrix is provided,
    /// then this transforms the vector from one frame to another (inverse transform).
    inline Vec2f MulT(const Mat22 &A, const Vec2f &v)
    {
        return {Dot(v, A.ex), Dot(v, A.ey)};
    }

    /// Add two vectors component-wise.
    inline Vec2f operator+(const Vec2f &a, const Vec2f &b)
    {
        return {a.x + b.x, a.y + b.y};
    }

    /// Subtract two vectors component-wise.
    inline Vec2f operator-(const Vec2f &a, const Vec2f &b)
    {
        return {a.x - b.x, a.y - b.y};
    }

    // 批量加减
    inline std::vector<Vec2f> operator+(const std::vector<Vec2f> &as, const Vec2f &b)
    {
        std::vector<Vec2f> result;
        result.reserve(as.size());
        for (auto &a : as)
        {
            result.emplace_back(a + b);
        }
        return result;
    }

    inline std::vector<Vec2f> operator-(const std::vector<Vec2f> &as, const Vec2f &b)
    {
        std::vector<Vec2f> result;
        result.reserve(as.size());
        for (auto &a : as)
        {
            result.emplace_back(a - b);
        }
        return result;
    }

    inline Vec2f operator*(float s, const Vec2f &a)
    {
        return {s * a.x, s * a.y};
    }

    inline bool operator==(const Vec2f &a, const Vec2f &b)
    {
        return a.x == b.x && a.y == b.y;
    }

    inline bool operator!=(const Vec2f &a, const Vec2f &b)
    {
        return a.x != b.x || a.y != b.y;
    }

    inline float Distance(const Vec2f &a, const Vec2f &b)
    {
        Vec2f c = a - b;

        return sqrt(c.x * c.x + c.y * c.y);
    }

    inline float DistanceSquared(const Vec2f &a, const Vec2f &b)
    {
        Vec2f c = a - b;
        return Dot(c, c);
    }

    inline Vec3f operator*(float s, const Vec3f &a)
    {
        return {s * a.x, s * a.y, s * a.z};
    }

    /// Add two vectors component-wise.
    inline Vec3f operator+(const Vec3f &a, const Vec3f &b)
    {
        return {a.x + b.x, a.y + b.y, a.z + b.z};
    }

    /// Subtract two vectors component-wise.
    inline Vec3f operator-(const Vec3f &a, const Vec3f &b)
    {
        return {a.x - b.x, a.y - b.y, a.z - b.z};
    }

    inline Mat22 operator+(const Mat22 &A, const Mat22 &B)
    {
        return Mat22(A.ex + B.ex, A.ey + B.ey);
    }

    // A * B
    inline Mat22 Mul(const Mat22 &A, const Mat22 &B)
    {
        return Mat22(Mul(A, B.ex), Mul(A, B.ey));
    }

    // A^T * B
    inline Mat22 MulT(const Mat22 &A, const Mat22 &B)
    {
        Vec2f c1(Dot(A.ex, B.ex), Dot(A.ey, B.ex));
        Vec2f c2(Dot(A.ex, B.ey), Dot(A.ey, B.ey));
        return Mat22(c1, c2);
    }

    /// Multiply a matrix times a vector.
    inline Vec3f Mul(const Mat33 &A, const Vec3f &v)
    {
        return v.x * A.ex + v.y * A.ey + v.z * A.ez;
    }

    /// Multiply a matrix times a vector.
    inline Vec2f Mul22(const Mat33 &A, const Vec2f &v)
    {
        return {A.ex.x * v.x + A.ey.x * v.y, A.ex.y * v.x + A.ey.y * v.y};
    }

    /// Multiply two rotations: q * r
    inline Rot Mul(const Rot &q, const Rot &r)
    {
        // [qc -qs] * [rc -rs] = [qc*rc-qs*rs -qc*rs-qs*rc]
        // [qs  qc]   [rs  rc]   [qs*rc+qc*rs -qs*rs+qc*rc]
        // s = qs * rc + qc * rs
        // c = qc * rc - qs * rs
        Rot qr;
        qr.m_sin = q.m_sin * r.m_cos + q.m_cos * r.m_sin;
        qr.m_cos = q.m_cos * r.m_cos - q.m_sin * r.m_sin;
        return qr;
    }

    /// Transpose multiply two rotations: qT * r
    inline Rot MulT(const Rot &q, const Rot &r)
    {
        // [ qc qs] * [rc -rs] = [qc*rc+qs*rs -qc*rs+qs*rc]
        // [-qs qc]   [rs  rc]   [-qs*rc+qc*rs qs*rs+qc*rc]
        // s = qc * rs - qs * rc
        // c = qc * rc + qs * rs
        Rot qr;
        qr.m_sin = q.m_cos * r.m_sin - q.m_sin * r.m_cos;
        qr.m_cos = q.m_cos * r.m_cos + q.m_sin * r.m_sin;
        return qr;
    }

    /// Rotate a vector
    inline Vec2f Mul(const Rot &q, const Vec2f &v)
    {
        return {q.m_cos * v.x - q.m_sin * v.y, q.m_sin * v.x + q.m_cos * v.y};
    }

    inline std::vector<Vec2f> Mul(const Rot &q, const std::vector<Vec2f> &vs)
    {
        std::vector<Vec2f> result;
        result.reserve(vs.size());
        for (auto &v : vs)
        {
            result.emplace_back(Mul(q, v));
        }
        return result;
    }

    /// Inverse rotate a vector
    inline Vec2f MulT(const Rot &q, const Vec2f &v)
    {
        return {q.m_cos * v.x + q.m_sin * v.y, -q.m_sin * v.x + q.m_cos * v.y};
    }

    inline Vec2f Mul(const Transform &T, const Vec2f &v)
    {
        float x = (T.m_rot.m_cos * v.x - T.m_rot.m_sin * v.y) + T.m_trans.x;
        float y = (T.m_rot.m_sin * v.x + T.m_rot.m_cos * v.y) + T.m_trans.y;

        return {x, y};
    }

    inline Vec2f MulT(const Transform &T, const Vec2f &v)
    {
        float px = v.x - T.m_trans.x;
        float py = v.y - T.m_trans.y;
        float x = (T.m_rot.m_cos * px + T.m_rot.m_sin * py);
        float y = (-T.m_rot.m_sin * px + T.m_rot.m_cos * py);

        return {x, y};
    }

    inline std::vector<Vec2f> Mul(const Transform &T, const std::vector<Vec2f> &vs)
    {
        std::vector<Vec2f> result;
        result.reserve(vs.size());
        for (auto &v : vs)
        {
            result.emplace_back(Mul(T, v));
        }
        return result;
    }

    inline std::vector<Vec2f> MulT(const Transform &T, const std::vector<Vec2f> &vs)
    {
        std::vector<Vec2f> result;
        result.reserve(vs.size());
        for (auto &v : vs)
        {
            result.emplace_back(MulT(T, v));
        }
        return result;
    }

    // v2 = A.q.Rot(B.q.Rot(v1) + B.p) + A.p
    //    = (A.q * B.q).Rot(v1) + A.q.Rot(B.p) + A.p
    inline Transform Mul(const Transform &A, const Transform &B)
    {
        Transform C;
        C.m_rot = Mul(A.m_rot, B.m_rot);
        C.m_trans = Mul(A.m_rot, B.m_trans) + A.m_trans;
        return C;
    }

    // v2 = A.q' * (B.q * v1 + B.p - A.p)
    //    = A.q' * B.q * v1 + A.q' * (B.p - A.p)
    inline Transform MulT(const Transform &A, const Transform &B)
    {
        Transform C;
        C.m_rot = MulT(A.m_rot, B.m_rot);
        C.m_trans = MulT(A.m_rot, B.m_trans - A.m_trans);
        return C;
    }

    template <typename T>
    inline T Abs(T a)
    {
        return a > T(0) ? a : -a;
    }

    inline Vec2f Abs(const Vec2f &a)
    {
        return {Abs(a.x), Abs(a.y)};
    }

    inline Mat22 Abs(const Mat22 &A)
    {
        return Mat22(Abs(A.ex), Abs(A.ey));
    }

    template <typename T>
    inline T Min(T a, T b)
    {
        return a < b ? a : b;
    }

    inline Vec2f Min(const Vec2f &a, const Vec2f &b)
    {
        return {Min(a.x, b.x), Min(a.y, b.y)};
    }

    template <typename T>
    inline T Max(T a, T b)
    {
        return a > b ? a : b;
    }

    inline Vec2f Max(const Vec2f &a, const Vec2f &b)
    {
        return {Max(a.x, b.x), Max(a.y, b.y)};
    }

    template <typename T>
    inline T Clamp(T a, T low, T high)
    {
        return Max(low, Min(a, high));
    }

    inline Vec2f Clamp(const Vec2f &a, const Vec2f &low, const Vec2f &high)
    {
        return Max(low, Min(a, high));
    }

} // namespace xviz

#endif
