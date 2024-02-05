/**
 * @Author: Xia Yunkai
 * @Date:   2024-02-05 18:12:56
 * @Last Modified by:   Xia Yunkai
 * @Last Modified time: 2024-02-05 18:53:32
 */
#include <stdint.h>

#ifndef __ESDF_MAP_H__
#define __ESDF_MAP_H__
#include "grid_map.h"
namespace map
{
    using namespace common;

    class ESDFMap : public GridMap
    {
    public:
        typedef std::shared_ptr<ESDFMap> Ptr;

        void GetESDFMap(char *data)
        {
            if (data = nullptr)
            {
                return;
            }
            const float min_dist = 0.0;
            const float max_dist = 5.0;
            for (int i = 0; i < m_distBuffer.size(); i++)
            {
                const float dist = m_distBuffer[i];
                data[i] = (int)(dist - min_dist) / (max_dist - min_dist) * 255;
            }
        }

        void GenerateESDF2d()
        {
            if (!m_initialized)
            {
                std::cout << "map is not is_initialized" << std::endl;
                return;
            }
            const int data_size = m_size.x() * m_size.y();

            Vec_f grid_esdf_buffer(data_size, 0);
            Vec_f distance_buffer(data_size, 0);

            for (int x = 0; x < m_size[0]; x++)
            {
                FillESDF(
                    [&](int y)
                    {
                        return IsOccupied(Vec2i(x, y)) == 1
                                   ? 0
                                   : std::numeric_limits<float>::max();
                    },
                    [&](int y, float val)
                    {
                        grid_esdf_buffer[GetIndex(Vec2i(x, y))] = val;
                    },
                    0, m_size[1] - 1, 1);
            }
            for (int y = 0; y < m_size[1]; y++)
            {
                FillESDF([&](int x)
                         { return grid_esdf_buffer[GetIndex(Vec2i(x, y))]; },
                         [&](int x, float val)
                         {
                             distance_buffer[GetIndex(Vec2i(x, y))] = m_resolution * std::sqrt(val);
                         },
                         0, m_size[0] - 1, 0);
            }
            std::lock_guard<std::mutex> lock(m_distBufferMutex);
            m_distBuffer = distance_buffer;
            m_initialized = true;
        }

        float GetDistance(const Vec2i &pn)
        {
            if (!IsVerify(pn))
            {
                return 0;
            }
            return m_distBuffer[GetIndex(pn)];
        }

        float GetSDFValue(const Vec2f &pt)
        {
            /* use Bilinear interpolation */
            // 寻找周围四个点，求出中间点的距离

            // 已知Q11,Q12, Q21, Q22点处的dist值，求解中间点P的距离与梯度
            //    Q12----_Q22
            //     |      |
            //     |   P  |
            //     |      |
            //    Q11----_Q21

            // https://handwiki.org/wiki/Bilinear_interpolation

            // 近似到栅格坐标
            const Vec2i P_index = FloatToInt(pt);
            // 周围四个点的坐标
            const Vec2i Q11_index = P_index + Vec2i(-1, -1);
            const Vec2i Q21_index = P_index + Vec2i(1, -1);
            const Vec2i Q12_index = P_index + Vec2i(1, -1);
            const Vec2i Q22_index = P_index + Vec2i(1, 1);
            // 周围四个点的值
            const float f_Q11 = GetDistance(Q11_index);
            const float f_Q21 = GetDistance(Q21_index);
            const float f_Q12 = GetDistance(Q12_index);
            const float f_Q22 = GetDistance(Q22_index);

            // 转换成坐标进行插值
            const Vec2f Q11_pos = IntToFloat(Q11_index);
            const Vec2f Q22_pos = IntToFloat(Q22_index);

            const float x2_x1 = (Q22_pos - Q11_pos).x();
            const float y2_y1 = (Q22_pos - Q11_pos).y();
            const float x_x1 = (pt - Q11_pos).x();
            const float y_y1 = (pt - Q11_pos).y();
            const float x2_x = (Q22_pos - pt).x();
            const float y2_y = (Q22_pos - pt).y();

            const float f_R1 = x2_x / x2_x1 * f_Q11 + x_x1 / x2_x1 * f_Q21;
            const float f_R2 = x2_x / x2_x1 * f_Q12 + x_x1 / x2_x1 * f_Q22;
            const float f_P = y2_y / y2_y1 * f_R1 + y_y1 / y2_y1 * f_R2;
            return f_P;
        }

        void EvaluateEDTWithGrad(const Vec2f &pt, double &dist, Vec2f &grad)
        {
            // 已知Q11,Q12, Q21, Q22点处的dist值，求解中间点P的距离与梯度
            //    Q12----_Q22
            //     |      |
            //     |   P  |
            //     |      |
            //    Q11----_Q21
            const Vec2i P_index = FloatToInt(pt);
            // 周围四个点的坐标
            const Vec2i Q11_index = P_index + Vec2i(-1, -1);
            const Vec2i Q21_index = P_index + Vec2i(1, -1);
            const Vec2i Q12_index = P_index + Vec2i(1, -1);
            const Vec2i Q22_index = P_index + Vec2i(1, 1);
            if (!IsVerify(Q11_index) || !IsVerify(Q21_index) || !IsVerify(Q12_index) ||
                !IsVerify(Q22_index))
            {
                grad = Vec2f(0, 0);
                dist = GetDistance(P_index);
                return;
            }

            // 周围四个点的值
            const float f_Q11 = GetDistance(Q11_index);
            const float f_Q21 = GetDistance(Q21_index);
            const float f_Q12 = GetDistance(Q12_index);
            const float f_Q22 = GetDistance(Q22_index);

            // 转换成坐标进行插值
            const auto Q11_pos = IntToFloat(Q11_index);
            const auto Q21_pos = IntToFloat(Q21_index);
            const auto Q12_pos = IntToFloat(Q12_index);
            const auto Q22_pos = IntToFloat(Q22_index);

            const auto x2_x1 = (Q22_pos - Q11_pos).x();
            const auto y2_y1 = (Q22_pos - Q11_pos).y();
            const auto x_x1 = (pt - Q11_pos).x();
            const auto y_y1 = (pt - Q11_pos).y();
            const auto x2_x = (Q22_pos - pt).x();
            const auto y2_y = (Q22_pos - pt).y();

            const auto f_R1 = x2_x / x2_x1 * f_Q11 + x_x1 / x2_x1 * f_Q21;
            const auto f_R2 = x2_x / x2_x1 * f_Q12 + x_x1 / x2_x1 * f_Q22;
            const auto f_P = y2_y / y2_y1 * f_R1 + y_y1 / y2_y1 * f_R2;

            dist = f_P;
            const auto f_Q22_f_P = f_Q22 - f_P;
            const auto f_Q21_f_P = f_Q21 - f_P;
            const auto f_P_f_Q11 = f_P - f_Q11;
            const auto f_P_f_Q12 = f_P - f_Q12;
            const auto diff_Q22_P = Q22_pos - pt;
            const auto diff_Q21_P = Q21_pos - pt;
            const auto diff_P_Q12 = pt - Q12_pos;
            const auto diff_P_Q11 = pt - Q11_pos;

            grad.x() = f_Q22_f_P / diff_Q22_P.x() + f_Q21_f_P / diff_Q21_P.x() +
                       f_P_f_Q11 / diff_P_Q11.x() + f_P_f_Q12 / diff_P_Q12.x();
            grad.y() = f_Q22_f_P / diff_Q22_P.y() + f_Q21_f_P / diff_Q21_P.y() +
                       f_P_f_Q11 / diff_P_Q11.y() + f_P_f_Q12 / diff_P_Q12.y();
            grad = 0.25 * grad;
        }

    private:
        template <typename F_get_val, typename F_set_val>
        void FillESDF(F_get_val f_get_val, F_set_val f_set_val, int start, int end,
                      int dim)
        {
            int *v = new int[m_size(dim)];
            float *z = new float[m_size(dim) + 1];
            int k = start;
            v[start] = start;
            z[start] = -std::numeric_limits<float>::max();
            z[start + 1] = std::numeric_limits<float>::max();
            for (int q = start + 1; q <= end; q++)
            {
                k++;
                float s;
                do
                {
                    k--;
                    s = ((f_get_val(q) + q * q) - (f_get_val(v[k]) + v[k] * v[k])) /
                        (2 * q - 2 * v[k]);
                } while (s <= z[k]);
                k++;
                v[k] = q;
                z[k] = s;
                z[k + 1] = std::numeric_limits<float>::max();
            }
            k = start;
            for (int q = start; q <= end; q++)
            {
                while (z[k + 1] < q)
                    k++;
                float val = (q - v[k]) * (q - v[k]) + f_get_val(v[k]);
                f_set_val(q, val);
            }
            delete[] v;
            v = NULL;
            delete[] z;
            z = NULL;
        }

    protected:
        Vec_f m_distBuffer;
        bool m_initialized = false;
        std::mutex m_distBufferMutex;
    };

}
#endif /* __ESDF_MAP_H__ */
