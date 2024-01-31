/**
 * @Author: Xia Yunkai
 * @Date:   2024-01-31 20:10:47
 * @Last Modified by:   Xia Yunkai
 * @Last Modified time: 2024-01-31 22:04:53
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
    /**
     * @brief  三次多项式求根
     * a*t^3+b*t^2+c*t+d = 0
     * @param a 三次项系数
     * @param b 二次项系数
     * @param c 一次项系数
     * @param d 常数项系数
     * @return Vec_f
     */
    Vec_f Cubic(float a, float b, float c, float d)
    {
        Vec_f dts;

        float a2 = b / a;
        float a1 = c / a;
        float a0 = d / a;

        float Q = (3 * a1 - a2 * a2) / 9;
        float R = (9 * a1 * a2 - 27 * a0 - 2 * a2 * a2 * a2) / 54;
        float D = Q * Q * Q + R * R;

        if (D > 0)
        {
            float S = std::cbrt(R + sqrt(D));
            float T = std::cbrt(R - sqrt(D));

            dts.push_back(-a2 / 3 + (S + T));
            return dts;
        }
        else if (D == 0)
        {
            float S = std::cbrt(R);
            dts.push_back(-a2 / 3 + S + S);
            dts.push_back(-a2 / 3 - S);
            return dts;
        }
        else
        {
            float theta = acos(R / sqrt(-Q * Q * Q));
            dts.push_back(2 * sqrt(-Q) * cos(theta / 3) - a2 / 3);
            dts.push_back(2 * sqrt(-Q) * cos((theta + 2 * M_PI) / 3) - a2 / 3);
            dts.push_back(2 * sqrt(-Q) * cos((theta + 4 * M_PI) / 3) - a2 / 3);
            return dts;
        }
    }

    /// Quartic equation: \f$a*t^4+b*t^3+c*t^2+d*t+e = 0\f$

    /**
     * @brief   四次多项式求根
     *
     * @param a 四次项系数
     * @param b 三次项系数
     * @param c 二次项系数
     * @param d 一次项系数
     * @param e 常数项系数
     * @return Vec_f
     */
    inline Vec_f Quartic(float a, float b, float c,
                         float d, float e)
    {
        Vec_f dts;

        float a3 = b / a;
        float a2 = c / a;
        float a1 = d / a;
        float a0 = e / a;

        Vec_f ys = Cubic(1, -a2, a1 * a3 - 4 * a0, 4 * a2 * a0 - a1 * a1 - a3 * a3 * a0);
        float y1 = ys.front();
        float r = a3 * a3 / 4 - a2 + y1;

        if (r < 0)
        {
            return dts;
        }

        float R = sqrt(r);
        float D, E;
        if (R != 0)
        {
            D = sqrt(0.75 * a3 * a3 - R * R - 2 * a2 + 0.25 * (4 * a3 * a2 - 8 * a1 - a3 * a3 * a3) / R);
            E = sqrt(0.75 * a3 * a3 - R * R - 2 * a2 - 0.25 * (4 * a3 * a2 - 8 * a1 - a3 * a3 * a3) / R);
        }
        else
        {
            D = sqrt(0.75 * a3 * a3 - 2 * a2 + 2 * sqrt(y1 * y1 - 4 * a0));
            E = sqrt(0.75 * a3 * a3 - 2 * a2 - 2 * sqrt(y1 * y1 - 4 * a0));
        }

        if (!std::isnan(D))
        {
            dts.push_back(-a3 / 4 + R / 2 + D / 2);
            dts.push_back(-a3 / 4 + R / 2 - D / 2);
        }
        if (!std::isnan(E))
        {
            dts.push_back(-a3 / 4 - R / 2 + E / 2);
            dts.push_back(-a3 / 4 - R / 2 - E / 2);
        }

        return dts;
    }

    /**
     * @brief 四次多项式求根，
     * a b c 可
     * @param a
     * @param b
     * @param c
     * @param d
     * @param e
     * @return Vec_f
     */
    inline Vec_f Solve(float a, float b, float c,
                       float d, float e)
    {
        Vec_f ts;
        if (a != 0)
        {
            return Quartic(a, b, c, d, e);
        }
        else if (b != 0)
        {
            return Cubic(b, c, d, e);
        }

        else if (c != 0)
        {
            return Quad(c, d, e);
        }

        else if (d != 0)
        {
            ts.push_back(-e / d);
            return ts;
        }
        else
        {
            return ts;
        }
    }

    inline Vec_f Solve(float a, float b, float c,
                       float d, float e, float f)
    {
        Vec_f ts;
        if (a == 0)
        {
            return Solve(b, c, d, e, f);
        }

        else
        {
            Eigen::VectorXf coeff(6);
            coeff << f, e, d, c, b, a;
            Eigen::PolynomialSolver<float, 5> solver;
            solver.compute(coeff);

            const Eigen::PolynomialSolver<float, 5>::RootsType &r = solver.roots();
            Vec_f ts;
            // std::cout << coeff.transpose() << std::endl;
            for (int i = 0; i < r.rows(); ++i)
            {
                if (r[i].imag() == 0)
                {
                    // std::cout << r[i] << std::endl;
                    ts.push_back(r[i].real());
                }
            }

            return ts;
        }
    }

    inline Vec_f Solve(float a, float b, float c,
                       float d, float e, float f,
                       float g)
    {
        Vec_f ts;
        if (a == 0 && b == 0)
        {
            return Solve(c, d, e, f, g);
        }

        else
        {
            Eigen::VectorXf coeff(7);
            coeff << g, f, e, d, c, b, a;
            Eigen::PolynomialSolver<float, 6> solver;
            solver.compute(coeff);

            const Eigen::PolynomialSolver<float, 6>::RootsType &r = solver.roots();
            Vec_f ts;
            // std::cout << coeff.transpose() << std::endl;
            for (int i = 0; i < r.rows(); ++i)
            {
                if (r[i].imag() == 0)
                {
                    // std::cout << r[i] << std::endl;
                    ts.push_back(r[i].real());
                }
            }

            return ts;
        }
    }

} // namespace common

#endif /* __POLY_ROOTS_H__ */
