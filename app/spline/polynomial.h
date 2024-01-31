/**
 * @Author: Xia Yunkai
 * @Date:   2024-01-31 19:54:31
 * @Last Modified by:   Xia Yunkai
 * @Last Modified time: 2024-01-31 20:02:49
 */

#ifndef __X_ROBOTICS_POLYNOMIAL_H__
#define __X_ROBOTICS_POLYNOMIAL_H__
#include "common/data_types.h"
#include <assert.h>
using namespace common;
namespace spline
{
    template <int N_DEG>
    class Polynomial
    {
    public:
        typedef Vecf<N_DEG + 1> VecNf;
        enum
        {
            NeedsToAlign = (sizeof(VecNf) % 16) == 0
        };
        Polynomial() {}

    private:
        VecNf m_coeffs; // 多项式的系数
        VecNf m_coeffNormalOrder;
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW_IF(NeedsToAlign)
    };

} // namespace spline

#endif /* __POLYNOMIAL_H__ */
