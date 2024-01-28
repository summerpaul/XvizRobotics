/**
 * @Author: Xia Yunkai
 * @Date:   2024-01-28 19:30:04
 * @Last Modified by:   Xia Yunkai
 * @Last Modified time: 2024-01-28 20:49:21
 */
#include <stdint.h>

#ifndef __X_ROBOTICS_DATA_TYPES_H__
#define __X_ROBOTICS_DATA_TYPES_H__
#include <Eigen/Geometry>
#include <Eigen/StdVector>
namespace common {

// Eigen aliasing
template <int N> using Vecf = Eigen::Matrix<float, N, 1>;

template <int N> using Veci = Eigen::Matrix<int, N, 1>;

template <int M, int N> using Matf = Eigen::Matrix<float, M, N>;

template <int M, int N> using Mati8 = Eigen::Matrix<uint8_t, M, N>;

template <int M, int N> using Mati = Eigen::Matrix<int, M, N>;

template <int N> using MatNf = Matf<N, N>;

template <int N> using MatDNf = Eigen::Matrix<float, Eigen::Dynamic, N>;

using MatDf = Matf<Eigen::Dynamic, Eigen::Dynamic>;
using MatDi = Mati<Eigen::Dynamic, Eigen::Dynamic>;
using MatDi8 = Mati8<Eigen::Dynamic, Eigen::Dynamic>;

using Mat2f = Matf<2, 2>;
using Mat3f = Matf<3, 3>;
using Mat4f = Matf<4, 4>;

using Vec2f = Vecf<2>;
using Vec3f = Vecf<3>;
using Vec4f = Vecf<4>;

using Vec2i = Veci<2>;
using Vec3i = Veci<3>;
using Vec4i = Veci<4>;

// Workaround with STL container with eigen with fixed size eigen vector
template <typename T> using vec_E = std::vector<T, Eigen::aligned_allocator<T>>;

template <int N> using vec_Vecf = vec_E<Vecf<N>>;

using Path2f = vec_E<Vec2f>;
using Path3f = vec_E<Vec3f>;
using Points2f = vec_E<Vec2f>;
using Points3f = vec_E<Vec3f>;
using Polygon2f = vec_E<Vec2f>;
using Polygon3f = vec_E<Vec3f>;;

const float kBigEPS = 1e-1f;

const float kEPS = 1e-6f;

const float kSmallEPS = 1e-10f;

const float kPi = acosf(-1.0f);

const float kInf = 1e20f;



} // namespace common

#endif /* __DATA_TYPES_H__ */
