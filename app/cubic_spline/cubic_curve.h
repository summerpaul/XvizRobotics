/**
 * @Author: Xia Yunkai
 * @Date:   2024-01-29 21:44:16
 * @Last Modified by:   Xia Yunkai
 * @Last Modified time: 2024-01-29 21:59:45
 */
#include <algorithm>
#include <stdint.h>
#include <vector>

#ifndef __CUBIC_CURVE_H__
#define __CUBIC_CURVE_H__
#include "common/data_types.h"
namespace cubic_spline {
using namespace common;
/**
 * @brief 一次样条曲线
 *
 */
class CubicPolynomial {
public:
  CubicPolynomial() = default;
  ~CubicPolynomial() = default;
  CubicPolynomial(const CubicPolynomial &) = default;
  CubicPolynomial &operator=(const CubicPolynomial &) = default;
  CubicPolynomial(CubicPolynomial &&) = default;
  CubicPolynomial &operator=(CubicPolynomial &&) = default;
  CubicPolynomial(float dur, const Matf<2, 4> &cMat)
      : m_duration(dur), m_coeffMat(cMat) {}

  inline int GetDim() const { return 2; }
  inline int GetDegree() const { return 3; }
  inline float GetDuration() const { return m_duration; }
  inline const Matf<2, 4> &GetCoeffMat() const { return m_coeffMat; }
  inline Matf<2, 4> &GetCoeffMat() { return m_coeffMat; }
  // s = a + b*t + c*t^2 + d * t^3
  inline Vec2f GetPos(float t) const {

    return m_coeffMat.col(3) +
           t * (m_coeffMat.col(2) +
                t * (m_coeffMat.col(1) + t * m_coeffMat.col(0)));
  }
  // v = b + 2*c*t + 3*d*t^2
  inline Vec2f GetVel(float t) const {
    return m_coeffMat.col(2) +
           t * (2 * m_coeffMat.col(1) + 3 * t * m_coeffMat.col(0));
  }
  // a = 2*c + 6 * d*t
  inline Vec2f GetAccel(float t) const {
    return 2 * m_coeffMat.col(1) + 6 * t * m_coeffMat.col(0);
  }

private:
  float m_duration;      // 样条曲线的时间
  Matf<2, 4> m_coeffMat; // 样条曲线系数
};

class CubicCurve {
private:
  typedef std::vector<CubicPolynomial> Pieces;
  Pieces m_pieces;

public:
  CubicCurve() = default;
  CubicCurve(const std::vector<float> &durs,
             const std::vector<Matf<2, 4>> &coeffs) {
    const int N = std::min(durs.size(), coeffs.size());
    m_pieces.reserve(N);
    for (int i = 0; i < N; ++i) {
      m_pieces.emplace_back(durs[i], coeffs[i]);
    }
  }
  inline int GetPiecesNum() const { return m_pieces.size(); }
};
} // namespace cubic_spline

#endif /* __CUBIC_CURVE_H__ */
