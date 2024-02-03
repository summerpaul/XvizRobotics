/**
 * @Author: Xia Yunkai
 * @Date:   2024-02-03 20:51:27
 * @Last Modified by:   Xia Yunkai
 * @Last Modified time: 2024-02-03 21:13:01
 */
#include <stdint.h>

#ifndef __POLYGON_COLLIDE_H__
#define __POLYGON_COLLIDE_H__

#include "data_types.h"
#include "calculations.h"

namespace common
{

    // 作用：判断点是否在多边形内

    bool PtInPolygon(const Vec2f &p, const Polygon2f &ptPolygon)
    {
        // 交点个数
        int nCross = 0;
        int nCount = ptPolygon.size();
        for (int i = 0; i < nCount; i++)
        {
            Vec2f p1 = ptPolygon[i];
            Vec2f p2 = ptPolygon[(i + 1) % nCount]; // 点P1与P2形成连线

            if (p1.y() == p2.y())
                continue;
            if (p.y() < Min(p1.y(), p2.y()))
                continue;
            if (p.y() >= Max(p1.y(), p2.y()))
                continue;
            // 求交点的x坐标（由直线两点式方程转化而来）

            float x = (p.y() - p1.y()) * (p2.x() - p1.x()) / (p2.y() - p1.y()) + p1.x();

            // 只统计p1p2与p向右射线的交点
            if (x > p.x())
            {
                nCross++;
            }
        }

        // 交点为偶数，点在多边形之外
        // 交点为奇数，点在多边形之内
        if ((nCross % 2) == 1)
        {
            return true;
        }
        else
        {

            return false;
        }
    }
} // namespace common

#endif /* __POLYGON_COLLIDE_H__ */
