/**
 * @Author: Xia Yunkai
 * @Date:   2024-02-10 22:49:06
 * @Last Modified by:   Xia Yunkai
 * @Last Modified time: 2024-02-10 23:59:32
 */
#include <stdint.h>

#ifndef __SHAPES_H__
#define __SHAPES_H__
#include "data_types.h"
namespace common
{
    struct OrientedBoundingBox2D
    {

        OrientedBoundingBox2D() {}
        OrientedBoundingBox2D(float x, float y, float heading, float width, float length) : m_x(x),
                                                                                            m_y(y),
                                                                                            m_heading(heading),
                                                                                            m_width(width),
                                                                                            m_length(length) {}
        float m_x;
        float m_y;
        float m_heading;
        float m_width;
        float m_length;
    };

    class ShapeUtils
    {
    public:
        inline static bool GetVerticesOfOrientedBoundingBox(const OrientedBoundingBox2D &obb, Points2f &vertices)
        {
            vertices.clear();
            vertices.resize(4);
            float cos_heading = cosf(obb.m_heading);
            float sin_heading = sinf(obb.m_heading);
            Vec2f corner1(
                obb.m_x + 0.5 * obb.m_length * cos_heading + 0.5 * obb.m_width * sin_heading,
                obb.m_y + 0.5 * obb.m_length * sin_heading - 0.5 * obb.m_width * cos_heading);
            Vec2f corner2(
                obb.m_x + 0.5 * obb.m_length * cos_heading - 0.5 * obb.m_width * sin_heading,
                obb.m_y + 0.5 * obb.m_length * sin_heading + 0.5 * obb.m_width * cos_heading);
            Vec2f corner3(
                obb.m_x - 0.5 * obb.m_length * cos_heading - 0.5 * obb.m_width * sin_heading,
                obb.m_y - 0.5 * obb.m_length * sin_heading + 0.5 * obb.m_width * cos_heading);
            Vec2f corner4(
                obb.m_x - 0.5 * obb.m_length * cos_heading + 0.5 * obb.m_width * sin_heading,
                obb.m_y - 0.5 * obb.m_length * sin_heading - 0.5 * obb.m_width * cos_heading);
            vertices[0] = corner1;
            vertices[1] = corner2;
            vertices[2] = corner3;
            vertices[3] = corner4;
            return true;
        }

        inline static bool GetDenseVerticesOfOrientedBoundingBox(const OrientedBoundingBox2D &obb, Points2f &vertices, float res)
        {
            vertices.clear();
            float cos_heading = cosf(obb.m_heading);
            float sin_heading = sinf(obb.m_heading);

            Vec2f corner1(
                obb.m_x + 0.5f * obb.m_length * cos_heading + 0.5f * obb.m_width * sin_heading,
                obb.m_y + 0.5f * obb.m_length * sin_heading - 0.5f * obb.m_width * cos_heading);
            Vec2f corner2(
                obb.m_x + 0.5 * obb.m_length * cos_heading - 0.5 * obb.m_width * sin_heading,
                obb.m_y + 0.5 * obb.m_length * sin_heading + 0.5 * obb.m_width * cos_heading);
            Vec2f corner3(
                obb.m_x - 0.5 * obb.m_length * cos_heading - 0.5 * obb.m_width * sin_heading,
                obb.m_y - 0.5 * obb.m_length * sin_heading + 0.5 * obb.m_width * cos_heading);
            Vec2f corner4(
                obb.m_x - 0.5 * obb.m_length * cos_heading + 0.5 * obb.m_width * sin_heading,
                obb.m_y - 0.5 * obb.m_length * sin_heading - 0.5 * obb.m_width * cos_heading);
            for (float dl = res; dl < (corner2 - corner1).norm(); dl += res)
            {
                Vec2f point12 = dl / (corner2 - corner1).norm() * (corner2 - corner1) + corner1;
                vertices.push_back(point12);
            }
            for (double dl = res; dl < (corner3 - corner2).norm(); dl += res)
            {
                Vec2f point23 = dl / (corner3 - corner2).norm() * (corner3 - corner2) + corner2;
                vertices.push_back(point23);
            }
            for (float dl = res; dl < (corner4 - corner3).norm(); dl += res)
            {
                Vec2f point34 = dl / (corner4 - corner3).norm() * (corner4 - corner3) + corner3;
                vertices.push_back(point34);
            }
            for (float dl = res; dl < (corner1 - corner4).norm(); dl += res)
            {
                Vec2f point41 = dl / (corner1 - corner4).norm() * (corner1 - corner4) + corner4;
                vertices.push_back(point41);
            }
            vertices.push_back(corner1);
            vertices.push_back(corner2);
            vertices.push_back(corner3);
            vertices.push_back(corner4);
            return true;
        }
    };

} // namespace common

#endif /* __SHAPES_H__ */
