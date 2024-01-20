/**
 * @Author: Xia Yunkai
 * @Date:   2024-01-09 11:18:01
 * @Last Modified by:   Xia Yunkai
 * @Last Modified time: 2024-01-18 10:32:03
 */
#include <stdint.h>

#ifndef __XVIZ_UTILS_H__
#define __XVIZ_UTILS_H__

#include "data_types.h"
#include "xviz_math.h"

namespace xviz
{
    template <typename T>
    inline T RandomRange(T min, T max)
    {
        T scale = rand() / (T)RAND_MAX;
        return min + scale * (max - min);
    }

    inline void GenRectangle(const float pose_left_x, const float pose_top_y, const float pose_right_x, const float pose_bottom_y, Polygon2f &rect)
    {
        rect.points.clear();
        rect.points.emplace_back(pose_left_x, pose_top_y);     // p0
        rect.points.emplace_back(pose_right_x, pose_top_y);    // p1
        rect.points.emplace_back(pose_right_x, pose_bottom_y); // p2
        rect.points.emplace_back(pose_left_x, pose_bottom_y);  // p2
        rect.points.emplace_back(pose_left_x, pose_top_y);
    }

    inline void GenRectangle(const Vec2f &near_pt, const Vec2f &far_pt, Polygon2f &rect)
    {
        const float pose_left_x = near_pt.x;
        const float pose_bottom_y = near_pt.y;
        const float pose_right_x = far_pt.x;
        const float pose_top_y = far_pt.y;
        GenRectangle(pose_left_x, pose_top_y, pose_right_x, pose_bottom_y, rect);
    }

    inline void GenRectangle(const Vec2f &pose, const float width, const float height, Polygon2f &rect)
    {
        const float pose_left_x = pose.x;
        const float pose_bottom_y = pose.y;
        const float pose_right_x = pose.x + width;
        const float pose_top_y = pose.y + height;
        GenRectangle(pose_left_x, pose_top_y, pose_right_x, pose_bottom_y, rect);
    }

    template <typename Path>
    inline void GetPath(const Path &path_in, const int pt_num, Path2f &path_out, const float ratio = 1.0f)
    {
        path_out.points.clear();
        path_out.points.resize(pt_num);
        for (int i = 0; i < pt_num; i++)
        {
            const float x = path_in[i].x * ratio;
            const float y = path_in[i].y * ratio;
            path_out.points[i] = Vec2f(x, y);
        }
    }

    inline Transform PoseToTransform(const Pose &pose)
    {
        return {pose.x, pose.y, pose.yaw};
    }

    inline Pose TransformToPose(const Transform &tf, const std::string &frame_id)
    {
        Pose pose;
        pose.x = tf.m_trans.x;
        pose.y = tf.m_trans.y;
        pose.yaw = tf.m_rot.GetAngle();
        pose.header.frameId = frame_id;
        return pose;
    }

    inline Circle GetTransformCircle(const Transform &tf, const float radius, const std::string &frame_id)
    {
        Circle circle;
        circle.center = Mul(tf, Vec2f(0, radius));
        circle.radius = std::fabs(radius);
        circle.header.frameId = frame_id;
        return circle;
    }

}
#endif /* __XVIZ_UTILS_H__ */
