/**
 * @Author: Xia Yunkai
 * @Date:   2024-01-09 11:18:01
 * @Last Modified by:   Xia Yunkai
 * @Last Modified time: 2024-02-04 20:22:44
 */
#include <stdint.h>

#ifndef __XVIZ_UTILS_H__
#define __XVIZ_UTILS_H__

#include "data_types.h"
#include "xviz_math.h"

namespace xviz
{

    /**
     * @brief 生成随机数字
     * @tparam T 数据类型
     * @param min 最小值
     * @param max 最大值
     * @return 随机数
     */
    template <typename T>
    inline T RandomRange(T min, T max)
    {
        T scale = rand() / (T)RAND_MAX;
        return min + scale * (max - min);
    }

    /**
     * @brief 根据坐标最大值最小值，生成矩形
     * @param pose_left_x x坐标最小值
     * @param pose_top_y y坐标最大值
     * @param pose_right_x x坐标最大值
     * @param pose_bottom_y y坐标最小值
     * @param rect 矩形
     */
    inline void GenRectangle(const float pose_left_x, const float pose_top_y, const float pose_right_x, const float pose_bottom_y, Polygon2f &rect)
    {
        rect.points.clear();
        rect.points.emplace_back(pose_left_x, pose_top_y);     // p0
        rect.points.emplace_back(pose_right_x, pose_top_y);    // p1
        rect.points.emplace_back(pose_right_x, pose_bottom_y); // p2
        rect.points.emplace_back(pose_left_x, pose_bottom_y);  // p2
        rect.points.emplace_back(pose_left_x, pose_top_y);
    }
    /**
     * @brief 根据最近点与最远点生成矩形
     * @param near_pt 最近点
     * @param far_pt 最远点
     * @param rect 矩形
     */
    inline void GenRectangle(const Vec2f &near_pt, const Vec2f &far_pt, Polygon2f &rect)
    {
        const float pose_left_x = near_pt.x;
        const float pose_bottom_y = near_pt.y;
        const float pose_right_x = far_pt.x;
        const float pose_top_y = far_pt.y;
        GenRectangle(pose_left_x, pose_top_y, pose_right_x, pose_bottom_y, rect);
    }
    /**
     * @brief 根据起点，矩形的长与宽生成矩形
     * @param pose 矩形左上角坐标
     * @param width 矩形的宽
     * @param height 矩形的长
     * @param rect 矩形
     */
    inline void GenRectangle(const Vec2f &pose, const float width, const float height, Polygon2f &rect)
    {
        const float pose_left_x = pose.x;
        const float pose_bottom_y = pose.y;
        const float pose_right_x = pose.x + width;
        const float pose_top_y = pose.y + height;
        GenRectangle(pose_left_x, pose_top_y, pose_right_x, pose_bottom_y, rect);
    }

    /**
     * @brief 生成路径
     * @tparam Path 路径的数据格式（数组）
     * @param path_in 输入路径
     * @param pt_num  路径点的数量
     * @param path_out 输出路径
     * @param ratio 转换比例，输出的路径单位为m
     */
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

    /**
     * @brief 位姿转坐标系
     *
     * @param pose 位姿
     * @return Transform 坐标系
     */

    inline Transform PoseToTransform(const Pose &pose)
    {
        return {pose.x, pose.y, pose.yaw};
    }
    /**
     * @brief 坐标系转位姿
     * @param tf 坐标系
     * @param frame_id 坐标系名
     * @return
     */
    inline Pose TransformToPose(const Transform &tf, const std::string &frame_id)
    {
        Pose pose;
        pose.x = tf.m_trans.x;
        pose.y = tf.m_trans.y;
        pose.yaw = tf.m_rot.GetAngle();
        pose.header.frameId = frame_id;
        return pose;
    }
    /**
     * @brief 生成转换后的圆
     * @param tf 转换坐标系
     * @param radius 坐标系原点
     * @param frame_id 坐标系名
     * @return
     */
    inline Circle GetTransformCircle(const Transform &tf, const float radius, const std::string &frame_id)
    {
        Circle circle;
        circle.center = Mul(tf, Vec2f(0, radius));
        circle.radius = std::fabs(radius);
        circle.header.frameId = frame_id;
        return circle;
    }
    /**
     * @brief 生成组合的车辆模型
     * @param steer_angle 方向盘转角 rad
     * @param wheel_angle 前轮转角 rad
     * @param width 车辆宽度 m
     * @param wb 车辆轴距 m
     * @param foh 车辆前悬 m
     * @param roh 车辆后悬 m
     * @param tyre_length 轮胎长度 m
     * @param tyre_width 车辆宽度 m
     * @param wheel_dist 车辆轮距 m
     * @param frameId 坐标系名
     * @return
     */
    inline void GenVehicleMarkers(float steer_angle, float wheel_angle, float width, float wb, float foh, float roh,
                                  float tyre_length, float tyre_width, float wheel_dist, const std::string &frameId, MarkerArray &markerArray)
    {

        markerArray.header.frameId = frameId;
        markerArray.markers.clear();
        Marker marker;
        marker.type = MarkerType::POLYGON;
        marker.thickness = 0.05;
        marker.colorType = ColorType::LIGHT_BLUE;
        // 创建车辆矩形
        GenRectangle(-roh, 0.5f * width, wb + foh, -0.5f * width, marker.polygon);
        markerArray.markers.emplace_back(marker);

        Polygon2f wheelBox;
        Rot wheel_rot(wheel_angle);
        // 获取车轮矩形
        GenRectangle(-0.5 * tyre_length, 0.5 * tyre_width, 0.5 * tyre_length, -0.5 * tyre_width, wheelBox);
        float half_wheel_dist = 0.5f * wheel_dist;
        // 添加左前轮
        marker.polygon.points = Mul(wheel_rot, wheelBox.points) + Vec2f(wb, half_wheel_dist);
        markerArray.markers.emplace_back(marker);
        // 添加右前轮
        marker.polygon.points = Mul(wheel_rot, wheelBox.points) + Vec2f(wb, -half_wheel_dist);
        markerArray.markers.emplace_back(marker);
        // 添加左后轮
        marker.polygon.points = wheelBox.points + Vec2f(0.0f, half_wheel_dist);
        markerArray.markers.emplace_back(marker);
        // 添加右后轮
        marker.polygon.points = wheelBox.points + Vec2f(0.0f, -half_wheel_dist);
        markerArray.markers.emplace_back(marker);
        marker.polygon.points.clear();
        marker.type = MarkerType::CIRCLE;
        marker.circle.center.x = wb;
        marker.circle.center.y = 0;
        marker.circle.radius = 0.5f;
        // 添加圆形方向盘
        markerArray.markers.emplace_back(marker);
        // 添加方向盘方向
        marker.type = MarkerType::POSE;
        marker.length = 0.5f;
        marker.pose.x = wb;
        marker.pose.y = 0.0f;
        marker.pose.yaw = steer_angle;
        markerArray.markers.emplace_back(marker);
        marker.length = 0.5f;
        marker.pose.x = 0.0f;
        marker.pose.y = 0.0f;
        marker.pose.yaw = 0.0f;
        markerArray.markers.emplace_back(marker);
    }

}
#endif /* __XVIZ_UTILS_H__ */
