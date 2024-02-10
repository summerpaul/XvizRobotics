/**
 * @Author: Xia Yunkai
 * @Date:   2023-12-31 02:28:45
 * @Last Modified by:   Xia Yunkai
 * @Last Modified time: 2024-01-20 12:51:15
 */

#ifndef __DATA_TYPES_H__
#define __DATA_TYPES_H__

#include <vector>
#include <string>
#include <iostream>
#include <cmath>
// 对外开放的基本数据类型
namespace xviz
{

    const std::string TF_ROOT_NAME = "none";
    const std::string WORLD_FRAME_ID = "world";

    struct Header
    {
        double timeStamp = 0.0;
        unsigned int seq = 2;
        std::string frameId = WORLD_FRAME_ID;
    };

    /// @brief 位姿
    struct Pose
    {

        Pose() : x(0.0f), y(0.0f), yaw(0.0f) {}
        Pose(float x, float y, float yaw) : x(x), y(y), yaw(yaw) {}
        float x, y, yaw;
        Header header;
        unsigned int id = 0;
    };

    struct Vec2f
    {
        Vec2f() : x(0.0f), y(0.0f) {}
        Vec2f(float x, float y) : x(x), y(y) {}
        void SetZero() { x = 0.0f, y = 0.0f; }
        float x, y;
    };

    struct Vec3f
    {
        Vec3f() : x(0.0f), y(0.0f), z(0.0f) {}
        Vec3f(float x, float y, float z) : x(x), y(y), z(z) {}
        void SetZero() { x = 0.0f, y = 0.0f, z = 0.0f; }
        float x, y, z;
    };

    struct Vec2i
    {
        Vec2i() : x(0), y(0) {}
        Vec2i(int x, int y) : x(x), y(y) {}
        void SetZero() { x = 0, y = 0; }
        int x, y;
    };

    struct PointXYZ
    {
        PointXYZ() : x(0.0f), y(0.0f), z(0.0f) {}
        PointXYZ(float x, float y, float z) : x(x), y(y), z(z) {}
        float x, y, z;
    };

    enum ColorType
    {
        WHITE = 0,
        BLACK = 1,
        BLUE = 2,
        GREEN = 3,
        RED = 4,
        YELLOW = 5,
        CYAN = 6,
        MAGENTA = 7,
        GRAY = 8,
        PURPLE = 9,
        PINK = 10,
        LIGHT_BLUE = 11,
        LIME_GREEN = 12,
        SLATE_GRAY = 13,
        COLOR_COUNT
    };

    struct GridMap
    {
        Header header;
        std::string m_data; // 使用string存图片
        float m_res;
        Pose m_origin;
        Vec2i m_size;
        unsigned int m_channel;
        bool m_reserve;
        unsigned int m_id = 0;
        bool m_isTopView = false;
    };

    struct Bezier
    {
        Header header;
        Vec2f p0;
        Vec2f p1;
        Vec2f p2;
        Vec2f p3;
        unsigned int m_id = 0;
    };

    struct Circle
    {
        Header header;
        Vec2f center;
        float radius{};
        unsigned int m_id = 0;
    };

    enum class MarkerType
    {
        PATH = 0,
        POLYGON = 1,
        CIRCLE = 3,
        BEZIER = 4,
        POSE = 5,
        POINT_CLOUD = 6,
        NONE_TYPE
    };

    struct PointCloud3f
    {
        Header header;
        std::vector<PointXYZ> points;
        unsigned int m_id = 0;
    };

    struct PointCloud2f
    {
        Header header;
        std::vector<Vec2f> points;
        unsigned int m_id = 0;
    };

    struct Path2f
    {
        Header header;
        std::vector<Vec2f> points;
        unsigned int m_id = 0;
    };

    struct Path2fArray
    {
        Header header;
        std::vector<Path2f> pathArray;
        unsigned int m_id = 0;
    };
    struct Polygon2f
    {
        Header header;
        std::vector<Vec2f> points;
        unsigned int m_id = 0;
    };

    struct Polygon2fArray
    {
        Header header;
        std::vector<Polygon2f> polygonArray;
        unsigned int m_id = 0;
    };

    struct Marker
    {
        Header header;
        MarkerType type = MarkerType::NONE_TYPE;
        Path2f path;
        Polygon2f polygon;
        Circle circle;
        Bezier bezier;
        Pose pose;
        PointCloud3f pointCloud;
        int colorType{};
        float length{};
        float thickness{};
        float radius{};
        unsigned int m_id = 0;
    };

    struct MarkerArray
    {
        Header header;
        std::vector<Marker> markers;
        unsigned int m_id = 0;
    };

    struct Rot
    {
        Rot() = default;

        explicit Rot(float angle)
        {
            m_sin = sinf(angle);
            m_cos = cosf(angle);
        }

        void Set(float angle)
        {

            m_sin = sinf(angle);
            m_cos = cosf(angle);
        }

        void SetIdentity()
        {
            m_sin = 0.0f;
            m_cos = 1.0f;
        }

        [[nodiscard]] float GetAngle() const
        {
            return atan2f(m_sin, m_cos);
        }

        [[nodiscard]] Vec2f GetXAxis() const
        {
            return {m_cos, m_sin};
        }

        [[nodiscard]] Vec2f GetYAxis() const
        {
            return {-m_sin, m_cos};
        }

        float m_sin = 0.0f;
        float m_cos = 1.0f;
    };

    struct Transform
    {
        Transform() { SetIdentity(); }
        Transform(float x, float y, float angle) { Set(Vec2f(x, y), angle); }
        Transform(const Vec2f &position, const Rot &rotation) : m_trans(position), m_rot(rotation) {}

        void Print() const
        {
            std::cout << " position x is " << m_trans.x << " y is " << m_trans.y << " yaw is " << m_rot.GetAngle() << std::endl;
        }

        void SetIdentity()
        {
            m_trans.SetZero();
            m_rot.SetIdentity();
        }

        void Set(const Vec2f &position, float angle)
        {
            m_trans = position;
            m_rot.Set(angle);
        }

        [[nodiscard]] Transform Inv() const
        {
            float inv_angle = -m_rot.GetAngle();
            float inv_x = -m_trans.x * m_rot.m_cos - m_trans.y * m_rot.m_sin;
            float inv_y = m_trans.x * m_rot.m_sin - m_trans.y * m_rot.m_cos;
            return {inv_x, inv_y, inv_angle};
        }

        Vec2f m_trans;
        Rot m_rot;
    };

    struct TransformNode
    {
        Transform m_transform;
        std::string m_frameId;
        std::string m_parentFrameId = TF_ROOT_NAME;
    };

    const std::string MSG_PATH = "MSG_PATH";
    const std::string MSG_POSE = "MSG_POSE";
    const std::string MSG_POINT_CLOUD = "MSG_POINT_CLOUD";
    const std::string MSG_POLYGON = "MSG_POLYGON";
    const std::string MSG_POLYGON_ARRAY = "MSG_POLYGONS";
    const std::string MSG_CIRCLE = "MSG_CIRCLE";
    const std::string MSG_BEZIER = "MSG_BEZIER";
    const std::string MSG_MARKER_ARRAY = "MSG_MARKER_ARRAY";
    const std::string MSG_MARKER = "MSG_MARKER";
    const std::string MSG_FLOAT_DATA = "MSG_FLOAT_DATA";
    const std::string MSG_STRING_DATA = "MSG_STRING_DATA";
    const std::string MSG_IMAGE = "MSG_IMAGE";
    const std::string MSG_TRANSFORM = "MSG_TRANSFORM";
    const std::string MSG_POINTS2F = "MSG_POINTS2F";
    const std::string MSG_PATH_ARRAY = "MSG_PATH_ARRAY";

} // namespace xviz

#endif /* __DATA_TYPES_H__ */
