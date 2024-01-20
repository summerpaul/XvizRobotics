/**
 * @Author: Xia Yunkai
 * @Date:   2023-12-29 09:41:07
 * @Last Modified by:   Xia Yunkai
 * @Last Modified time: 2024-01-20 19:04:21
 */
#include <iostream>

using namespace std;
#include "data_types.h" //发送的基本数据类型
#include "xvizMsgBridge.h"
#include "xviz_marcro.h" //宏定义
#include "xviz_math.h"   // 坐标转换相关函数
#include "xviz_utils.h"  //
#include <chrono>
#include <cmath>
#include <thread>

using namespace xviz;
// 发布的数据单位为m，float类型
class Demo
{
public:
    /// @brief 初始化程序
    /// @return
    bool Init()
    {
        // step 1 :创建XvizMsgBridge对象
        m_bridge = std::make_unique<xviz::XvizMsgBridge>();
        // step 2:初始化通信
        if (!m_bridge->Init("tcp://127.0.0.1:8888", "tcp://127.0.0.1:8899"))
        {
            return false;
        }
        // 绑定初始位姿回调函数
        m_bridge->SetInitPoseFunc(
            [this](auto && PH1) { InitPoseCallback(std::forward<decltype(PH1)>(PH1)); });
        // 绑定目标位姿回调函数
        m_bridge->SetTargetPoseFunc(
            [this](auto && PH1) { TarPoseCallback(std::forward<decltype(PH1)>(PH1)); });

        m_bridge->Run();

        return true;
    }
    /// @brief 发布全局坐标系下的路径demo
    void PubPathDemo(const std::string &name, const std::string &frameId)
    {
        xviz::Path2f path;
        path.header.frameId = frameId; // 确定绘制对象的坐标系，默认全局坐标系
                                       // 添加路径点
        for (int i = 0; i < 5; i++)
        {
            xviz::Vec2f p;
            p.x = float(i);
            p.y = 2.0f * float(i);
            path.points.emplace_back(p);
        }
        m_bridge->PathPub(name, path);
    }
    /// @brief
    /// @param x 相对父坐标系的x轴偏移量
    /// @param y 相对父坐标系的y轴偏移量
    /// @param yaw 相对父坐标系的y轴偏移量
    /// @param frame_id 坐标系名称
    /// @param parentFrameId 父坐标系名称
    void PubTransformNodeDemo(const float x, const float y, const float yaw,
                              const std::string &frame_id,
                              const std::string &parentFrameId)
    {
        TransformNode tf_node;
        tf_node.m_frameId = frame_id;
        tf_node.m_parentFrameId = parentFrameId;
        tf_node.m_transform.Set(Vec2f(x, y), yaw);
        m_bridge->TransformPub(frame_id, tf_node);
    }

    /// @brief 发布位姿
    /// @param x 相对名称为frame_id的坐标轴的x值
    /// @param y 相对名称为frame_id的坐标轴的y值
    /// @param yaw 相对名称为frame_id的坐标轴的yaw值
    /// @param name 位姿名称
    /// @param frameId 位姿所在坐标系
    void PubPoseDemo(const float x, const float y, const float yaw,
                     const std::string &name, const std::string &frameId)
    {
        Pose pose;
        pose.header.frameId = frameId;
        pose.x = x;
        pose.y = y;
        pose.yaw = yaw;
        m_bridge->PosePub(name, pose);
    }

    /// @brief 圆发布demo
    /// @param name
    /// @param frameId
    void PubCircleDemo(const std::string &name, const std::string &frameId)
    {
        Circle circle;
        circle.center.x = 2.0f;
        circle.center.y = 2.0f;
        circle.header.frameId = frameId;
        circle.radius = 1.0f;
        m_bridge->CirclePub(name, circle);
    }
    /// @brief 多边形发布demo
    /// @param name
    /// @param frameId
    void PubPolygonDemo(const std::string &name, const std::string &frameId)
    {
        // 图形需要封闭
        Polygon2f polygon;
        polygon.header.frameId = frameId;
        Vec2f p0{0.0f, 0.0f}, p1{2.5f, 0.0f}, p2{2.5f, 2.5f}, p3{0.0f, 2.5f};
        polygon.points.emplace_back(p0);
        polygon.points.emplace_back(p1);
        polygon.points.emplace_back(p2);
        polygon.points.emplace_back(p3);
        polygon.points.emplace_back(p0);
        m_bridge->PolygonPub(name, polygon);
    }

    /// @brief 发布线段组合
    /// @param name
    /// @param frameId
    void PubPathArrayDemo(const std::string &name, const std::string &frameId)
    {
        Path2f path;
        path.header.frameId = frameId; // 确定绘制对象的坐标系，默认全局坐标系
                                       // 添加路径点
        for (int i = 0; i < 5; i++)
        {
            Vec2f p;
            p.x = float(i);
            p.y = 2.0f * float(i);
            path.points.emplace_back(p);
        }

        Path2f path1, path2, path3;
        Path2fArray pathArray;
        pathArray.header.frameId = frameId;
        path1.header.frameId = frameId;
        path2.header.frameId = frameId;
        path3.header.frameId = frameId;
        pathArray.pathArray.resize(3);
        Transform tf1(1.0f, 1.0f, 0.5f * XVIZ_PI2);
        // 路径平移与旋转
        path1.points = Mul(tf1, path.points);

        path2.points = Mul(tf1, path1.points);
        path3.points = Mul(tf1, path2.points);
        path1.m_id = 1;
        path2.m_id = 2;
        path3.m_id = 3;
        pathArray.pathArray[0] = path1;
        pathArray.pathArray[1] = path2;
        pathArray.pathArray[2] = path3;
        m_bridge->PathArrayPub(name, pathArray);
    }

    /// @brief 发布多边形组合，用于普通库位与统一类型的障碍物
    void PubPolygonArrayDemo(const std::string &name,
                             const std::string &frameId)
    {

        Polygon2f polygon, polygon1, polygon2, polygon3;
        polygon.header.frameId = frameId;
        polygon1.header.frameId = frameId;
        polygon2.header.frameId = frameId;
        polygon3.header.frameId = frameId;
        Vec2f p0{0.0f, 0.0f}, p1{2.5f, 0.0f}, p2{2.5f, 2.5f}, p3{0.0f, 2.5f};
        polygon.points.emplace_back(p0);
        polygon.points.emplace_back(p1);
        polygon.points.emplace_back(p2);
        polygon.points.emplace_back(p3);
        polygon.points.emplace_back(p0);

        Transform tf1(1.0f, 1.0f, 0.5f * XVIZ_PI2);
        Polygon2fArray polygonArray;
        polygon1.points = Mul(tf1, polygon.points);
        polygon2.points = Mul(tf1, polygon1.points);
        polygon3.points = Mul(tf1, polygon2.points);
        polygon1.m_id = 1;
        polygon2.m_id = 2;
        polygon3.m_id = 3;
        polygonArray.polygonArray.emplace_back(polygon1);
        polygonArray.polygonArray.emplace_back(polygon2);
        polygonArray.polygonArray.emplace_back(polygon3);
        m_bridge->PolygonArrayPub(name, polygonArray);
    }
    /// @brief 发布点云数据
    /// @param name
    /// @param frameId
    void PubPointCloudDemo(const std::string &name, const std::string &frameId)
    {

        PointCloud3f cloud;
        cloud.header.frameId = frameId;
        cloud.m_id = 1;
        PointXYZ pt;
        for (int i = 0; i < 10; i++)
        {
            for (int j = 0; j < 10; j++)
            {
                pt.x = 0.5f * i;
                pt.y = 0.5f * j;
                cloud.points.emplace_back(pt);
            }
        }

        m_bridge->PointCloudPub(name, cloud);
    }
    /// @brief 绘制直线类型的marker，可指定颜色，粗细
    /// @param name
    /// @param frameId
    void PubMarkerPathDemo(const std::string &name, const std::string &frameId)
    {
        Marker marker;
        marker.header.frameId = frameId;
        marker.colorType = ColorType::RED;
        marker.thickness = 0.05f;
        marker.type = MarkerType::PATH;

        for (float t = 0.0f; t < XVIZ_PI2; t += 0.1f)
        {
            float x = 5.0f * (t - sinf(t));
            float y = 5.0f * (1 - cosf(t));
            marker.path.points.emplace_back(x, y);
        }

        m_bridge->MarkerPub(name, marker);
    }
    /// @brief 发布多边形类型的marker
    /// @param name
    /// @param frameId
    void PubMarkerPolygonDemo(const std::string &name,
                              const std::string &frameId)
    {
        Marker marker;
        marker.header.frameId = frameId;

        marker.type = MarkerType::POLYGON;
        marker.colorType = ColorType::PURPLE;
        marker.thickness = 0.05f;
        Vec2f p0{4.0f, 4.0f}, p1{5.0f, 5.0f}, p2{5.0f, 4.0f};
        marker.polygon.points.emplace_back(p0);
        marker.polygon.points.emplace_back(p1);
        marker.polygon.points.emplace_back(p2);
        marker.polygon.points.emplace_back(p0);

        m_bridge->MarkerPub(name, marker);
    }
    /// @brief 发你位姿类型的marker
    /// @param name
    /// @param frameId
    void PubMarkerPoseDemo(const std::string &name, const std::string &frameId)
    {
        Marker marker;
        marker.header.frameId = frameId;

        marker.type = MarkerType::POSE;
        marker.colorType = ColorType::PINK;
        marker.thickness = 0.1f;
        marker.length = 2.0f;
        marker.pose.x = 6.0f;
        marker.pose.y = 0.0f;
        marker.pose.yaw = XVIZ_PI2;
        m_bridge->MarkerPub(name, marker);
    }
    /// @brief 发布组合几何类型，以构建车辆为例子
    /// @param name
    /// @param frameId
    void PubMarkerArrayDemo(const std::string &name, const std::string &frameId)
    {
        const float wb = 2.984f;
        const float width = 1.977f;
        const float foh = 0.895f;
        const float roh = 1.163f;
        const float tyre_length = 0.6f; // 车轮直径
        const float tyre_width = 0.2f;  // 车轮宽度
        const float half_wheel_dist = 0.7f;
        Polygon2f wheelBox;
        const float angle = 0.5f * XVIZ_PI2;
        Rot steer_rot(angle);
        // 获取车轮矩形
        GenRectangle(-0.5f * tyre_length, 0.5f * tyre_width, 0.5f * tyre_length,
                     -0.5f * tyre_width, wheelBox);

        MarkerArray markerArray;
        markerArray.header.frameId = frameId;

        Marker marker;
        marker.type = MarkerType::POLYGON;
        marker.thickness = 0.05f;
        marker.colorType = ColorType::BLACK;
        // 创建车辆矩形
        GenRectangle(-roh, 0.5f * width, wb + foh, -0.5f * width, marker.polygon);
        markerArray.markers.emplace_back(marker);
        // 添加左前轮
        marker.polygon.points =
            Mul(steer_rot, wheelBox.points) + Vec2f(wb, half_wheel_dist);
        markerArray.markers.emplace_back(marker);
        // 添加右前轮
        marker.polygon.points =
            Mul(steer_rot, wheelBox.points) + Vec2f(wb, -half_wheel_dist);
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
        marker.pose.yaw = angle;
        markerArray.markers.emplace_back(marker);
        m_bridge->MarkerArrayPub(name, markerArray);
    }

    void PubMarkerPointsDemo(const std::string &name,
                             const std::string &frameId)
    {
        Marker marker;
        marker.header.frameId = frameId;

        marker.type = MarkerType::POINTCLOUD;
        marker.colorType = ColorType::CYAN;
        marker.radius = 0.1f;
        marker.pointcloud.points.emplace_back(5.0f, 0.0f, 0.0f);
        marker.pointcloud.points.emplace_back(4.5f, 0.0f, 0.0f);
        marker.pointcloud.points.emplace_back(5.5f, 0.0f, 0.0f);
        marker.pointcloud.points.emplace_back(5.0f, 0.5f, 0.0f);
        marker.pointcloud.points.emplace_back(5.0f, -0.5f, 0.0f);

        m_bridge->MarkerPub(name, marker);
    }

    /// @brief 发布float历史数据demo
    void PubFloatDataDemo()
    {
        float a1 = RandomRange(0.0f, 10.0f);
        float a2 = RandomRange(5.0f, 10.0f);
        float a3 = RandomRange(0.0f, 100.0f);
        float a4 = RandomRange(50.0f, 100.0f);
        float a5 = RandomRange(-500.0f, 500.0f);

        float b1 = 2.0f * a1;
        float b2 = 2.0f * a2;
        float b3 = 2.0f * a3;
        float b4 = 2.0f * a4;
        float b5 = 2.0f * a5;
        // 添加数据
        m_bridge->AddFloatData("a1", a1);
        m_bridge->AddFloatData("a2", a2);
        m_bridge->AddFloatData("a3", a3);
        m_bridge->AddFloatData("a4", a4);
        m_bridge->AddFloatData("a5", a5);

        m_bridge->AddFloatData("b1", b1);
        m_bridge->AddFloatData("b2", b2);
        m_bridge->AddFloatData("b3", b3);
        m_bridge->AddFloatData("b4", b4);
        m_bridge->AddFloatData("哈哈", b5);
        // 发布数据
        m_bridge->FloatDataPub("FloatData");
    }

    void PubStringDataDemo()
    {
        std::string aaa = "aaa";
        std::string bbb = "bbb";
        std::string ccc = "ccc";
        std::string ddd = "ddd";
        std::string haha = "哈哈";
        // 添加数据
        m_bridge->AddStringData("aaa", aaa);
        m_bridge->AddStringData("bbb", bbb);
        m_bridge->AddStringData("ccc", ccc);
        m_bridge->AddStringData("ddd", ddd);
        m_bridge->AddStringData("哈哈", haha);
        // 发布数据
        m_bridge->StringDataPub("StringData");
    }

    /// @brief 发布局部坐标系下的路径demo

    /// @brief 初始位姿回调函数
    /// @param pose
    void InitPoseCallback(const Pose &pose)
    {
        std::cout << "receive init pose" << std::endl;
        std::cout << "Pose x " << pose.x << " y " << pose.y << " yaw" << pose.yaw
                  << std::endl;
    }
    /// @brief 目标位姿回调函数
    /// @param pose
    void TarPoseCallback(const Pose &pose)
    {
        std::cout << "receive tar pose" << std::endl;
        std::cout << "Pose x " << pose.x << " y " << pose.y << " yaw" << pose.yaw
                  << std::endl;
    }

private:
    std::unique_ptr<xviz::XvizMsgBridge> m_bridge;
};

int main(int argc, char const *argv[])
{

    Demo demo;
    if (!demo.Init())
    {
        std::cout << "failed to init " << std::endl;

        return 1;
    }
    const std::string worldFrameId =
        WORLD_FRAME_ID;                      // 全局坐标系名称，Xviz中默认的全局坐标系
    const std::string frameId1 = "frameId1"; // 局部坐标系1
    const std::string frameId2 = "frameId2"; // 局部坐标系2

    while (true)
    {
        // 发布全局路径
        demo.PubPathDemo("worldPath", worldFrameId);
        // 发布全局坐标系下的圆
        demo.PubCircleDemo("worldCircle", worldFrameId);

        demo.PubPolygonDemo("worldPolygon", worldFrameId);

        demo.PubPathArrayDemo("worldPathArray", worldFrameId);

        demo.PubPolygonArrayDemo("worldPolygonArray", worldFrameId);

        demo.PubPointCloudDemo("worldPoints", worldFrameId);

        demo.PubMarkerPathDemo("worldMarkerPath", worldFrameId);
        demo.PubMarkerPolygonDemo("worldMarkerPolygon", worldFrameId);

        demo.PubMarkerPoseDemo("worldMarkerPose", worldFrameId);
        demo.PubMarkerPointsDemo("worldMarkerPoints", worldFrameId);
        demo.PubMarkerArrayDemo("worldMarkeArray", worldFrameId);

        float x = 10.0f;
        float y = 10.0f;
        float yaw = XVIZ_PI2;
        // 发布全局坐标系下位姿数据
        demo.PubPoseDemo(x, y, yaw, "worldPose", worldFrameId);
        //  发布该位姿的局部坐标系frameId1
        demo.PubTransformNodeDemo(x, y, yaw, frameId1, worldFrameId);
        // 发布局部坐标系frameId1下的位姿数据
        demo.PubPoseDemo(x, y, yaw, "frameId1Pose1", frameId1);
        // 发布局部坐标系frameId1下的路径
        demo.PubPathDemo("frameId1Path", frameId1);

        // 发布局部坐标系frameId1下的圆
        demo.PubCircleDemo("frameId1Circle", frameId1);

        demo.PubPolygonDemo("frameId1Polygon", frameId1);

        demo.PubPathArrayDemo("frameId1PathArray", frameId1);

        demo.PubPolygonArrayDemo("frameId1PolygonArray", frameId1);

        demo.PubPointCloudDemo("frameId1Points", frameId1);

        demo.PubMarkerPathDemo("frameId1MarkerPath", frameId1);

        demo.PubMarkerPolygonDemo("frameId1MarkerPolygon", frameId1);

        demo.PubMarkerPoseDemo("frameId1MarkerPose", frameId1);

        demo.PubMarkerPointsDemo("frameId1MarkerPoints", frameId1);

        demo.PubMarkerArrayDemo("frameId1MarkerArray", frameId1);

        // 发布局部坐标系frameId2下的位姿数据
        demo.PubPoseDemo(x, y, yaw, "frameId2Pose2", frameId2);
        // 相对frameId1 平移旋转后的frameId2
        demo.PubTransformNodeDemo(x, y, yaw, frameId2, frameId1);
        // 发布局部坐标系frameId2下的路径
        demo.PubPathDemo("frameId2Path", frameId2);
        // 发布局部坐标系frameId2下的圆
        demo.PubCircleDemo("frameId2Circle", frameId2);

        demo.PubPolygonDemo("frameId2Polygon", frameId2);

        demo.PubPathArrayDemo("frameId2PathArray", frameId2);

        demo.PubPolygonArrayDemo("frameId2PolygonArray", frameId2);

        demo.PubPointCloudDemo("frameId2Points", frameId2);
        demo.PubMarkerPathDemo("frameId2MarkerPath", frameId2);

        demo.PubMarkerPolygonDemo("frameId2MarkerPolygon", frameId2);

        demo.PubMarkerPoseDemo("frameId2MarkerPose", frameId2);

        demo.PubMarkerPointsDemo("frameId2MarkerPoints", frameId2);
        demo.PubMarkerArrayDemo("frameId2MarkerArray", frameId2);

        // 发布float数据demo
        demo.PubFloatDataDemo();
        // 发布string数据demo
        demo.PubStringDataDemo();

        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

    return 0;
}
