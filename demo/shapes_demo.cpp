/**
 * @Author: Xia Yunkai
 * @Date:   2024-02-10 23:28:06
 * @Last Modified by:   Xia Yunkai
 * @Last Modified time: 2024-02-11 00:07:46
 */
#include <iostream>
#include "common/shapes.h"
#include "xvizMsgBridge.h"
#include "xviz_marcro.h" //宏定义
#include "xviz_math.h"   // 坐标转换相关函数
#include "xviz_utils.h"  //
#include "common/vehicle_param.h"
using namespace std;

int main(int argc, char const *argv[])
{

    float x = 1.0f;
    float y = 1.0f;
    float yaw = XVIZ_PI2 * 0.5f;

    common::VehicleParam param;
    common::Points2f pts;
    common::OrientedBoundingBox2D obb(x, y, yaw, param.Width(), param.Length());

    auto flag = common::ShapeUtils::GetDenseVerticesOfOrientedBoundingBox(obb, pts, 0.05f);

    if (!flag)
    {
        return -1;
    }

    std::unique_ptr<xviz::XvizMsgBridge> xviz_msg_bridge =
        std::make_unique<xviz::XvizMsgBridge>();

    flag = xviz_msg_bridge->Init("tcp://127.0.0.1:8888", "tcp://127.0.0.1:8899");

    if (!flag)
    {
        std::cout << "Init Error" << std::endl;
    }
    xviz_msg_bridge->Run();

    xviz::PointCloud3f points;
    points.header.frameId = xviz::WORLD_FRAME_ID;
    points.points.resize(pts.size());
    for (int i = 0; i < pts.size(); i++)
    {
        points.points[i].x = pts[i].x();
        points.points[i].y = pts[i].y();
        points.points[i].z = 0.0f;
        // std::cout << points.points[i].x << " " << points.points[i].y << std::endl;
    }
    std::cout << points.points.size() << std::endl;
    common::Points2f obb_points;
    common::ShapeUtils::GetVerticesOfOrientedBoundingBox(obb, obb_points);
    xviz::Polygon2f polygon;
    polygon.header.frameId = xviz::WORLD_FRAME_ID;
    for (auto &pt : obb_points)
    {
        polygon.points.push_back(xviz::Vec2f(pt.x(), pt.y()));
        // std::cout << pt.x() << " " << pt.y() << std::endl;
    }
    polygon.points.push_back(xviz::Vec2f(obb_points.front().x(), obb_points.front().y()));

    while (true)
    {
        xviz_msg_bridge->PointCloudPub("points", points);
        xviz_msg_bridge->PolygonPub("polygon", polygon);
        // std::cout << "publish" << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    xviz_msg_bridge->Shutdown();

    return 0;
}
