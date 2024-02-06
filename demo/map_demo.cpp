/**
 * @Author: Xia Yunkai
 * @Date:   2024-02-06 18:03:02
 * @Last Modified by:   Xia Yunkai
 * @Last Modified time: 2024-02-06 18:46:03
 */
#include <iostream>
#include "data_types.h"
#include "parking_case_parser/parking_case_parser.h"
#include "xvizMsgBridge.h"
#include "xviz_marcro.h" //宏定义
#include "xviz_math.h"   // 坐标转换相关函数
#include "xviz_utils.h"  //
#include <chrono>
#include <iostream>
#include <memory>
#include <string>
#include <thread>
#include "map/grid_map.h"
#include "map/esdf_map.h"

using namespace std;

class MapDemo
{

public:
    using Ptr = std::unique_ptr<MapDemo>;
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
            [this](auto &&PH1)
            { InitPoseCallback(std::forward<decltype(PH1)>(PH1)); });
        // 绑定目标位姿回调函数
        m_bridge->SetTargetPoseFunc(
            [this](auto &&PH1)
            { TarPoseCallback(std::forward<decltype(PH1)>(PH1)); });

        m_bridge->Run();
        m_parkingCase = std::make_shared<parking_case_parser::ParkingCase>();

        return true;
    }

    bool Parse(const std::string &path, const float res = 0.05f)
    {
        return m_parkingCase->Parse(path, res);
    }

    void GenXvizMsg()
    {
        m_startPose.x = float(m_parkingCase->m_startPose[0]);
        m_startPose.y = float(m_parkingCase->m_startPose[1]);
        m_startPose.yaw = float(m_parkingCase->m_startPose[2]);
        m_tarPose.x = float(m_parkingCase->m_tarPose[0]);
        m_tarPose.y = float(m_parkingCase->m_tarPose[1]);
        m_tarPose.yaw = float(m_parkingCase->m_tarPose[2]);

        m_baseLink.m_transform = xviz::PoseToTransform(m_startPose);
        m_baseLink.m_frameId = "base_link";
        m_baseLink.m_parentFrameId = xviz::WORLD_FRAME_ID;

        const int obs_num = m_parkingCase->m_obs.size();
        m_obsArray.polygonArray.resize(obs_num);
        for (int i = 0; i < obs_num; i++)
        {
            xviz::Polygon2f ob;
            const int points_num = m_parkingCase->m_obs[i].size();
            ob.points.resize(points_num + 1);
            for (int j = 0; j < points_num; j++)
            {
                ob.points[j].x = m_parkingCase->m_obs[i][j][0];
                ob.points[j].y = m_parkingCase->m_obs[i][j][1];
                if (j == points_num - 1)
                {
                    ob.points[j + 1] = ob.points[0];
                }
            }
            m_obsArray.polygonArray[i] = ob;
        }

        auto map = m_parkingCase->GetMap();

        m_gridMap.m_isTopView = true;
        m_gridMap.m_reserve = true;
        m_gridMap.m_res = map->GetResolution();
        m_gridMap.m_data = std::string(map->GetData(), map->GetDataSize());

        m_gridMap.m_origin.x = map->GetOrigin().translation().x();
        m_gridMap.m_origin.y = map->GetOrigin().translation().y();
        m_gridMap.m_origin.yaw = map->GetOrigin().rotation().angle();
        m_gridMap.m_size.x = map->GetSize().x();
        m_gridMap.m_size.y = map->GetSize().y();
    }

    void Publish()
    {
        m_bridge->PosePub("start_pose", m_startPose);
        m_bridge->TransformPub("base_link", m_baseLink);

        m_bridge->PosePub("tarPose", m_tarPose);
        m_bridge->PolygonArrayPub("obs", m_obsArray);
        m_bridge->GridMapPub("map", m_gridMap);
        // 发布栅格地图
    }

    void InitPoseCallback(const xviz::Pose &pose)
    {

        auto grid_map = m_parkingCase->GetMap();
        common::Vec2f pt(pose.x, pose.y);

        if (grid_map->IsOccupied(pt))
        {
            std::cout << "pose " << pt.transpose() << " IsOccupied" << std::endl;
        }
        else
        {
            std::cout << "pose " << pt.transpose() << " IsFree" << std::endl;
        }
    }
    /// @brief 目标位姿回调函数
    /// @param pose
    void TarPoseCallback(const xviz::Pose &pose)
    {
        std::cout << "receive tar pose" << std::endl;
        std::cout << "Pose x " << pose.x << " y " << pose.y << " yaw" << pose.yaw
                  << std::endl;
    }

private:
    parking_case_parser::ParkingCase::Ptr m_parkingCase;
    std::unique_ptr<xviz::XvizMsgBridge> m_bridge;

    xviz::Pose m_startPose, m_tarPose;
    xviz::TransformNode m_baseLink;
    xviz::Polygon2fArray m_obsArray;
    xviz::GridMap m_gridMap;
};

int main(int argc, char const *argv[])
{

    MapDemo::Ptr demo = std::make_unique<MapDemo>();

    if (!demo->Init())
    {
        return 1;
    }

    std::string name = argv[1];

    if (!demo->Parse(name))
    {
        return 1;
    }

    demo->GenXvizMsg();

    while (true)
    {
        demo->Publish();

        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    return 0;
}
