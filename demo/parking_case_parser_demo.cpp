/**
 * @Author: Xia Yunkai
 * @Date:   2024-01-28 20:08:41
 * @Last Modified by:   Xia Yunkai
 * @Last Modified time: 2024-02-05 18:29:37
 */
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

int main(int argc, char const *argv[])
{

  if (argc < 2)
  {
    return 1;
  }
  std::string name = argv[1];
  parking_case_parser::ParkingCase parking_case;
  std::cout << name << std::endl;
  auto flag = parking_case.Parse(name);

  if (!flag)
  {
    std::cout << "Parse Error" << std::endl;
  }
  std::unique_ptr<xviz::XvizMsgBridge> xviz_msg_bridge =
      std::make_unique<xviz::XvizMsgBridge>();

  flag = xviz_msg_bridge->Init("tcp://127.0.0.1:8888", "tcp://127.0.0.1:8899");

  if (!flag)
  {
    std::cout << "Init Error" << std::endl;
  }
  xviz_msg_bridge->Run();

  float max_x = 0, min_x = 0, max_y = 0, min_y = 0;

  xviz::Pose start_pose, tar_pose;
  start_pose.x = float(parking_case.m_startPose[0]);
  start_pose.y = float(parking_case.m_startPose[1]);
  start_pose.yaw = float(parking_case.m_startPose[2]);
  tar_pose.x = float(parking_case.m_tarPose[0]);
  tar_pose.y = float(parking_case.m_tarPose[1]);
  tar_pose.yaw = float(parking_case.m_tarPose[2]);
  std::cout << start_pose.x << " " << start_pose.y << " " << start_pose.yaw << std::endl;
  std::cout << tar_pose.x << " " << tar_pose.y << " " << tar_pose.yaw << std::endl;

  xviz::TransformNode base_link;
  base_link.m_transform = xviz::PoseToTransform(start_pose);
  base_link.m_frameId = "base_link";
  base_link.m_parentFrameId = xviz::WORLD_FRAME_ID;

  xviz::Polygon2fArray obsArray;
  const int obs_num = parking_case.m_obs.size();
  obsArray.polygonArray.resize(obs_num);
  for (int i = 0; i < obs_num; i++)
  {
    xviz::Polygon2f ob;
    const int points_num = parking_case.m_obs[i].size();
    ob.points.resize(points_num + 1);
    for (int j = 0; j < points_num; j++)
    {
      ob.points[j].x = parking_case.m_obs[i][j][0];
      ob.points[j].y = parking_case.m_obs[i][j][1];
      if (j == points_num - 1)
      {
        ob.points[j + 1] = ob.points[0];
      }
    }
    obsArray.polygonArray[i] = ob;
  }

  xviz::GridMap grid_map;

  auto map = parking_case.GetMap();

  grid_map.m_isTopView = true;
  grid_map.m_reserve = true;
  grid_map.m_res = map->GetResolution();
  grid_map.m_data = std::string(map->GetData(), map->GetDataSize());

  grid_map.m_origin.x = map->GetOrigin().translation().x();
  grid_map.m_origin.y = map->GetOrigin().translation().y();
  grid_map.m_origin.yaw = map->GetOrigin().rotation().angle();
  grid_map.m_size.x = map->GetSize().x();
  grid_map.m_size.y = map->GetSize().y();

  while (true)
  {
    xviz_msg_bridge->PosePub("start_pose", start_pose);
    xviz_msg_bridge->TransformPub("base_link", base_link);

    xviz_msg_bridge->PosePub("tarPose", tar_pose);
    xviz_msg_bridge->PolygonArrayPub("obs", obsArray);
    xviz_msg_bridge->GridMapPub("map", grid_map);

    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
  xviz_msg_bridge->Shutdown();
  return 0;
}
