/**
 * @Author: Xia Yunkai
 * @Date:   2024-01-20 20:51:42
 * @Last Modified by:   Xia Yunkai
 * @Last Modified time: 2024-01-20 23:11:10
 */
#include "auto_parking/geometry_parking_utils.h"
#include "auto_parking/reverse_verticle_parking.h"
#include "data_types.h" //发送的基本数据类型
#include "xvizMsgBridge.h"
#include "xviz_marcro.h" //宏定义
#include "xviz_math.h"   // 坐标转换相关函数
#include "xviz_utils.h"  //
#include <algorithm>
#include <fstream>
#include <iostream>
#include <string>
using namespace std;

class AutoParkingDemo {
public:
  AutoParkingDemo() {}
  ~AutoParkingDemo() {}
  bool Init() {
    // step 1 :创建XvizMsgBridge对象
    m_bridge = std::make_unique<xviz::XvizMsgBridge>();
    // step 2:初始化通信
    if (!m_bridge->Init("tcp://127.0.0.1:8888", "tcp://127.0.0.1:8899")) {
      return false;
    }
    // 绑定初始位姿回调函数
    m_bridge->SetInitPoseFunc([this](auto &&PH1) {
      InitPoseCallback(std::forward<decltype(PH1)>(PH1));
    });
    // 绑定目标位姿回调函数
    m_bridge->SetTargetPoseFunc([this](auto &&PH1) {
      TarPoseCallback(std::forward<decltype(PH1)>(PH1));
    });
    m_tf.m_frameId = BASE_LINK;
    m_tf.m_parentFrameId = xviz::WORLD_FRAME_ID;
    m_vehicle.header.frameId = BASE_LINK;

    GenBoundaries();
    GenVehicleBox();

    m_bridge->Run();

    return true;
  }

  void GenVehicleBox() {
    const float HALF_WIDTH = 1.0;
    const float REAR_TO_RAC = 1.0;
    const float FRONT_TO_RAC = 3.7;

    xviz::Vec2f p0(-REAR_TO_RAC, HALF_WIDTH);
    xviz::Vec2f p1(-REAR_TO_RAC, -HALF_WIDTH);
    xviz::Vec2f p2(FRONT_TO_RAC, -HALF_WIDTH);
    xviz::Vec2f p3(FRONT_TO_RAC, HALF_WIDTH);
    m_vehicle.points.emplace_back(p0);
    m_vehicle.points.emplace_back(p1);
    m_vehicle.points.emplace_back(p2);
    m_vehicle.points.emplace_back(p3);
    m_vehicle.points.emplace_back(p0);
  }

  void GenBoundaries() {

    using namespace apollo::common::math;

    m_boundaries.emplace_back(Vec2d(-5.0, 4.0), Vec2d(-1.25, 4.0));
    m_boundaries.emplace_back(Vec2d(-1.25, 4.0), Vec2d(-1.25, -1.1));
    m_boundaries.emplace_back(Vec2d(-1.25, -1.1), Vec2d(1.25, -1.1));
    m_boundaries.emplace_back(Vec2d(1.25, -1.1), Vec2d(1.25, 4.0));
    m_boundaries.emplace_back(Vec2d(1.25, 4.0), Vec2d(5.0, 4.0));
    m_boundaries.emplace_back(Vec2d(-4.0, 9.8), Vec2d(5.0, 9.8));

    for (auto &seg : m_boundaries) {
      xviz::Path2f path;
      path.points.emplace_back(seg.start().x(), seg.start().y());
      path.points.emplace_back(seg.end().x(), seg.end().y());
      m_showBoundaries.pathArray.emplace_back(path);
    }
  }

  void InitPoseCallback(const xviz::Pose &pose) {
    std::cout << "receive init pose" << std::endl;
    std::cout << "Pose x " << pose.x << " y " << pose.y << " yaw" << pose.yaw
              << std::endl;
    m_tf.m_transform = xviz::PoseToTransform(pose);

    m_start = {Vec2d(pose.x, pose.y), pose.yaw,
               Vec2d::CreateUnitVec2d(pose.yaw)};
    // m_startVis = pose
  }

  void TarPoseCallback(const xviz::Pose &pose) {

    m_planPath.points.clear();
    const bool success =
        RunReverseVerticleParking(m_boundaries, m_start, &m_result);
    if (success) {
      const auto path = ConvertPathToDiscretePoses(m_result, 0.05);
      m_resultPath = path;
      for (const auto &pose : path) {
        m_planPath.points.emplace_back(pose.pos.x(), pose.pos.y());
        m_startRun = true;
      }
      printf("Success!\n");
    } else {
      m_startRun = false;
      printf("Fail!\n");
    }
  }
  void Show() {
    m_bridge->PathArrayPub("Boundaries", m_showBoundaries);
    m_bridge->PathPub("PlanPath", m_planPath);
    m_bridge->PolygonPub("Vehicle", m_vehicle);
    m_bridge->TransformPub("base_link", m_tf);
    static int path_id = 0;
    if (m_startRun) {
      if (path_id < m_resultPath.size() - 1) {
        const float x = (float)m_resultPath[path_id].pos.x();
        const float y = (float)m_resultPath[path_id].pos.y();
        const float angle = (float)m_resultPath[path_id].theta;

        m_tf.m_transform = {x, y, angle};
        m_bridge->AddFloatData("x", x);
        m_bridge->AddFloatData("y", y);
        m_bridge->AddFloatData("angle", angle);

        m_bridge->FloatDataPub("FloatData");
        path_id++;
      } else {
        m_startRun = false;
        path_id = 0;
      }
    }
    // m_bridge->PosePub("Start", m_startVis);
  }

private:
  std::unique_ptr<xviz::XvizMsgBridge> m_bridge;

public:
  std::vector<LineSegment2d> m_boundaries;
  xviz::Path2fArray m_showBoundaries;
  xviz::Polygon2f m_vehicle;
  xviz::Path2f m_planPath;
  Pose m_start;
  xviz::Pose m_startVis;
  xviz::TransformNode m_tf;
  const std::string BASE_LINK = "base_link";
  LineCirclePath m_result;
  std::vector<Pose> m_resultPath;
  bool m_startRun = false;
};

int main(int argc, char const *argv[]) {

  AutoParkingDemo demo;
  if (!demo.Init()) {
    return 1;
  }

  while (true) {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    demo.Show();
  }

  return 0;
}
