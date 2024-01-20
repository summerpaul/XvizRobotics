/**
 * @Author: Xia Yunkai
 * @Date:   2023-12-29 09:22:25
 * @Last Modified by:   Xia Yunkai
 * @Last Modified time: 2024-01-20 12:40:57
 */
#include <stdint.h>

#ifndef __XVIZMSGSENDER_H__
#define __XVIZMSGSENDER_H__

#include "data_types.h"
#include <mutex>
#include <string>
#include <memory>
#include <functional>
#include <thread>
#include <unordered_map>
namespace zmq
{
    class context_t;
    class socket_t;

}

namespace xviz
{
    /// @brief 下发位姿的回调函数
    typedef std::function<void(const Pose &)> PoseCallbackFunc;
    /// @brief 作为Server与Xviz通信
    class XvizMsgBridge
    {
    public:
        typedef std::unique_ptr<XvizMsgBridge> Ptr;
        /// @brief 构造函数
        XvizMsgBridge();
        
        /// @brief 析构函数
        ~XvizMsgBridge();
        
        /// @brief 初始化
        /// @param pub_connect 发布数据需要绑定的zmq Ip与端口 
        /// @param sub_connect 接受数据需要绑定的zmq Ip与端口 （目前实现接受起点与终点位姿）
        /// @return 
        bool Init(const std::string &pub_connect, const std::string &sub_connect);
        
        /// @brief Init之后运行
        void Run();

        /// @brief 设置起点位置回调函数 
        /// @param func 回调函数
        void SetInitPoseFunc(const PoseCallbackFunc &func);

        /// @brief 设置终点位置回调函数
        /// @param func 回调函数
        void SetTargetPoseFunc(const PoseCallbackFunc &func);

        /// @brief 下发可视化路径
        /// @param topic 名称
        /// @param path 路径
        void PathPub(const std::string &topic, const Path2f &path);

        /// @brief 下发可视化路径数组
        /// @param topic 名称
        /// @param pathArray 路径数组
        void PathArrayPub(const std::string &topic, const Path2fArray &pathArray);


        /// @brief 发布位姿
        /// @param topic 名称
        /// @param pose 位姿
        void PosePub(const std::string &topic, const Pose &pose);

        /// @brief 发布点云数据
        /// @param topic 名称
        /// @param pointcloud 点云数据 
        void PointCloudPub(const std::string &topic, const PointCloud3f &pointcloud);

        /// @brief 多边形数据发布
        /// @param topic 名称
        /// @param polygon 多边形
        void PolygonPub(const std::string &topic, const Polygon2f &polygon);

        /// @brief 发布多边形数据组
        /// @param topic 名称
        /// @param polygonArray  多边形数据组
        void PolygonArrayPub(const std::string &topic, const Polygon2fArray &polygonArray);

        /// @brief 发布圆数据类型
        /// @param topic 
        /// @param circle 
        void CirclePub(const std::string &topic, const Circle &circle);

        /// @brief bezier 数据发布
        /// @param topic 名称
        /// @param bezier 贝塞尔数据
        void BezierPub(const std::string &topic, const Bezier &bezier);
        
        /// @brief 
        /// @param topic 
        /// @param marker 
        void MarkerPub(const std::string&topic, const Marker& marker);

        /// @brief 发布组合可视化数据组
        /// @param topic 名称
        /// @param markerArray 可视化数据组 
        void MarkerArrayPub(const std::string &topic, const MarkerArray &markerArray);

        /// @brief 添加float 数据类型
        /// @param name 名称
        /// @param data 数据
        void AddFloatData(const std::string &name, const float data);

        /// @brief 添加string数据类型数据
        /// @param name 名称
        /// @param data 数据
        void AddStringData(const std::string &name, const std::string &data);

        /// @brief 发布float类型数据，在添加flaot数据类型后使用该函数
        /// @param name 名称
        void FloatDataPub(const std::string &name);

        /// @brief 发布string数据，在添加完string数据类型后使用该函数
        /// @param name 名称
        void StringDataPub(const std::string &name);

        /// @brief 发布栅格地图与单通道图片
        /// @param name 名称
        /// @param map 栅格地图或单通道图片
        void GridMapPub(const std::string &name, const GridMap &map);

        /// @brief 发布坐标系
        /// @param name 名称
        /// @param transform 坐标系 
        void TransformPub(const std::string &name, const TransformNode &transform);
        
        /// @brief 销毁程序
        void Shutdown();

    private:
        /// @brief 发布protobuf数据模板
        /// @tparam PROTO_MSG proto数据模板
        /// @param proto_msg 
        /// @param topic 名称
        /// @param msg_type 数据类型
        template <typename PROTO_MSG>
        void PubProto(const PROTO_MSG &proto_msg,
                      const std::string &topic, const std::string &msg_type);
        /// @brief 接受数据回调函数
        void ReceiveLoop();

    private:
        std::unique_ptr<zmq::context_t> m_ctx;
        std::unique_ptr<zmq::socket_t> m_pub;
        std::unique_ptr<zmq::socket_t> m_sub;
        std::thread m_receiveThread;
        std::string m_pubConnect;
        std::string m_subConnect;
        bool m_running;
        std::mutex m_mtx;
        PoseCallbackFunc m_initPoseCB;
        bool m_bSetInitPoseCB = false;
        PoseCallbackFunc m_tarPoseCB;
        bool m_bSetTarPoseCB = false;
        std::unordered_map<std::string, float> m_floatData;
        std::unordered_map<std::string, std::string> m_stringData;
    };
}
#endif /* __XVIZMSGSENDER_H__ */
