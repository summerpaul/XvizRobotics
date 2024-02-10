/**
 * @Author: Xia Yunkai
 * @Date:   2024-02-10 20:48:50
 * @Last Modified by:   Xia Yunkai
 * @Last Modified time: 2024-02-10 22:47:11
 */
#include <stdint.h>

#ifndef __HYBRID_ASTAR_H__
#define __HYBRID_ASTAR_H__
#include "common/data_types.h"
#include "map/grid_map.h"
#include "common/vehicle_param.h"
namespace planner
{
    using namespace common;
    using namespace map;

    class HybridAStar
    {

    public:
        HybridAStar() {}
        ~HybridAStar() {}
        void SetMap(const GridMap::Ptr &map) { m_map = map; }

        bool Search(const Vec4f &start, const Vec4f &goal, const Vec2f &init_ctrl);

    private:
        bool CheckCollisionUsingPosAndYaw(const Vec3f &pos);

    private:
        GridMap::Ptr m_gridMap;
        VehicleParam m_param;
        };

} // namespace planner

#endif /* __HYBRID_ASTAR_H__ */
