/**
 * @Author: Xia Yunkai
 * @Date:   2024-02-10 22:38:08
 * @Last Modified by:   Xia Yunkai
 * @Last Modified time: 2024-02-10 22:48:20
 */
#include <iostream>
#include "hybrid_astar.h"

namespace planner
{

    bool HybridAStar::Search(const Vec4f &start, const Vec4f &goal, const Vec2f &init_ctrl)
    {

        if (m_gridMap == nullptr)
        {
            return false;
        }

        bool isocc = false;
        bool initsearch = false;
    }

    bool HybridAStar::CheckCollisionUsingPosAndYaw(const Vec3f &pos)
    {

        if (m_gridMap == nullptr)
        {
            return false;
        }
    }
}