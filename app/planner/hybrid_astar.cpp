/**
 * @Author: Xia Yunkai
 * @Date:   2024-02-10 22:38:08
 * @Last Modified by:   Xia Yunkai
 * @Last Modified time: 2024-02-17 23:47:47
 */
#include <iostream>
#include "hybrid_astar.h"
#include "common/shapes.h"
namespace planner
{

    bool HybridAStar::Search(const Vec4f &start, const Vec4f &goal, const Vec2f &init_ctrl)
    {

        if (m_gridMap == nullptr)
        {
            return false;
        }

        bool initsearch = false;

        if (CheckCollisionUsingPosAndYaw(start.head(3)))
        {
            std::cout << " head is not free " << std::endl;
            return false;
        }

        if (CheckCollisionUsingPosAndYaw(goal.head(3)))
        {
            std::cout << " goal is not free " << std::endl;
            return false;
        }
        m_startState = start;
        m_goalState = goal;
        m_initCtrl = init_ctrl;
        Vec2i end_index = m_gridMap->FloatToInt(goal.head(2));
        /* ---------- initialize ---------- */

        

    }

    bool HybridAStar::CheckCollisionUsingPosAndYaw(const Vec3f &pos)
    {

        if (m_gridMap == nullptr)
        {
            return false;
        }

        Points2f vertices;
        common::OrientedBoundingBox2D obb_ego;
        obb_ego.m_x = pos[0] + m_param.DCR() * cosf(pos[2]);
        obb_ego.m_y = pos[1] + m_param.DCR() * sinf(pos[2]);
        obb_ego.m_heading = pos[2];
        obb_ego.m_width = m_param.Width();
        obb_ego.m_length = m_param.Length();
        common::ShapeUtils::GetDenseVerticesOfOrientedBoundingBox(obb_ego, vertices);
        for (auto &v : vertices)
        {
            if (m_gridMap->IsOccupied(v))
            {
                return true;
            }
        }
        return false;
    }
}