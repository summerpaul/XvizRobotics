/**
 * @Author: Xia Yunkai
 * @Date:   2024-02-10 22:29:41
 * @Last Modified by:   Xia Yunkai
 * @Last Modified time: 2024-02-10 22:35:27
 */
#include <stdint.h>

#ifndef __NODE3D_H__
#define __NODE3D_H__

#include "common/data_types.h"
namespace planner
{
#define IN_CLOSE_SET 'a'
#define IN_OPEN_SET 'b'
#define NOT_EXPAND 'c'
    using namespace common;
    class Node3D
    {
    public:
        Vec2i m_index;
        int m_yawIdx;
        Vec3f m_state;
        float m_gScore;
        float m_fScore;
        Vec2f m_input;
        Node3D *m_parent;
        char m_nodeState;
        int m_singul = 0;

        Node3D()
        {
            m_nodeState = NOT_EXPAND;
            m_parent = NULL;
        }
        ~Node3D() {}
    };

    using Node3DPtr = Node3D *;

    class Node3DCompare
    {
    public:
        bool operator()(const Node3DPtr &a, const Node3DPtr &b) const
        {
            return a->m_fScore > b->m_fScore;
        }
    };

} // namespace planner

#endif /* __NODE3D_H__ */
