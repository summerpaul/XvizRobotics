/**
 * @Author: Xia Yunkai
 * @Date:   2024-02-10 22:26:01
 * @Last Modified by:   Xia Yunkai
 * @Last Modified time: 2024-02-10 23:23:01
 */
#include <stdint.h>

#ifndef __KINO_MODEL_H__
#define __KINO_MODEL_H__

#include "common/data_types.h"
#include "common/vehicle_param.h"

namespace planner
{
    using namespace common;
    class KinoModel
    {

    public:
        KinoModel() {}
        virtual ~KinoModel() {}

        void SetParam(const VehicleParam& param){ m_param = param;}

        

    public:
        VehicleParam m_param;
        Vec4f m_state;
        Vec2f m_control;
    };

} // namespace planner

#endif /* __KINO_MODEL_H__ */
