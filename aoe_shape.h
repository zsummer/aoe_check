/*
* aoe License
* Copyright (C) 2019 YaweiZhang <yawei.zhang@foxmail.com>.
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
* http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*/

#pragma once
#ifndef SPELL_RANGE_H
#define SPELL_RANGE_H

#include "aoe_common.h"
#include "vector3.h"
#include <array>

struct DeviationShape;

enum AreaShapeType
{
    AREA_SHAPE_CIRCLE = 0,
    AREA_SHAPE_FAN = 1,
    AREA_SHAPE_RECT = 2,
    AREA_SHAPE_RING = 3,
    AREA_SHAPE_FRAME = 4,
    AREA_SHAPE_FOV = 5,
};

class AreaShapeRect
{
public:
    static const std::size_t VERTEX_SIZE = 4;
    s32 Init(DeviationShape deviation, f32 radius, bool collide_test, bool frame_test);
    s32 PointInRange(const Point3& pos, f32 radius, f32 & dist_sq);
private:
    std::array<Point3, VERTEX_SIZE> vertexs_; //凸多边形的多有顶点
    std::array<f32, VERTEX_SIZE> line_inv_length_; //凸多边形顶点组成的每个边长的倒数
    std::array<Point3, VERTEX_SIZE> lines_; //凸多边形顶点组成的每个边长的倒数
    Point3 anchor_; //定位点
    f32 anchor_radius_; //定位点半径
    f32 distance_;
    f32 high_;
    bool collide_test_;
    bool frame_test_;
};

class AreaShapeFan
{
public:
    s32 Init(DeviationShape deviation, f32 radius);
    s32 PointInRange(const Point3& pos, f32 radius, f32 & dist_sq);
private:
    Point3 anchor_; //定位点
    f32 anchor_radius_; //定位点半径
    bool is_circle_; //是否是圆
    Point3 normalize_dir_; //朝向
    f32 radian_; //弧度 (half)
    f32 radian_domain_; //余弦值域 (half)
    f32 radian_radius_; //扇形半径
    f32 high_;
};

class AreaShapeCircle
{
public:
    //param2为大圆半径, param1为挖空的小圆半径 小圆大小可以为0
    s32 Init(DeviationShape deviation, f32 radius);
    s32 PointInRange(const Point3& pos, f32 radius, f32 & dist_sq);
private:
    Point3 anchor_; //定位点
    f32 anchor_radius_; //定位点半径
    f32 min_radius_sq_;
    f32 max_radius_;
    f32 high_;
};


class AreaShapeFov
{
public:
    //param2为大圆半径, param1为挖空的小圆半径 小圆大小可以为0
    s32 Init(DeviationShape deviation, f32 radius);
    s32 PointInRange(const Point3& pos, f32 radius, f32& dist_sq);
private:
    DeviationShape shape;
};



class AreaShape
{
public:
    AreaShape();
    ~AreaShape() {}
    s32 Init(u32 shape_type, DeviationShape deviation, f32 radius);
    s32 PointInRange(const Point3& pos, f32 radius, f32& dist_sq);
    u32 shape_type() const { return shape_type_; }
private:
    union
    {
        AreaShapeFan fan_;
        AreaShapeCircle circle_;
        AreaShapeRect rect_;
        AreaShapeFov fov_;
    };
    u32 shape_type_;
};


#endif // 
