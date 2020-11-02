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

#include "aoe_shape.h"


s32 AreaShapeRect::Init(DeviationShape deviation, f32 radius, bool collide_test, bool frame_test)
{
    float len = deviation.pivot_scale.x;
    float wide = deviation.pivot_scale.y;
    float high = deviation.pivot_scale.z;

    if (len < FLOAT_POINT_PRECISION || wide < FLOAT_POINT_PRECISION)
    {
        LOGFMTE("config error. len:<%f>, wide:<%f>", len, wide);
        return -2;
    }
    Point3 tmp_dir = deviation.pivot_dir;
    if (tmp_dir.is_zero())
    {
        LOGFMTE("dir zero");
        return -3;
    }
    if (!tmp_dir.normalize())
    {
        LOGFMTE("dir normalize false");
        return -4;
    }
    frame_test_ = frame_test;
    collide_test_ = collide_test;

    Point3 vertex_point = deviation.pivot_pos;
    vertex_point.z = 0.0f;
    anchor_ = deviation.pivot_pos;
    anchor_radius_ = radius;


    //二维逆时针的凸多边形
    Point3 right = { tmp_dir.y, tmp_dir.x * -1.0f, 0.0f};
    Point3 left = { tmp_dir.y * -1.0f, tmp_dir.x, 0.0f};
    vertexs_[0] = vertex_point + right * (wide * 0.5f);
    vertexs_[1] = (vertexs_[0] + tmp_dir * len);
    lines_[0] = vertexs_[1] - vertexs_[0];
    line_inv_length_[0] = 1.0f / len;

    vertexs_[2] = (vertexs_[1] + left * wide);
    lines_[1] = vertexs_[2] - vertexs_[1];
    line_inv_length_[1] = 1.0f / wide;

    vertexs_[3] = vertexs_[0] + left * wide;
    lines_[2] = vertexs_[3] - vertexs_[2];
    line_inv_length_[2] = line_inv_length_[0];

    lines_[3] = vertexs_[0] - vertexs_[3];
    line_inv_length_[3] = line_inv_length_[1];

    distance_ = sqrtf(wide*wide + len * len);
    high_ = high;
    return 0;
}

s32 AreaShapeRect::PointInRange(const Point3& pos, f32 radius, f32 & dist_sq)
{
    //高度差检测
    using std::max;
    float height_dist = fabsf(pos.z - anchor_.z);
    if (height_dist > high_)
    {
        LOGFMTD("range rect too high anchor z:<%f>, test z:<%f>, limit high:<%f>", anchor_.z, pos.z, high_);
        return 1;
    }

    dist_sq = Point3(pos.x - anchor_.x, pos.y - anchor_.y, 0.0f).square_length();
    if (collide_test_ && dist_sq < FLOAT_POINT_PRECISION)
    {
        return 0;
    }

    //快速判断是否超过矩形的最远端. (对角线距离+双方半径) 
    float range_sq = (distance_ + radius + anchor_radius_)*(distance_ + radius + anchor_radius_);
    if (dist_sq > range_sq)
    {
        return 2;
    }

    //是否双方半径相交或者重叠
    f32 radius_sq = (radius + anchor_radius_)*(radius + anchor_radius_);
    if (collide_test_ && dist_sq < radius_sq)
    {
        return 0;
    }

    //二维凸多边形的点包含检测 判断目标半径 
    f32 min_shorted_line = distance_;
    for (size_t i = 0; i < VERTEX_SIZE; i++)
    {
        const Point3& line = lines_[i];
        Point3 test_side(pos.x - vertexs_[i].x, pos.y - vertexs_[i].y, 0.0f);
        if (test_side.is_zero())
        {
            return 0;
        }
        Point3 cross = line.cross(test_side);
        if (cross.z < 0)
        {
            if (radius < FLOAT_POINT_PRECISION)
            {
                return 2;
            }
            //在凸多边形外部 需要检测下碰撞圆是否全部在外侧  
            f32 shorted_line = cross.length() * line_inv_length_[i];
            if (shorted_line > radius)
            {
                return 3;
            }
            else
            {
                if (min_shorted_line > shorted_line)
                {
                    min_shorted_line = shorted_line;
                }
            }
        }
        else if (frame_test_)
        {
            f32 shorted_line = cross.length() * line_inv_length_[i];
            if (min_shorted_line > shorted_line)
            {
                min_shorted_line = shorted_line;
            }
        }
    }
    if (frame_test_)
    {
        if (min_shorted_line > radius)
        {
            return 4;
        }
    }
    return 0;
}



s32 AreaShapeFan::Init(DeviationShape deviation, f32 radius)
{
    if (deviation.pivot_scale.x < FLOAT_POINT_PRECISION || deviation.pivot_scale.y < FLOAT_POINT_PRECISION)
    {
        LOGFMTE("config error. param1:<%f>, param2:<%f>", deviation.pivot_scale.x, deviation.pivot_scale.y);
        return -2;
    }

    deviation.pivot_scale.x = std::fmod(deviation.pivot_scale.x, 360.0f);
    if (deviation.pivot_scale.x < FLOAT_POINT_PRECISION)
    {
        deviation.pivot_scale.x = 360.0f;
    }

    anchor_ = deviation.pivot_pos;
    anchor_radius_ = radius;
    radian_ = deviation.pivot_scale.x / 360.0f * PI;
    radian_radius_ = deviation.pivot_scale.y;
    high_ = deviation.pivot_scale.z;
    is_circle_ = deviation.pivot_scale.x > 360.0f * 0.8f ? true : false; //超过一定度数(接近360度)则认为是一个圆
    if (!is_circle_)
    {
        radian_domain_ = cos(radian_);
        normalize_dir_ = deviation.pivot_dir;
        if (normalize_dir_.is_zero())
        {
            LOGFMTE("dir zero");
            return -5;
        }
        if (!normalize_dir_.normalize())
        {
            LOGFMTE("dir normalize false");
            return -6;
        }
    }
    return 0;
}


s32 AreaShapeFan::PointInRange(const Point3& pos, f32 radius, f32 & dist_sq)
{

    //高度差检测
    using std::max;
    float height_dist = fabsf(pos.z - anchor_.z);
    if (height_dist > high_)
    {
        LOGFMTD("range fan too high anchor z:<%f>, test z:<%f>, limit high:<%f>", anchor_.z, pos.z, high_);
        return 1;
    }

    Point3 test_line = Point3(pos.x - anchor_.x, pos.y - anchor_.y, 0.0f);
    //距离太远直接筛掉
    //这里相当于把区域的长度延长了两个碰撞圆的半径 
    dist_sq = test_line.square_length();
    float range_sq = (radian_radius_ + radius + anchor_radius_) * (radian_radius_ + radius + anchor_radius_);
    if (dist_sq > range_sq)
    {
        LOGFMTD("range too far dist_sq:<%f>", dist_sq);
        return 2;
    }

    //距离小于等于两个点的半径和直接通过, 这个对下面的向量公式计算做了值域保证 
    float radius_sq = (radius + anchor_radius_) * (radius + anchor_radius_);
    if (dist_sq < FLOAT_POINT_PRECISION || dist_sq <= radius_sq)
    {
        return 0;
    }

    if (!is_circle_)
    {
        f32 radian_domain = test_line.dot(normalize_dir_) * Point3::INVERSE_SQRT(dist_sq); //值域总是在[-1,+1], angle[180, 0]
        if (radian_domain  > radian_domain_)
        {
            return 0;
        }

        //判断目标半径是否相交比较浪费性能 如果目标圆太小最好直接忽略
        if (radius < FLOAT_POINT_PRECISION)
        {
            LOGFMTD("range over radian. cur %f,  fan %f", acos(radian_domain) / 2.0f / 3.14f*360.0f, radian_ / 2.0f / 3.14f*360.0f);
            return 3;
        }

        f32 radian_add_domain = radius * radius / 2.0f / dist_sq;
        if (acos(radian_domain) < acos(1 - radian_add_domain) + radian_)
        {
            return 0;
        }
        LOGFMTD("range over radian. cur %f,  fan %f", acos(radian_domain) / 2.0f / 3.14f*360.0f, radian_ / 2.0f / 3.14f*360.0f);
        return 4;
    }
    return 0;
}




s32 AreaShapeCircle::Init(DeviationShape deviation, f32 radius)
{
    anchor_ = deviation.pivot_pos;
    anchor_radius_ = radius;
    min_radius_sq_ = deviation.pivot_scale.x * deviation.pivot_scale.x;
    max_radius_ = deviation.pivot_scale.y;
    high_ = deviation.pivot_scale.z;
    return 0;
}


s32 AreaShapeCircle::PointInRange(const Point3& pos, f32 radius, f32 & dist_sq)
{
    //高度差检测
    using std::max;
    float height_dist = fabsf(pos.z - anchor_.z);
    if (height_dist > high_)
    {
        LOGFMTD("range circle too high anchor z:<%f>, test z:<%f>, limit high:<%f>", anchor_.z, pos.z, high_);
        return 1;
    }

    Point3 test_line = Point3(pos.x - anchor_.x, pos.y - anchor_.y, 0.0f);
    dist_sq = test_line.square_length();
    float range_sq = (max_radius_ + radius + anchor_radius_) * (max_radius_ + radius + anchor_radius_);
    if (dist_sq > range_sq)
    {
        LOGFMTD("range too far dist_sq:<%f>", dist_sq);
        return 2;
    }
    if (min_radius_sq_ < FLOAT_POINT_PRECISION)
    {
        return 0;
    }
    if (dist_sq < min_radius_sq_)
    {
        LOGFMTD("range in ring dist_sq:<%f>", dist_sq);
        return 3;
    }
    return 0;
}


AreaShape::AreaShape()
{
    shape_type_ = -1;
}

s32 AreaShape::Init(u32 shape_type, DeviationShape deviation, f32 radius)
{
    deviation.pivot_dir.z = 0;
    LOGFMTD("range init: shape_type:<%u>, radius:<%f>, "
        "deviation.pivot_pos:<%f,%f,%f>, deviation.pivot_dir:<%f,%f,%f>, param:<%f,%f,%f>",
        shape_type, radius, deviation.pivot_pos.x, deviation.pivot_pos.y, deviation.pivot_pos.z,
        deviation.pivot_dir.x, deviation.pivot_dir.y, deviation.pivot_dir.z, 
        deviation.pivot_scale.x, deviation.pivot_scale.y, deviation.pivot_scale.z);
    if (-1 != shape_type_)
    {
        LOGFMTE("range init conflict: shape_type old:<%u>, now:<%u>", shape_type_, shape_type);
        return -1;
    }
    if (deviation.pivot_dir.is_zero())
    {
        LOGFMTE("range init param error. shape_type:<%u>, deviation.pivot_dir:<%f,%f,%f>", 
            shape_type, deviation.pivot_dir.x, deviation.pivot_dir.y, deviation.pivot_dir.z);
        return -2;
    }
    
    s32 ret = 0;
    switch (shape_type)
    {
    case AREA_SHAPE_CIRCLE:
        deviation.pivot_scale.x = 0.0f;
        ret = circle_.Init(deviation, 0.0f);
        break;
    case AREA_SHAPE_FAN:
        ret = fan_.Init(deviation, radius);
        break;
    case AREA_SHAPE_RECT:
        ret = rect_.Init(deviation, radius, true, false);
        break;
    case AREA_SHAPE_RING:
        ret = circle_.Init(deviation, radius);
        break;
    case AREA_SHAPE_FRAME:
        ret = rect_.Init(deviation, radius, true, true);
        break;
    default:
        LOGFMTE("unknown range shape_type:<%u>", shape_type);
        return -5;
    }
    if (ret != 0)
    {
        LOGFMTE("range init shape error:<%d>, shape_type:<%u>", ret, shape_type);
        return ret;
    }
    shape_type_ = shape_type;
    return 0;
}

s32 AreaShape::PointInRange(const Point3& pos, f32 radius, f32& dist_sq)
{
    s32 ret = 0;
    if (-1 == shape_type_)
    {
        LOGFMTE("point in range test error. not init.");
        return -1;
    }

    switch (shape_type_)
    {
    case AREA_SHAPE_CIRCLE:
        ret = circle_.PointInRange(pos, radius, dist_sq);
        break;
    case AREA_SHAPE_FAN:
        ret = fan_.PointInRange(pos, radius, dist_sq);
        break;
    case AREA_SHAPE_RECT:
        ret = rect_.PointInRange(pos, radius, dist_sq);
        break;
    case AREA_SHAPE_RING:
        ret = circle_.PointInRange(pos, radius, dist_sq);
        break;
    case AREA_SHAPE_FRAME:
        ret = rect_.PointInRange(pos, radius, dist_sq);
        break;
    default:
        LOGFMTE("point in range test error. range not init. pos:<%f,%f,%f>, radius:<%f>, ret:<%d>, shape_type_:<%u>", 
            pos.x, pos.y, pos.z, radius, ret, shape_type_);
        return -4;
    }
    if (ret < 0)
    {
        LOGFMTE("point in range test error:<%d>. pos:<%f,%f,%f>, radius:<%f>, ret:<%d>, shape_type_:<%u>", 
            ret, pos.x, pos.y, pos.z, radius, ret, shape_type_);
        return ret;
    }

    return ret;
}







