#pragma once
#ifndef AOE_COMMON_H
#define AOE_COMMON_H
#include "comm_def.h"
#include <array>
#include "vector3.h"
#include "zarray.h"
using namespace zsummer::shm_arena;
using Point3 = Vector3<float>;



struct Rotation
{
    f32 x;
    f32 y;
    f32 z;
    f32 w;
};
const Rotation ROTATION_ZERO = { 0.0f, 0.0f, 0.0f, 0.0f };

struct SpellCastParam
{
    Point3 cast_pos;
    Point3 cast_dir;
    Point3 throw_pos;
    Point3 move_pos;

    u64 target_id;
    Point3 cast_target_pos;
    float cast_target_face;
    
    Rotation rotation;
};


//搜索到的目标
struct RangeTarget
{
    u64 role_id;
    f32 dist_sq; //该role与定位点的距离 
    s32 hp;
};

inline bool RangeTargetSortLeastDist(const RangeTarget& rt1, const RangeTarget& rt2) { return rt1.dist_sq < rt2.dist_sq; }
inline bool RangeTargetSortFarthestDist(const RangeTarget& rt1, const RangeTarget& rt2) { return rt1.dist_sq > rt2.dist_sq; }
inline bool RangeTargetSortLeastHP(const RangeTarget& rt1, const RangeTarget& rt2) { return rt1.hp < rt2.hp; }
inline bool RangeTargetSortMostHP(const RangeTarget& rt1, const RangeTarget& rt2) { return rt1.hp > rt2.hp; }


struct DeviationShape
{
    Point3 pivot_pos; //轴点坐标, 三组数据用来查看服务器选定目标的区域形状参数, 配置无效的情况下这三组数据同样无效.
    Point3 pivot_dir; //朝向.
    Point3 pivot_scale; //长宽高,弧度半径高, 内径外径高
};
#define MAX_DEVIATION_SHAPE 5
using DeviationShapeArray = zarray<DeviationShape, MAX_DEVIATION_SHAPE>;



//目标判定方式
inline u32 PEEK_RANGE_TYPE(u32 specify) { return specify / 10000; }
inline u32 PEEK_RANGE_SHAPE(u32 specify) { return specify / 1000 % 10; }
inline u32 PEEK_RANGE_POINT_REFERENCE(u32 specify) { return specify / 100 % 10; }
inline u32 PEEK_RANGE_DIR(u32 specify) { return specify / 10 % 10; }
inline u32 PEEK_RANGE_SORT(u32 specify) { return specify % 10; }



#endif // 
