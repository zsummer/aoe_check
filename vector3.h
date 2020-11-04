/*
* vector3 License
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

#include <math.h>
#include <cmath>

static const float PI = 3.1415926535897932f;
static const float PI_PER_ANGLE = PI / 180.0f;
static const float ANGLE_PER_PI = 180.0f / PI;
static const float FLOAT_POINT_PRECISION = 0.0002f;


#define SignBitF(f) ((*(u32*)&f) & (1u << 31))

template<class Float>
static inline bool FLOAT_IS_ZERO(Float val) { return fabs(val) < FLOAT_POINT_PRECISION; }
template<class Float>
static inline bool FLOAT_IS_EQUAL(Float val1, Float val2) { return fabs(val1 - val2) < FLOAT_POINT_PRECISION; }
template<class Float>
static inline Float PRUNING(Float v, Float min, Float max) { return v > max ? max : (v < min ? min : v); }





template<class Float>
struct Vector3
{
public:

    static inline Float INVERSE_SQRT(Float val)
    {
        float xhalf = 0.5f * val;
        int i = *(int*)& val;
        i = 0x5f3759df - (i >> 1);
        val = *(float*)& i;
        val = val * (1.5f - xhalf * val * val);
        return val;
    }

public:
    Float x, y, z;
    Vector3(Float fx, Float fy, Float fz):x(fx),y(fy),z(fz){}
    Vector3(const Vector3 & ) = default;
    Vector3() :Vector3(0.0f, 0.0f, 0.0f) {};
    Vector3 & operator =(const Vector3 & v3) = default;

    void reset() { x = 0.0f; y = 0.0f; z = 0.0f; }
    Float dot(const Vector3& v) const { return x * v.x + y * v.y + z * v.z; };
    Vector3 det(const Vector3& v) const { return { y * v.z - z * v.y , z * v.x - x * v.z , x * v.y - y * v.x }; };
    Vector3 cross(const Vector3& v) const { return det(v); }
    Float square_length()const { return dot(*this); }
    Float length()const { return sqrtf(square_length()); }
    bool is_zero() const { return FLOAT_IS_ZERO(x) && FLOAT_IS_ZERO(y) && FLOAT_IS_ZERO(z); }
    bool is_valid() const{return !std::isnan(x) && !std::isnan(y) && !std::isnan(z) && !std::isinf(x) && !std::isinf(y) && !std::isinf(z);}
    //单位化
    bool normalize()
    {
        Float square = square_length();
        if (FLOAT_IS_ZERO(square))
        {
            return false;
        }
        *this /= sqrt(square);
        return true;
    }

    bool from_angle(Float angle)
    {
        float radian = angle * PI_PER_ANGLE;
        x = cos(radian);
        y = sin(radian);
        z = 0.0f;
        return true;
    }


    Float to_agnle() const
    {
        Float radian = dot({ 1.0f, 0.0f, 0.0f });
        if(y < 0.0f)
        {
            return (PI * 2 - radian) * ANGLE_PER_PI;
        }
        return radian * ANGLE_PER_PI;
    }

    bool from_uv(Float u, Float v)
    {
        x = sin(PI * v) * cos(PI2 * u);
        y = sin(PI * v) * sin(PI2 * u);
        z = cos(PI * v);
        return true;
    }
    static Vector3<Float> new_from_uv(Float u, Float v)
    {
        return Vector3<Float>(sin(PI * v) * cos(PI * 2 * u), sin(PI * v) * sin(PI * 2 * u), cos(PI * v));
    }
    static Vector3<Float> new_from_uv2(Float u, Float v)
    {
        return Vector3<Float>(cos(PI * v) * cos(PI * u), sin(PI * v) * cos(PI * u), sin(PI * v));
    }
    inline bool operator == (const Vector3 &v) const { return FLOAT_IS_EQUAL(x, v.x) && FLOAT_IS_EQUAL(y, v.y) && FLOAT_IS_EQUAL(z, v.z); }
    inline bool operator != (const Vector3 &v) const { return !(*this == v); }

    Vector3 operator + (const Vector3 & v) const { return { x + v.x, y + v.y, z + v.z }; }
    Vector3 & operator += (const Vector3 & v) { x += v.x, y += v.y, z += v.z; return *this; }
    Vector3 operator - (const Vector3 & v) const { return { x - v.x, y - v.y, z - v.z }; }
    Vector3 & operator -= (const Vector3 & v) { x -= v.x, y -= v.y, z -= v.z; return *this; }
    Vector3 operator * (const Vector3 & v) const { return { x * v.x, y * v.y, z * v.z }; }
    Vector3 & operator *= (const Vector3 & v) { x *= v.x, y *= v.y, z *= v.z; return *this; }
    Vector3 operator / (const Vector3 & v) const { return { x / v.x, y / v.y, z / v.z }; }
    Vector3 & operator /= (const Vector3 & v) { x /= v.x, y /= v.y, z /= v.z; return *this; }

    Vector3 operator + (Float scalar) const { return { x + scalar, y + scalar, z + scalar }; }
    Vector3 & operator += (Float scalar) { x += scalar, y += scalar, z += scalar; return *this; }
    Vector3 operator - (Float scalar) const { return { x - scalar, y - scalar, z - scalar }; }
    Vector3 & operator -= (Float scalar) { x -= scalar, y -= scalar, z -= scalar; return *this; }
    Vector3 operator * (Float scalar) const { return { x * scalar, y * scalar, z * scalar }; }
    Vector3 & operator *= (Float scalar) { x *= scalar, y *= scalar, z *= scalar; return *this; }
    Vector3 operator / (Float scalar) const { return { x / scalar, y / scalar, z / scalar }; }
    Vector3 & operator /= (Float scalar) { x /= scalar, y /= scalar, z /= scalar; return *this; }
};



static const Vector3<float> FLOAT_UNIT_X = { 1.0f, 0.0f, 0.0f };
static const Vector3<float> FLOAT_UNIT_Y = { 0.0f, 1.0f, 0.0f };
static const Vector3<float> FLOAT_UNIT_Z = { 0.0f, 0.0f, 1.0f };