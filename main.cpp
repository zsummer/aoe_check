/*
* breeze License
* Copyright (C) 2014-2017 YaweiZhang <yawei.zhang@foxmail.com>.
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


#pragma warning(disable:4996)
#pragma warning(disable:4819)


#define WIN32_LEAN_AND_MEAN

#include <WS2tcpip.h>
#include <WinSock2.h>
#include <windows.h>
#include <io.h>
#include <shlwapi.h>
#include <process.h>
#include <direct.h>
#include <glad/glad.h>
#include <gl/GL.h>


#include <iomanip>
#include <string.h>
#include <signal.h>
#include <time.h>
#include <cstdint>

#include <iostream>
#include <fstream>
#include <sstream>
#include <utility>
#include <algorithm>
#include <limits>

#include <functional>
#include <memory>
#include <unordered_map>
#include <chrono>
#include <random>

#include <string>
#include <set>
#include <vector>
#include <list>
#include <map>
#include <array>


#define USE_LOG4Z_FORMAT
#include "fn_log.h"


#include <vector>
#include <glad/glad.h>
#include <gl/GL.h>
#include <gl/GLU.h>

#include "glfw3.h"
#include "glfw3native.h"
#include "linmath.h"

#include <stdlib.h>
#include <stdio.h>


#pragma comment(lib, "OpenGL32")
#pragma comment(lib, "GLu32")
#include "glm/glm.hpp"
#include "glm/matrix.hpp"
#include "glm/gtc/matrix_transform.hpp"


#include "aoe_shape.h"
#define SCREEN_X 800
#define SCREEN_Y 800
#define BENCH_MARK false
static const int TEST_RANGE_SIZE = 4;
static const int SCALAR_NUM = 600;
static const int SCALAR_BEGIN = SCALAR_NUM / -2;
static const int SCALAR_END = SCALAR_NUM / 2;
static const int PIXELS_SIZE = SCALAR_NUM * SCALAR_NUM * TEST_RANGE_SIZE;

static void error_callback(int error, const char* description)
{
    fprintf(stderr, "Error: %s\n", description);
}

static void key_callback(GLFWwindow* window, int key, int scancode, int action, int mods)
{
    if (key == GLFW_KEY_ESCAPE && action == GLFW_PRESS)
        glfwSetWindowShouldClose(window, GLFW_TRUE);
}

struct Pixel
{
    double r;
    double g;
    double b;

    double x;
    double y;
    double z;
};

class Shape 
{
public:
    Shape(u32 s, f32 r, Point3 os, Point3 sr, Point3 e)
    {
        specify = s;
        redius = r;
        offset = os;
        scalar = sr;
        ext = e;
    }
    u32 specify;
    f32 redius;
    Point3 scalar;
    Point3 offset;
    Point3 ext;
};

std::vector<Shape> g_shapes =
{
{AREA_SHAPE_FOV, 0.02f, {40.0f, 0.0f, 40.0f}, { 120.0f, 0.8f, 0.5f }, {0.1f, 0.0f, 0.0f} },
{AREA_SHAPE_FOV, 0.02f, {40.0f, 0.0f, 20.0f}, { 120.0f, 0.8f, 0.4f }, {0.1f, 0.0f, 0.0f} },
{AREA_SHAPE_FOV, 0.02f, {0.0f, 0.0f, 0.0f}, { 90.0f, 0.5f, 0.4f }, {0.3f, 0.0f, 0.0f} },
{AREA_SHAPE_FOV, 0.02f, {0.0f, 0.0f, 0.0f}, { 30.0f, 0.5f, 0.4f }, {0.3f, 0.0f, 0.0f} },
{AREA_SHAPE_FOV, 0.02f, {0.0f, 0.0f, 0.0f}, { 30.0f, 0.5f, 0.4f }, {0.15f, 0.0f, 0.0f} },
{AREA_SHAPE_FOV, 0.02f, {0.0f, 0.0f, 0.0f}, { 30.0f, 1.5f, 0.4f }, {0.15f, 0.0f, 0.0f} },
{AREA_SHAPE_CIRCLE,0.02f, {0.0f, 0.0f, 0.0f}, { 0.0f, 0.2f, 0.2f }, {0.0f, 0.0f, 0.0f} },
{AREA_SHAPE_CIRCLE,0.02f,{0.0f, 0.0f, 0.0f}, { 0.1f, 0.2f, 0.2f }, {0.0f, 0.0f, 0.0f} },
{AREA_SHAPE_FAN,0.02f,{0.0f, 0.0f, 0.0f}, { 180.0f, 0.2f, 0.2f }, {0.0f, 0.0f, 0.0f} },
{AREA_SHAPE_FAN,0.00f,{0.0f, 0.0f, 0.0f}, { 120.0f, 0.2f, 0.2f }, {0.0f, 0.0f, 0.0f} },
{AREA_SHAPE_FAN,0.02f,{0.0f, 0.0f, 0.0f}, { 60.0f, 0.2f, 0.2f }, {0.0f, 0.0f, 0.0f} },
{AREA_SHAPE_FAN,0.00f,{0.0f, 0.0f, 0.0f}, { 60.0f, 0.2f, 0.2f }, {0.0f, 0.0f, 0.0f} },
{AREA_SHAPE_RECT,0.00f,{0.0f, 0.0f, 0.0f}, { 0.2f, 0.2f, 0.2f }, {0.0f, 0.0f, 0.0f} },
{AREA_SHAPE_RECT,0.02f,{0.0f, 0.0f, 0.0f}, { 0.2f, 0.2f, 0.2f }, {0.0f, 0.0f, 0.0f} },
{AREA_SHAPE_RING,0.00f,{0.0f, 0.0f, 0.0f}, { 0.2f, 0.2f, 0.2f }, {0.0f, 0.0f, 0.0f} },
{AREA_SHAPE_RING,0.02f,{0.0f, 0.0f, 0.0f}, { 0.2f, 0.2f, 0.2f }, {0.0f, 0.0f, 0.0f} },
{AREA_SHAPE_RING,0.02f,{0.0f, 0.0f, 0.0f}, { 0.0f, 0.2f, 0.2f }, {0.0f, 0.0f, 0.0f} },
{AREA_SHAPE_FRAME,0.02f,{0.0f, 0.0f, 0.0f}, { 0.2f, 0.2f, 0.2f }, {0.0f, 0.0f, 0.0f} },
{AREA_SHAPE_FRAME,0.00f,{0.0f, 0.0f, 0.0f}, { 0.2f, 0.2f, 0.2f }, {0.0f, 0.0f, 0.0f} }
};

class TestRange
{
public:
    TestRange(Point3 pos, Point3 color1, Point3 color2):_pos(pos), _color_redius(color1), _color(color2) {}
    ~TestRange() {}
public:

    void Test(Point3 dir)
    {
        time_t now = time(NULL);
        if (_last == 0)
        {
            _last = now;
        }
        if (now - _last > 2)
        {
            _last = now;
            _cur_specify++;
            //_cur_specify %= 1;
            _cur_specify %= g_shapes.size();

        }
        if (_cur_locked_specify != 0)
        {
            _cur_specify = _cur_locked_specify;
        }

        AreaShape range;
        Point3 target = _pos + dir * g_shapes[_cur_specify].scalar.y;
        s32 ret = range.Init(g_shapes[_cur_specify].specify, { _pos, dir, g_shapes[_cur_specify].offset, g_shapes[_cur_specify].scalar, g_shapes[_cur_specify].ext}, g_shapes[_cur_specify].redius);
        if (ret != 0)
        {
            LOGFMTE("error");
            return;
        }
        
        f32 dist = 0;
        Point3 pos;

        for (int i = SCALAR_BEGIN; i < SCALAR_END; i++)
        {
            for (int j = SCALAR_BEGIN; j < SCALAR_END; j++)
            {
                pos = { i*2.0f / SCALAR_NUM, j * 2.0f / SCALAR_NUM, 0.0f};
                if (0 == range.PointInRange(pos, 0.01, dist))
                {
                    if (_len < PIXELS_SIZE)
                    {
                        _pixels[_len++] = { _color_redius.x, _color_redius.y, _color_redius.z, pos.x, pos.y, pos.z };
                    }
                }
                if (0 == range.PointInRange(pos, 0.0f, dist))
                {
                    if (_len < PIXELS_SIZE)
                    {
                        _pixels[_len++] = { _color.x, _color.y, _color.z, pos.x, pos.y, pos.z };
                    }
                }
            }
        }
        if (_len < 10)
        {
            int a = 1;
            a++;
            (void)a;
        }

        if (_len < PIXELS_SIZE + 200)
        {
            for (int i = -5; i < 5; i++)
            {
                for (int j = -5; j < 5; j++)
                {
                    _pixels[_len++] = { 1.0f, 0.0f, 0.0f, _pos.x + i * 0.002f, _pos.y + j * 0.002f, _pos.z };
                    _pixels[_len++] = { 1.0f, 0.0f, 0.0f, target.x + i * 0.002f, target.y + j * 0.002f, target.z };
                }
            }
        }
    }

public:
    std::array<Pixel, PIXELS_SIZE>  _pixels;
    size_t _len = 0;
    time_t _last = 0;
    size_t _cur_specify = 0;
    size_t _cur_locked_specify = 0;
    Point3 _pos;
    Point3 _color_redius;
    Point3 _color;
};

TestRange g_tr[] =
{
    { { -0.5f, -0.5f, 0.0f }, { 0.0,1.0,1.0 }, { 1.0,1.0,1.0 }},
    { { 0.5f, -0.5f , 0.0f }, { 0.0,1.0,1.0 }, { 1.0,1.0,1.0 } },
    { { 0.5f, 0.5f  , 0.0f }, { 0.0,1.0,1.0 }, { 1.0,1.0,1.0 } },
    { { -0.5f, 0.5f , 0.0f }, { 0.0,1.0,1.0 }, { 1.0,1.0,1.0 } },
    { { 0.0f, 0.05f , 0.0f }, { 0.0,1.0,1.0 }, { 1.0,1.0,1.0 } }
};

inline void rcVcross(float* dest, const float* v1, const float* v2)
{
    dest[0] = v1[1] * v2[2] - v1[2] * v2[1];
    dest[1] = v1[2] * v2[0] - v1[0] * v2[2];
    dest[2] = v1[0] * v2[1] - v1[1] * v2[0];
}

inline float rcVdot(const float* v1, const float* v2)
{
    return v1[0] * v2[0] + v1[1] * v2[1] + v1[2] * v2[2];
}
inline void rcVsub(float* dest, const float* v1, const float* v2)
{
    dest[0] = v1[0] - v2[0];
    dest[1] = v1[1] - v2[1];
    dest[2] = v1[2] - v2[2];
}
bool intersectSegmentTriangle(const float* sp, const float* sq,
    const float* a, const float* b, const float* c,
    float& t)
{
    float v, w;
    float ab[3], ac[3], qp[3], ap[3], norm[3], e[3];
    rcVsub(ab, b, a);
    rcVsub(ac, c, a);
    rcVsub(qp, sp, sq);

    // Compute triangle normal. Can be precalculated or cached if
    // intersecting multiple segments against the same triangle
    rcVcross(norm, ab, ac);

    // Compute denominator d. If d <= 0, segment is parallel to or points
    // away from triangle, so exit early
    float d = rcVdot(qp, norm);
    if (d <= 0.0f) return false;

    // Compute intersection t value of pq with plane of triangle. A ray
    // intersects iff 0 <= t. Segment intersects iff 0 <= t <= 1. Delay
    // dividing by d until intersection has been found to pierce triangle
    rcVsub(ap, sp, a);
    t = rcVdot(ap, norm);
    if (t < 0.0f) return false;
    if (t > d) return false; // For segment; exclude this code line for a ray test

    // Compute barycentric coordinate components and test if within bounds
    rcVcross(e, qp, ap);
    v = rcVdot(ac, e);
    if (v < 0.0f || v > d) return false;
    w = -rcVdot(ab, e);
    if (w < 0.0f || v + w > d) return false;

    // Segment/ray intersects triangle. Perform delayed division
    t /= d;

    return true;
}


void stress_2d()
{
    float fov = 120.0f;
    float aspect = 0.8f;
    glm::mat4 mat(1.0f);
    glm::vec4 v_dir(-0.8f, 0.6f, 0.4f, 1.0f);
    auto v = v_dir *mat;
    mat = glm::rotate(mat, glm::radians(fov * aspect / 2.0f * 1.0f), glm::vec3(1.0f, 0.0f, 0.0f));
    v_dir = mat * v_dir;

    auto old = g_shapes[6];
    g_tr[0]._cur_locked_specify = 6;
    for (auto t : {AREA_SHAPE_CIRCLE,AREA_SHAPE_FAN,AREA_SHAPE_RECT,AREA_SHAPE_RING,AREA_SHAPE_FRAME, AREA_SHAPE_FOV })
    {
        g_shapes[6] = Shape(t, 0.02f, { 0.0f, 0.0f, 0.0f }, { 60.0f, 0.5f, 0.8f }, { 0.5f, 0.0f, 0.0f });
        double now = Now();
        g_tr[0].Test({ 0.0,1.0,1.0 });
        LogInfo() << "used time:" << (Now() - now) / ((SCALAR_END - SCALAR_BEGIN) * (SCALAR_END - SCALAR_BEGIN)) * 1000 * 10000;
    }
    g_shapes[6] = old;
    g_tr[0]._cur_locked_specify = 0;


    float a[3] = { -3.0f, 3.0f, 1.0f };
    float b[3] = { 3.0f, 3.0f, 1.0f };
    float c[3] = { 0.0f, -3.0f, 1.0f };
    float sp[3] = { 0.0f, 0.0f, 5.0f };
    float sq[3] = { 0.0f, 0.0f, -5.0f };
    float ret = 0.0f;
    double now = Now();
    float testz = -5.0f;
    float sum = .0f;
    for (size_t i = 0; i < 1000 * 10000; i++)
    {
        sp[0] = 0.0f + (rand() % 100) / 1000.0f;
        sp[2] = testz * -1.0f  + ((i%2) * -1 * testz * 2);
        sq[2] = sp[2] * -1.0f;
        intersectSegmentTriangle(sp, sq, a, b, c, ret);
        sum += ret;
    }
    LOGI("intersectSegmentTriangle used:" << (Now() - now)  << "s when test 1000*10000. " << sum);
}

int main(void)
{
#ifndef _WIN32
    //! linux下需要屏蔽的一些信号
    signal(SIGHUP, SIG_IGN);
    signal(SIGALRM, SIG_IGN);
    signal(SIGPIPE, SIG_IGN);
    signal(SIGXCPU, SIG_IGN);
    signal(SIGXFSZ, SIG_IGN);
    signal(SIGPROF, SIG_IGN);
    signal(SIGVTALRM, SIG_IGN);
    signal(SIGQUIT, SIG_IGN);
    signal(SIGCHLD, SIG_IGN);
    setenv("TZ", "GMT-8", 1);
#else
    //system("chcp 65001");
#endif
    srand((unsigned int)time(NULL));

    FNLog::FastStartDebugLogger();
    FNLog::BatchSetChannelConfig(FNLog::GetDefaultLogger(), FNLog::CHANNEL_CFG_PRIORITY, FNLog::PRIORITY_INFO);
    stress_2d();


    GLFWwindow * window;


    glfwSetErrorCallback(error_callback);

    if (!glfwInit())
        exit(EXIT_FAILURE);

    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 0);
    glfwWindowHint(GLFW_SAMPLES, 4);

    window = glfwCreateWindow(SCREEN_X, SCREEN_Y, "Simple example", NULL, NULL);
    if (!window)
    {
        glfwTerminate();
        exit(EXIT_FAILURE);
    }

    glfwSetKeyCallback(window, key_callback);

    glfwMakeContextCurrent(window);
    gladLoadGLLoader((GLADloadproc)glfwGetProcAddress);
    glfwSwapInterval(1);
    glEnable(GL_MULTISAMPLE);

    LOGI("GL_VERSION:" << (const char*)glGetString(GL_VERSION));
    for (size_t i = 0; i < TEST_RANGE_SIZE - 1; i++)
   {
       g_tr[i+1]._cur_specify = g_tr[i]._cur_specify + 1;
   }
    
    while (BENCH_MARK || !glfwWindowShouldClose(window))
    {
        if (!BENCH_MARK)
        {
            glClear(GL_COLOR_BUFFER_BIT);
            glShadeModel(GL_SMOOTH);
        }

        for (size_t i = 0; i < TEST_RANGE_SIZE; i++)
        {
            g_tr[i]._len = 0;
        }
        double begin_time = std::chrono::duration_cast<std::chrono::duration<double>>(std::chrono::steady_clock::now().time_since_epoch()).count();
        static double last_time = begin_time;
        static double frame_count = 0;
        static double last_hit_count = 0;
        static double cur_hit_count = 0;
        static double last_test_count = 0;
        static double cur_test_count = 0;
        frame_count++;
        Point3 dir = { 0.0f, 0.0f, 0.0f };
        double radian = (float)fmod(begin_time, 2.0f) / 2.0f * 3.141592654*2.0f;
        dir.x = cos(radian);
        dir.y = sin(radian);

        double begin_time2 = std::chrono::duration_cast<std::chrono::duration<double>>(std::chrono::steady_clock::now().time_since_epoch()).count();
        for (size_t i = 0; i < TEST_RANGE_SIZE; i++)
        {
            g_tr[i].Test(dir);
            cur_test_count += SCALAR_NUM * SCALAR_NUM;
        }
        double now = std::chrono::duration_cast<std::chrono::duration<double>>(std::chrono::steady_clock::now().time_since_epoch()).count();
        if (!BENCH_MARK)
        {
            glBegin(GL_POINTS);
        }
        
        for (size_t i = 0; i < TEST_RANGE_SIZE; i++)
        {
            for (size_t j = 0; j < g_tr[i]._len; j++)
            {
                if (!BENCH_MARK)
                {
                    glColor3f(g_tr[i]._pixels[j].r, g_tr[i]._pixels[j].g, g_tr[i]._pixels[j].b);
                    glVertex3f(g_tr[i]._pixels[j].x, g_tr[i]._pixels[j].y, g_tr[i]._pixels[j].z);
                }

            }
        }
        if (!BENCH_MARK)
        {

            glEnd();
            glfwSwapBuffers(window);
            glfwPollEvents();
        }


        for (size_t i = 0; i < TEST_RANGE_SIZE; i++)
        {
            cur_hit_count += g_tr[i]._len;
        }
        if (now - last_time  > 1.0f)
        {

            char title[100] = { 0 };
            sprintf(title, "specify type:<%u>, fps:<%lf>  lapse:<%lf>, test:<%lf>, hit:<%lf>", g_shapes[g_tr[0]._cur_specify].specify,
                frame_count / (now - last_time), now - begin_time,
                (cur_test_count - last_test_count) / (now - last_time),
                (cur_hit_count -last_hit_count) / (now - last_time));
            BENCH_MARK ? 0==0:glfwSetWindowTitle(window, title);
            LOGD(title);
            last_time = now;
            last_test_count = cur_test_count;
            last_hit_count = cur_hit_count;
            frame_count = 0.0f;
        }
    }

    glfwDestroyWindow(window);
    glGetError();
    glfwTerminate();
    exit(EXIT_SUCCESS);
}

void example()
{
    /* Draw a triangle */
    glBegin(GL_TRIANGLES);

    glColor3f(1.0f, 0.0f, 0.0f);    // Red
    glVertex3f(0.0f, 1.0f, 0.0f);

    glColor3f(0.0f, 1.0f, 0.0f);    // Green
    glVertex3f(-1.0f, -1.0f, 0.0f);

    glColor3f(0.0f, 0.0f, 1.0);    // Blue
    glVertex3f(1.0f, -1.0f, 0.0f);

    glEnd();


    //glEnable(GL_LINE_STIPPLE);
    //glLineStipple(1, 1);
    glColor3f(1.0f, 1.0f, 1.0f);    // Red
    glLineWidth(1);


    glBegin(GL_LINES);
    glVertex2f(0.0f, 0.0f);
    glColor3f(1.0f, 0.0f, 1.0f);    // Red
    glVertex2f(1, 1);
    glEnd();


    glBegin(GL_LINES);
    glVertex3f(0.0f, 1.0f, 0.0f);
    glColor3f(0.0f, 0.0f, 1.0f);    // Red
    glVertex3f(1, 0.0f, 1.0f);
    glEnd();


    glBegin(GL_POINTS);
    glColor3f(0.0f, 1.0f, 1.0f);
    glVertex3f(0.0f, 0.0f, 0.0f);
    glEnd();
}