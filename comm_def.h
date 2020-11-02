﻿/*
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

#ifndef COMM_DEF_H
#define COMM_DEF_H

#include <stdlib.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

#include <limits>
#include <string>
#include <vector>
#include <map>
#include <memory>
#include <list>
#include <array>
#define USE_LOG4Z_FORMAT
#include "fn_log.h"

typedef char s8;
typedef unsigned char u8;
typedef short int s16;
typedef unsigned short int u16;
typedef int s32;
typedef unsigned int u32;
typedef long long s64;
typedef unsigned long long u64;
typedef unsigned long pointer;
typedef unsigned int qq_t;
typedef int BOOL;
typedef float f32;

inline void empty_test(...) {}





#define Now() std::chrono::duration<double>(std::chrono::system_clock().now().time_since_epoch()).count()











#endif // COMM_DEF_H