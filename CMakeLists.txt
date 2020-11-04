cmake_minimum_required(VERSION 3.7)
project(range_scan)
enable_language(ASM)

#from github.com/zsummer

# plat : WIN32 APPLE UNIX    (UNIX contain UNIX like)

# CMAKE_CXX_COMPILER_ID:  GNU Intel Clang AppleClang MSVC 
# example IF (CMAKE_CXX_COMPILER_ID MATCHES "Clang") ENDIF()

# jump compiler works check
# if(WIN32)
#    set(CMAKE_C_COMPILER_WORKS TRUE)
#    set(CMAKE_CXX_COMPILER_WORKS TRUE)
# endif(WIN32)

# jump this project build when msvc 
# set_target_properties(${PROJECT_NAME} PROPERTIES EXCLUDE_FROM_ALL 1 EXCLUDE_FROM_DEFAULT_BUILD 1)

# set this project setup build when msvc 
# set_property(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY VS_STARTUP_PROJECT ${PROJECT_NAME})

# show msvc folder
#  SET_PROPERTY(GLOBAL PROPERTY USE_FOLDERS ON) 

# use C++ 14
# set(CMAKE_CXX_FLAGS -std=c++14) 

# 
# CMAKE_SOURCE_DIR   cmake root dir 
# CMAKE_CURRENT_SOURCE_DIR current cmakelist.txt dir  
# EXECUTABLE_OUTPUT_PATH can set it change bin out dir
# CMAKE_MODULE_PATH can set it change module dir 
# PROJECT_NAME cur project name 

# include 
# include_directories  
# link_directories 
# link_libraries 

# 
# execute_process

#分组添加到IDE
macro(GROUP_SRC_BY_DIR FILES)
    foreach(PATH_NAME ${${FILES}})
        string(REGEX REPLACE ${CMAKE_CURRENT_SOURCE_DIR}/\(.*\) \\1 RELATIVE_PATH ${PATH_NAME})
        string(REGEX MATCH "(.+)/.*" HAS_GROUP ${RELATIVE_PATH})
        if(HAS_GROUP)
                string(REGEX REPLACE "(.+)/.*" \\1 GROUP_NAME ${RELATIVE_PATH})
                string(REPLACE "/" "\\" GROUP_NAME ${GROUP_NAME})
                message(${PATH_NAME} " into " ${GROUP_NAME})
                source_group(${GROUP_NAME} FILES ${PATH_NAME})
        else()
                source_group("" FILES ${PATH_NAME})
        endif()
    endforeach(PATH_NAME)
endmacro(GROUP_SRC_BY_DIR)


FILE(GLOB_RECURSE SOURCES_H ${CMAKE_SOURCE_DIR}/*.h ${CMAKE_SOURCE_DIR}/*.hpp)
FILE(GLOB_RECURSE SOURCES_C ${CMAKE_SOURCE_DIR}/*.c ${CMAKE_SOURCE_DIR}/*.cpp ${CMAKE_SOURCE_DIR}/*.S)

# 自动INCLUDE所有头文件的目录  (可能会影响到include的优先级)
# list(FILTER <list> <INCLUDE|EXCLUDE> REGEX <regular_expression>)  
LIST(FILTER SOURCES_H EXCLUDE REGEX "build")
LIST(FILTER SOURCES_C EXCLUDE REGEX "build")
set(SOURCES ${SOURCES_H} ${SOURCES_C})
GROUP_SRC_BY_DIR(SOURCES)

FOREACH (FILE_NAME ${SOURCES})
    MESSAGE(${FILE_NAME})
endforeach()

FOREACH(FILE_NAME ${SOURCES_H})
        STRING( REGEX REPLACE "[^/\\\\]+$" " " DIR_NAME ${FILE_NAME} )
        LIST(APPEND DIR_NAME_LIST ${DIR_NAME})
ENDFOREACH()
LIST(REMOVE_DUPLICATES DIR_NAME_LIST)
FOREACH(DIR_NAME ${DIR_NAME_LIST})
        MESSAGE("auto include " ${DIR_NAME} )
        include_directories(${DIR_NAME})
ENDFOREACH()

#通用设置部分 包括启用分组 设置 启动项目  
if(WIN32)
    ADD_DEFINITIONS(-DWIN32 -W3)
    SET_PROPERTY(GLOBAL PROPERTY USE_FOLDERS ON) 
    set_property(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY VS_STARTUP_PROJECT ${PROJECT_NAME})
else()
    ADD_DEFINITIONS(-Wall -g -ggdb -O0 )
    set(CMAKE_CXX_FLAGS -std=c++14)
    link_libraries(pthread m c)
endif()

#输出
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)


#自定义部分 
ADD_DEFINITIONS(-DGLFW_USE_CONFIG_H -D_GLFW_WIN32 -DGLM_FORCE_LEFT_HANDED)

add_executable("${PROJECT_NAME}" ${SOURCES})



