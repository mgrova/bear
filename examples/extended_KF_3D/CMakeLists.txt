##---------------------------------------------------------------------------------------------------------------------
##  BEAR
##---------------------------------------------------------------------------------------------------------------------
##  Copyright 2019 ViGUS University of Seville
##---------------------------------------------------------------------------------------------------------------------
##  Permission is hereby granted, free of charge, to any person obtaining a copy of this software
##  and associated documentation files (the "Software"), to deal in the Software without restriction,
##  including without limitation the rights to use, copy, modify, merge, publish, distribute,
##  sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is
##  furnished to do so, subject to the following conditions:
##
##  The above copyright notice and this permission notice shall be included in all copies or substantial
##  portions of the Software.
##
##  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING
##  BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
##  NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES
##  OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
##  CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
##---------------------------------------------------------------------------------------------------------------------
cmake_minimum_required (VERSION 3.12 FATAL_ERROR)
project(extended_KF_3D)

#########################################
###### Loading 3rd party libraries.######
#########################################

#########################################
## Load Eigen
find_package(Eigen3 REQUIRED)
if(${Eigen3_FOUND})
	MESSAGE( STATUS "Eigen FOUND")
endif()

#########################################
## Compiler specific options
set(REMOTE_CONTROLLER_COMPILE_OPTIONS "-Wall -g -O0")

## Collect all sources
file(GLOB_RECURSE SRC_FILES "src/*.cpp")
file(GLOB_RECURSE HDR_FILES "include/*.h")

## Add Executable
add_executable(${PROJECT_NAME} ${SRC_FILES} ${HDR_FILES})
include_directories(${PROJECT_NAME} PRIVATE ${PROJECT_SOURCE_DIR}/include)

string(REPLACE " " ";" REMOTE_CONTROLLER_COMPILE_OPTIONS_REPLACED_FLAGS ${REMOTE_CONTROLLER_COMPILE_OPTIONS})

target_compile_options(${PROJECT_NAME} PUBLIC ${REMOTE_CONTROLLER_COMPILE_OPTIONS_REPLACED_FLAGS})
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_11)

target_include_directories(${PROJECT_NAME} PUBLIC "/usr/include/eigen3")