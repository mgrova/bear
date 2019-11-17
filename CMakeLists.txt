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

cmake_minimum_required(VERSION 3.12 FATAL_ERROR)
PROJECT(bear VERSION 0.1 LANGUAGES C CXX)

#########################################
######			General config	   ######
#########################################

option(BUILD_EXAMPLES "Compile examples" ON)
option(BUILD_TESTS    "Prepare tests"    OFF)
option(BUILD_TOOLS    "Compile tools"    OFF)

#########################################
###### Loading 3rd party libraries ######
#########################################

if( NOT CMAKE_BUILD_TYPE )
  set( CMAKE_BUILD_TYPE Release CACHE STRING
       "Choose the type of build, options are: None Debug Release RelWithDebInfo MinSizeRel."
       FORCE )
endif()

# set(CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake/modules/" ${CMAKE_MODULE_PATH})

list(APPEND BEAR_DEFINITIONS "")
set(BEAR_COMPILE_OPTIONS "-Wall -g -O0")

#########################################
## Load Eigen
find_package(Eigen3 REQUIRED)
if(${Eigen3_FOUND})
	MESSAGE( STATUS "Eigen FOUND")
	list(APPEND BEAR_DEFINITIONS "BEAR_USE_EIGEN3")
endif()

## Load OpenCV
find_package(OpenCV)
if(${OpenCV_FOUND})
	MESSAGE( STATUS "OpenCV FOUND")
	list(APPEND BEAR_DEFINITIONS "BEAR_USE_OpenCV")
endif()


#########################################
######			Library	   ######
#########################################
file(GLOB_RECURSE BEAR_HEADERS "include/*.h" "include/*.inl")
file(GLOB_RECURSE BEAR_SOURCES "src/*.cpp" "src/*.c")

add_library(${PROJECT_NAME} SHARED ${BEAR_SOURCES} ${BEAR_HEADERS})

target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_compile_definitions(${PROJECT_NAME} PUBLIC ${BEAR_DEFINITIONS})
string(REPLACE " " ";" BEAR_COMPILE_OPTIONS_REPLACED_FLAGS ${BEAR_COMPILE_OPTIONS})

target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_11)
set_target_properties(${PROJECT_NAME} PROPERTIES LINKER_LANGUAGE C)

if(OpenCV_FOUND)
  target_include_directories(${PROJECT_NAME} PUBLIC ${OpenCV_INCLUDE_DIRS})
  target_link_libraries(${PROJECT_NAME} LINK_PUBLIC ${OpenCV_LIBRARIES})
endif(OpenCV_FOUND)

if (Eigen3_FOUND)
  target_include_directories(${PROJECT_NAME} PUBLIC "/usr/include/eigen3")
endif(Eigen3_FOUND)

#########################################
######			Documentation	   ######
#########################################
# find_package(Doxygen)
# if(DOXYGEN_FOUND)
# 	configure_file(${CMAKE_CURRENT_SOURCE_DIR}/Doxyfile.in ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile @ONLY)
# 	add_custom_target(	doc_bear
# 						${DOXYGEN_EXECUTABLE} ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile
# 						WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
# 						COMMENT "Generating API documentation with Doxygen" VERBATIM )
# endif(DOXYGEN_FOUND)


#########################################
######			Examples	             ######
#########################################
if(${BUILD_EXAMPLES})
	add_subdirectory(examples)
endif()

#########################################
######			Tools	                 ######
#########################################
# if(${BUILD_TOOLS})
# 	add_subdirectory(tools)
# endif()


#########################################
######			Test				######
#########################################

# find_package(GTest)
# if(${GTest_FOUND})
#   enable_testing()
#   add_subdirectory(tests)
# endif()

#########################################
######			INSTALL 		   ######
#########################################
include(CMakePackageConfigHelpers)

if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/usr/local")
endif() 

install(TARGETS bear
        EXPORT bearTargets
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
        INCLUDES DESTINATION ${CMAKE_INSTALL_PREFIX}
        )

write_basic_package_version_file(
		bearConfigVersion.cmake
    VERSION ${PACKAGE_VERSION}
    COMPATIBILITY AnyNewerVersion
    )

# This installs the include folder
install(DIRECTORY include DESTINATION ${CMAKE_INSTALL_PREFIX} FILES_MATCHING PATTERN "*.h")
install(DIRECTORY include DESTINATION ${CMAKE_INSTALL_PREFIX} FILES_MATCHING PATTERN "*.inl")

export(TARGETS bear NAMESPACE bear:: FILE bearTargets.cmake)

# This generates bearTargets.cmake
install(EXPORT bearTargets
    FILE bearTargets.cmake
    NAMESPACE bear::
    DESTINATION lib/cmake/${PROJECT_NAME}
    )

configure_file("${CMAKE_SOURCE_DIR}/cmake/templates/bearConfig.cmake.in"
               "${CMAKE_BINARY_DIR}/${PROJECT_NAME}Config.cmake"
                @ONLY)
install(FILES "${CMAKE_CURRENT_BINARY_DIR}/bearConfig.cmake"
              "${CMAKE_CURRENT_BINARY_DIR}/bearConfigVersion.cmake" 
        DESTINATION lib/cmake/${PROJECT_NAME}
        )

# # uninstall target 
# if(NOT TARGET uninstall) 
#   configure_file( 
#       "${CMAKE_CURRENT_SOURCE_DIR}/cmake/templates/cmake_uninstall.cmake.in" 
#       "${CMAKE_CURRENT_BINARY_DIR}/cmake/templates/cmake_uninstall.cmake" 
#       IMMEDIATE @ONLY) 
 
#   add_custom_target(uninstall COMMAND ${CMAKE_COMMAND} -P ${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake) 
# endif() 
