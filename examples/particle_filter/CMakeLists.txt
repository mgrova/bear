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


add_executable(example_pf_2d example_2d.cpp)
target_link_libraries(example_pf_2d LINK_PUBLIC bear)


find_package(PCL QUIET REQUIRED)
if(${PCL_FOUND})
	add_executable(example_pf_3d example_3d.cpp)
	target_link_libraries(example_pf_3d LINK_PUBLIC bear)
	target_include_directories(example_pf_3d PUBLIC ${PCL_INCLUDE_DIRS})
	target_link_libraries(example_pf_3d LINK_PUBLIC ${PCL_LIBRARIES})
	
	target_compile_definitions(example_pf_3d PUBLIC "ENABLE_PCL" "DISABLE_PCAP" "DISABLE_PNG" "DISABLE_LIBUSB_1_0")
endif()