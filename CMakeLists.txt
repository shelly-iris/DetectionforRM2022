cmake_minimum_required(VERSION 3.10)

project(TDTVision_RM2022)

if(BUILD_DEFENSIVE_CAMERA)
	project(defensive_camera)
endif()
set(CMAKE_BUILD_TYPE "Debug")
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_FLAGS_DEBUG "-g -ggdb")
SET(PROJECT_ROOT_PATH "${PROJECT_SOURCE_DIR}")
SET(EXECUTABLE_OUTPUT_PATH "${PROJECT_BINARY_DIR}/bin/")
SET(LIBRARY_OUTPUT_PATH "${PROJECT_BINARY_DIR}/lib/")
link_directories(${PROJECT_BINARY_DIR}/lib/)


add_definitions(-D ARMOR_DETECT_DEBUG)
# add_definitions(-D VIDEO_DEBUG_USART)
# add_definitions(-D SENTRY_CODE)

#注释下方宏定义以开启警告
#add_definitions(-w)
#TODO：不注释掉这个宏进行标定，此时无论是否定义RECORDER都不会录像
#开关O3优化（打开后无法进行单步调试，目前只涉及灯条检测解算以及开火决策部分）
add_definitions(-D O3ENABLE)
#add_definitions(-D LIGHT_BAR_MATCH_DEBUG)
#add_definitions(-D OUTPUT_LIGHT_BAR_INFORMATION)
#add_definitions(-D OUTPUT_SINGLE_LIGHT_BAR_DETECT_INFORMATION)
add_definitions(-D AUTO_DECIDE)#注释后代表半自动选择

option(BUILD_DEFENSIVE_CAMERA         		"Build defensive_camera which is used to distinguish enemy beyond the robot"  OFF)

if(BUILD_DEFENSIVE_CAMERA)
	add_definitions(-D DEFENSIVE_CAMERA)
endif()

set(CMAKE_INCLUDE_CURRENT_DIR ON)

set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

set(CMAKE_CXX_STANDARD_REQUIRED ON)#能运行,zhaobudao

find_package(OpenCV 4 REQUIRED)

#openvino
include_directories(
		#OpenVINO推理引擎的头文件
		/opt/intel/openvino_2021/deployment_tools/inference_engine/include/
		/opt/intel/openvino_2021/deployment_tools/ngraph/include/
)
set(InferenceEngine_DIR "/opt/intel/openvino_2021/deployment_tools/inference_engine/share")

find_package(InferenceEngine)

set(ngraph_DIR "/opt/intel/openvino_2021/deployment_tools/ngraph/cmake")

find_package(ngraph REQUIRED)


set(ngraph_INCLUDE_DIRS "/opt/intel/openvino_2021/deployment_tools/ngraph/include/")





include_directories(
		${CMAKE_CURRENT_SOURCE_DIR}/include
		${OpenCV_INCLUDE_DIR}
		${InferenceEngine_INCLUDE_DIRS}
		${ngraph_INCLUDE_DIRS}
)
######################################################

include_directories(${OpenCV_INCLUDE_DIRS})

include_directories(${PROJECT_SOURCE_DIR}/tdt_common)
add_subdirectory(tdt_common)
include_directories(${PROJECT_SOURCE_DIR}/ksconnect)
add_subdirectory(ksconnect)
aux_source_directory(./ksconnect KS_SRCS)

include_directories(${PROJECT_SOURCE_DIR}/light)
add_subdirectory(light)
aux_source_directory(./light LIGHT_SRCS)

include_directories(${PROJECT_SOURCE_DIR}/usart)
add_subdirectory(usart)
aux_source_directory(./usart USART_SRCS)

include_directories(${PROJECT_SOURCE_DIR}/tool)
add_subdirectory(tool)
aux_source_directory(./tool TOOL_SRCS)


include_directories(${PROJECT_SOURCE_DIR}/tdt_camera)
file(GLOB HIKVISION_LIBS ${PROJECT_SOURCE_DIR}/tdt_camera/hikvisioncam/libhikvision/lib64/*)
file(COPY ${HIKVISION_LIBS} DESTINATION lib)
# 玄学问题需要把海康库拷贝到可执行文件目录或者系统库目录才能正常链接
# 待大佬发现缘由
# 建议将海康库拷贝到/usr/lib目录下
file(COPY ${HIKVISION_LIBS} DESTINATION bin)

file(COPY ${PROJECT_SOURCE_DIR}/libpython3.6.so DESTINATION lib)
file(COPY ${PROJECT_SOURCE_DIR}/script.py DESTINATION bin)

add_subdirectory(tdt_camera)



include_directories(${PROJECT_SOURCE_DIR}/tdt_communication)
add_subdirectory(tdt_communication)
aux_source_directory(./tdt_communication COMMUNICATION_SRCS)



add_subdirectory(./tdt_config)
include_directories(./tdt_config)
aux_source_directory(./tdt_config CONFIG_SRCS)
add_executable(
		TDTVision_RM2022

		main.cpp
		detector.cpp
		${BUFF_DETECTOR_SRCS}
		${IMM_DETECTOR_SRCS}
		${KS_SRCS}
		${LIGHT_SRCS}
		${TOOlKIT_SRCS}
		${TOOL_SRCS}
		${USART_SRCS}
		${DECISION_DETECTOR_SRCS}
		${DEBUG_SRCS}
		${LOG_SRCS}
		${COMMUNICATION_SRCS}
		${LOB_SRCS}
		${CONFIG_SRCS}
		${OPERATOR_SRCS}
)
target_link_libraries(
		TDTVision_RM2022
		PUBLIC
		${OpenCV_LIBS}
		${TDT_LOG_LIBS}
        ${InferenceEngine_LIBRARIES}
        ${OpenCV_LIBS}
        ${NGRAPH_LIBRARIES}

		tdtcommon
		tdtcamera
		pthread
		-lpthread -lm

)
