
cmake_minimum_required(VERSION "3.10")
project(PluginOBEmpower)

# file(GLOB project_SRC
#     "src/OBEmpowerPlugin.cpp"
# )

add_library(${PROJECT_NAME} SHARED 
	src/OBEmpowerPlugin.cpp)

target_include_directories(${PROJECT_NAME} PRIVATE
	${CMAKE_SOURCE_DIR}/lib/Producers/DevicePlugin
)

target_include_directories(${PROJECT_NAME} PUBLIC
	${PROJECT_SOURCE_DIR}/src
)

target_link_libraries(${PROJECT_NAME}
	AdsClient
	XercesC::XercesC
	Boost::boost
	PluginInterface
	PluginCommon
	InterThreadRO
	MapTools
	FileLogger
	Filter
	XMLExecutionInterpreter)


# add_library(ConstantRateFilter 
# 	src/ConstantRateFilter.cpp
# )

# target_include_directories(ConstantRateFilter PUBLIC
# 	${PROJECT_SOURCE_DIR}/src
# )
# target_link_libraries(ConstantRateFilter
# 	Filter
# )