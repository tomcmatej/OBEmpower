
cmake_minimum_required(VERSION "3.10")
project(PluginOBEmpower)

file(GLOB project_SRC
	"src/*.hpp"
    "src/*.cpp"
)

add_library("${PROJECT_NAME}" SHARED ${project_SRC})

target_include_directories("${PROJECT_NAME}" PRIVATE
	${CMAKE_SOURCE_DIR}/lib/Producers/DevicePlugin
)

target_include_directories("${PROJECT_NAME}" PUBLIC
	${PROJECT_SOURCE_DIR}/src
)

target_link_libraries("${PROJECT_NAME}"
	AdsClient
	XMLInterpreter
	xmlInput
	XercesC::XercesC
	Boost::boost
	PluginInterface
	PluginCommon
	InterThreadRO
	MapTools)
