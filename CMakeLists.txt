cmake_minimum_required (VERSION 3.1) 
project (fahrdrahthoehe)

set(CMAKE_CXX_STANDARD 17)

add_subdirectory(parser)
generate_zusi_parser(zusi_parser ${CMAKE_CURRENT_BINARY_DIR}/zusi_parser)

add_executable(fahrdrahthoehe fahrdrahthoehe.cpp nowide/src/iostream.cpp)
target_link_libraries(fahrdrahthoehe PRIVATE zusi_parser)

find_package(Boost COMPONENTS filesystem REQUIRED)
target_include_directories(fahrdrahthoehe PRIVATE ${Boost_INCLUDE_DIRS} ${PROJECT_SOURCE_DIR} nowide/include)
target_link_libraries(fahrdrahthoehe PRIVATE ${Boost_LIBRARIES})
target_compile_definitions(fahrdrahthoehe PRIVATE GLM_ENABLE_EXPERIMENTAL)
install(TARGETS fahrdrahthoehe DESTINATION bin)
