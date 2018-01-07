cmake_minimum_required (VERSION 3.1) 
project (fahrdrahthoehe)

set(CMAKE_CXX_STANDARD 17)

add_subdirectory(parser)

add_executable(fahrdrahthoehe fahrdrahthoehe.cpp)
target_link_libraries(fahrdrahthoehe PRIVATE zusi_parser)

find_package(Boost COMPONENTS filesystem graph program_options REQUIRED)
target_include_directories(fahrdrahthoehe PRIVATE ${Boost_INCLUDE_DIRS})
target_link_libraries(fahrdrahthoehe PRIVATE ${Boost_LIBRARIES})
