cmake_minimum_required(VERSION 3.0)

project(disperser)

find_package(ArrayFire)

include_directories(
	include/
	${ArrayFire_INCLUDE_DIRS}
)

add_definitions(
)

file(GLOB sources
	src/*.cpp
	src/*.hpp
	
)

add_executable(disperser ${sources})
target_link_libraries(disperser ${ArrayFire_LIBRARIES})