# dls-ctl client utility
# Can be built standalone or as part of main project

# Support standalone build
if(NOT CMAKE_PROJECT_NAME)
    cmake_minimum_required(VERSION 3.14)
    project(dls-ctl CXX)
    set(CMAKE_CXX_STANDARD 14)
    set(CMAKE_CXX_STANDARD_REQUIRED ON)

    # Fetch nlohmann_json for standalone build
    include(FetchContent)
    FetchContent_Declare(
        json
        URL https://github.com/nlohmann/json/releases/download/v3.11.3/json.tar.xz
    )
    FetchContent_MakeAvailable(json)

    set(DLS_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../include)
else()
    set(DLS_INCLUDE_DIR ${CMAKE_SOURCE_DIR}/include)
endif()

add_executable(dls-ctl dls_ctl.cpp)

target_link_libraries(dls-ctl
    PRIVATE
    nlohmann_json::nlohmann_json
)

target_include_directories(dls-ctl PRIVATE ${DLS_INCLUDE_DIR})

install(TARGETS dls-ctl DESTINATION bin)
