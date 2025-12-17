cmake_minimum_required(VERSION 3.14)
project(dls-daemon VERSION 1.0.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Export compile commands for IDE integration
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

# Compiler flags
set(CMAKE_CXX_FLAGS_DEBUG "-g -O0 -DDEBUG")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -DNDEBUG")

# Common warning flags
add_compile_options(-Wall -Wextra -Wpedantic)

# Find required packages
find_package(Threads REQUIRED)

# nlohmann/json - fetch if not found
include(FetchContent)
FetchContent_Declare(
    json
    GIT_REPOSITORY https://github.com/nlohmann/json.git
    GIT_TAG v3.11.2
)
FetchContent_MakeAvailable(json)

# Include directories
include_directories(${CMAKE_SOURCE_DIR}/include)

# Daemon sources
set(DAEMON_SOURCES
    src/main.cpp
    src/serial_reader.cpp
    src/dlt_parser.cpp
    src/packet_dispatcher.cpp
    src/data_server.cpp
    src/control_server.cpp
    src/binary_logger.cpp
    src/stats.cpp
    src/config.cpp
)

# Daemon executable
add_executable(dls-daemon ${DAEMON_SOURCES})
target_link_libraries(dls-daemon
    PRIVATE
    Threads::Threads
    nlohmann_json::nlohmann_json
)

# Enable real-time scheduling capabilities and POSIX extensions
target_compile_definitions(dls-daemon PRIVATE _GNU_SOURCE)

# Client utility
add_subdirectory(client)

# Install targets
install(TARGETS dls-daemon DESTINATION bin)
install(DIRECTORY config/ DESTINATION etc/dls-daemon
        FILES_MATCHING PATTERN "*.json")
install(PROGRAMS scripts/test_client.py DESTINATION bin
        RENAME dls-test-client)

# CPack for packaging
set(CPACK_PACKAGE_NAME "dls-daemon")
set(CPACK_PACKAGE_VERSION ${PROJECT_VERSION})
set(CPACK_PACKAGE_DESCRIPTION_SUMMARY "High-performance DLS/DLT packet capture daemon")
set(CPACK_PACKAGE_VENDOR "DLS Tools")
include(CPack)

# =============================================================================
# Manifest for VMBOX Integration
# =============================================================================
# Generate manifest.json from template
if(EXISTS "${CMAKE_SOURCE_DIR}/manifest.template.json")
    configure_file(manifest.template.json
                   ${CMAKE_CURRENT_BINARY_DIR}/manifest.json
                   @ONLY)
    install(FILES ${CMAKE_CURRENT_BINARY_DIR}/manifest.json
            DESTINATION .)
endif()


# Print configuration summary
message(STATUS "")
message(STATUS "DLS-DAEMON Configuration:")
message(STATUS "  Version:      ${PROJECT_VERSION}")
message(STATUS "  Build type:   ${CMAKE_BUILD_TYPE}")
message(STATUS "  C++ Standard: ${CMAKE_CXX_STANDARD}")
message(STATUS "")
