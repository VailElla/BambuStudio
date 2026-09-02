cmake_minimum_required(VERSION 2.8.12)
project(miniz)

add_library(miniz INTERFACE)

add_library(miniz_static STATIC
    miniz.c
    miniz.h
)

if(APPLE)
    # Apple's zlib uses an architecture-optimized CRC32 implementation. Keep
    # miniz's archive validation semantics while avoiding its byte-at-a-time
    # checksum loop on large 3MF members.
    target_compile_definitions(miniz_static PRIVATE MINIZ_USE_SYSTEM_CRC32)
    target_link_libraries(miniz_static PRIVATE ZLIB::ZLIB)
endif()

if(${CMAKE_C_COMPILER_ID} STREQUAL "GNU")
    target_compile_definitions(miniz_static PRIVATE _GNU_SOURCE)
endif()

target_link_libraries(miniz INTERFACE miniz_static)
target_include_directories(miniz INTERFACE ${CMAKE_CURRENT_SOURCE_DIR})
