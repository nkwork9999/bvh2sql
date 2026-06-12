# This file is included by DuckDB's build system. It specifies which extension to load

# DuckDB v1.5.x still defaults to C++11 in CMake, but its bundled fmt needs
# C++17 with current MSVC. Force the global standard before third_party targets
# such as duckdb_fmt are created.
set(CMAKE_CXX_STANDARD 17 CACHE STRING "C++ standard to enforce" FORCE)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# VS 2026 no longer provides stdext::checked_array_iterator, but the fmt copy
# bundled with DuckDB v1.5.3 still references it when _SECURE_SCL is defined.
if(MSVC)
    set(BVH2SQL_MSVC_COMPAT_HEADER "${CMAKE_CURRENT_LIST_DIR}/src/include/bvh2sql/msvc_secure_scl_compat.hpp")
    string(REPLACE "\\" "/" BVH2SQL_MSVC_COMPAT_HEADER "${BVH2SQL_MSVC_COMPAT_HEADER}")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /U_SECURE_SCL /FI${BVH2SQL_MSVC_COMPAT_HEADER}")
endif()

# Extension from this repo
duckdb_extension_load(bvh2sql
    SOURCE_DIR ${CMAKE_CURRENT_LIST_DIR}
    LOAD_TESTS
)

# Any extra extensions that should be built
# e.g.: duckdb_extension_load(json)
