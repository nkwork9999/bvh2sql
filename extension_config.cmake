# This file is included by DuckDB's build system. It specifies which extension to load

# DuckDB v1.5.x still defaults to C++11 in CMake, but its bundled fmt needs
# C++17 with current MSVC. Force the global standard before third_party targets
# such as duckdb_fmt are created.
set(CMAKE_CXX_STANDARD 17 CACHE STRING "C++ standard to enforce" FORCE)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Extension from this repo
duckdb_extension_load(bvh2sql
    SOURCE_DIR ${CMAKE_CURRENT_LIST_DIR}
    LOAD_TESTS
)

# Any extra extensions that should be built
# e.g.: duckdb_extension_load(json)
