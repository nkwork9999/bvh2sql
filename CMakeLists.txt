cmake_minimum_required(VERSION 3.5...3.29)

set(TARGET_NAME bvh2sql)  # ← miniplot から変更
set(EXTENSION_NAME ${TARGET_NAME}_extension)
set(LOADABLE_EXTENSION_NAME ${TARGET_NAME}_loadable_extension)

project(${TARGET_NAME})

set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# OpenSSLを検索（コミュニティ提出のため残す）
find_package(OpenSSL REQUIRED)

set(EXTENSION_SOURCES src/bvh2sql_extension.cpp)  # ← ファイル名を変更

include_directories(src/include ${OPENSSL_INCLUDE_DIR})

build_static_extension(${TARGET_NAME} ${EXTENSION_SOURCES})
build_loadable_extension(${TARGET_NAME} " " ${EXTENSION_SOURCES})

# OpenSSLをリンク（コミュニティ提出のため残す）
target_link_libraries(${EXTENSION_NAME} OpenSSL::SSL OpenSSL::Crypto)
target_link_libraries(${LOADABLE_EXTENSION_NAME} OpenSSL::SSL OpenSSL::Crypto)

install(
  TARGETS ${EXTENSION_NAME}
  EXPORT "${DUCKDB_EXPORT_SET}"
  LIBRARY DESTINATION "${INSTALL_LIB_DIR}"
  ARCHIVE DESTINATION "${INSTALL_LIB_DIR}")