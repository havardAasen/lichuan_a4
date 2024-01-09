cmake_minimum_required(VERSION 3.16)
project(lichuan_a4 CXX)

include(GNUInstallDirs)

set(CMAKE_CXX_STANDARD 17)
add_compile_options(-Wall -Wextra -Weffc++ -Wsign-conversion)

find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBMODBUS libmodbus REQUIRED)

add_subdirectory(docs)
add_subdirectory(src)
