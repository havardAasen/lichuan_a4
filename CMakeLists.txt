cmake_minimum_required(VERSION 3.16)
project(lichuan_a4)

include(GNUInstallDirs)

set(CMAKE_CXX_STANDARD 17)
add_compile_options(-Wall -Wextra -Weffc++ -Wsign-conversion)

find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBMODBUS libmodbus)

# FIXME: Don't hard-code dependency location
link_directories(/srv/git/linuxcnc/lib)
add_executable(lichuan_a4 main.cpp lichuan_a4.cpp)
target_include_directories(lichuan_a4
        PRIVATE
        /srv/git/linuxcnc/include
        ${LIBMODBUS_INCLUDE_DIRS}
)
target_compile_definitions(lichuan_a4 PRIVATE RTAPI)
target_link_libraries(lichuan_a4
        PRIVATE
        linuxcnchal
        ${LIBMODBUS_LIBRARIES}
)
add_subdirectory(docs)
