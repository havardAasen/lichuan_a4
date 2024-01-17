# FIXME: Don't hard-code dependency location
link_directories(/srv/git/linuxcnc/lib)
add_executable(lichuan_a4 hal.cpp main.cpp modbus.cpp lichuan_a4.cpp)
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

install(TARGETS lichuan_a4
        RUNTIME DESTINATION bin
)
