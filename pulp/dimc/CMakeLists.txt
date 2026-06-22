get_filename_component(MOD_ROOT "${CMAKE_CURRENT_SOURCE_DIR}" REALPATH)

find_package(Threads REQUIRED)

set(DIMC_SRCS
    "${MOD_ROOT}/src/dimc_hwpe.cpp"
    "${MOD_ROOT}/src/dimc_macro.cpp"
    "${MOD_ROOT}/src/dimc_hwpe_streamer.cpp"
    "${MOD_ROOT}/src/dimc_hwpe_fsm.cpp"
    )
set(DIMC_HDRS
    "${MOD_ROOT}/include/dimc_hwpe_archi.hpp"
    "${MOD_ROOT}/include/dimc.hpp"
    "${MOD_ROOT}/include/dimc_macro.hpp"
    "${MOD_ROOT}/include/dimc_fifo.hpp"
    )

foreach(f IN LISTS DIMC_SRCS DIMC_HDRS)
    if(NOT EXISTS "${f}")
        message(FATAL_ERROR "DIMC: missing file: ${f}")
    endif()
endforeach()

vp_model(
    NAME    pulp.dimc.dimc
    SOURCES ${DIMC_SRCS}
)

vp_model_link_libraries(
    NAME pulp.dimc.dimc
    LIBRARY pthread
)

vp_model_include_directories(
    NAME      pulp.dimc.dimc
    DIRECTORY "${MOD_ROOT}/include"
)
