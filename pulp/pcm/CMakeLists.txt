# CMakeLists.txt sitting in: .../pcm/
# Layout assumed:
#   pcm/
#     src/pcm.cpp
#     include/pcm.hpp
#     pcm.py

get_filename_component(MOD_ROOT "${CMAKE_CURRENT_SOURCE_DIR}" REALPATH)

set(PCM_SRCS "${MOD_ROOT}/src/pcm.cpp")
set(PCM_HDRS "${MOD_ROOT}/include/pcm.hpp")

# Sanity checks so we fail with a clear message if paths are wrong
foreach(f IN LISTS PCM_SRCS PCM_HDRS)
  if(NOT EXISTS "${f}")
    message(FATAL_ERROR "PCM: missing file: ${f}")
  endif()
endforeach()

# Build the GVSoC model (target name == vp class)
vp_model(
  NAME    pulp.pcm.pcm
  SOURCES ${PCM_SRCS}
)

# Include directory for this model
vp_model_include_directories(
  NAME      pulp.pcm.pcm
  DIRECTORY "${MOD_ROOT}/include"
)

# Compile definition (applies to this target)
# target_compile_definitions(pulp.pcm.pcm PRIVATE PCM_MEM_PATH="${MOD_ROOT}")
