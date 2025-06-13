###############################################################################
#  HardFOC HAL – **single-component** build script
#  Copy this file to hf-hal/CMakeLists.txt
###############################################################################

cmake_minimum_required(VERSION 3.16)

set(HF_HAL_ROOT ${CMAKE_CURRENT_LIST_DIR})

#
# ─── 1 · Locate all implementation files (skip docs, examples, tests) ──────────
#
set(_SRC_GLOBS
    "${HF_HAL_ROOT}/*.c"  "${HF_HAL_ROOT}/*.cpp"  "${HF_HAL_ROOT}/*.cc"
    "${HF_HAL_ROOT}/*.S"  "${HF_HAL_ROOT}/**/*.c" "${HF_HAL_ROOT}/**/*.cpp"
    "${HF_HAL_ROOT}/**/*.cc" "${HF_HAL_ROOT}/**/*.S"
)
file(GLOB_RECURSE _ALL_SRCS CONFIGURE_DEPENDS ${_SRC_GLOBS})

set(_EXCLUDE_REGEX "/docs/|/Datasheet/|/datasheet/|/examples/|/tests/")

foreach(_f IN LISTS _ALL_SRCS)
    if(NOT _f MATCHES "${_EXCLUDE_REGEX}")
        list(APPEND HF_HAL_SRCS "${_f}")
    endif()
endforeach()

#
# ─── 2 · Collect every directory that owns a header we kept ───────────────────
#
file(GLOB_RECURSE _ALL_HDRS
     CONFIGURE_DEPENDS
     "${HF_HAL_ROOT}/*.h"    "${HF_HAL_ROOT}/*.hpp"
     "${HF_HAL_ROOT}/**/*.h" "${HF_HAL_ROOT}/**/*.hpp"
)

foreach(_h IN LISTS _ALL_HDRS)
    if(NOT _h MATCHES "${_EXCLUDE_REGEX}")
        get_filename_component(_dir "${_h}" DIRECTORY)
        list(APPEND HF_HAL_INC_DIRS "${_dir}")
    endif()
endforeach()
list(REMOVE_DUPLICATES HF_HAL_INC_DIRS)

#
# ─── 3 · Register as ONE ESP-IDF component ────────────────────────────────────
#
idf_component_register(
    SRCS          ${HF_HAL_SRCS}
    INCLUDE_DIRS  ${HF_HAL_INC_DIRS}
    #
    # PUBLIC deps: anything the *application* must also link against.
    # (change to taste – these are the most common for a HAL).
    #
    REQUIRES      driver freertos
    #
    # PRIVATE deps: used only while building this lib.
    #
    PRIV_REQUIRES hal esp_timer
)

#
# ─── 4 · Optional: tighten warnings for hf-hal only ───────────────────────────
#
target_compile_options(${COMPONENT_LIB} PRIVATE -Wall -Wextra -Werror)
