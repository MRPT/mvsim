# -----------------------------------------------------------------------------
# mvsim_set_target_build_options(target)
#
# Set defaults for each MVSIM cmake target
# -----------------------------------------------------------------------------
function(mvsim_set_target_build_options TARGETNAME)
  # Build for C++17
  # Not needed, automatically imported from mrpt targets.

  # Warning level:
  if (MSVC)
    # msvc:
    target_compile_options(${TARGETNAME} PRIVATE /W3)
    target_compile_definitions(${TARGETNAME} PRIVATE
      _CRT_SECURE_NO_DEPRECATE
      _CRT_NONSTDC_NO_DEPRECATE
      _SILENCE_ALL_CXX17_DEPRECATION_WARNINGS
    )
  else()
    # gcc & clang:
    target_compile_options(${TARGETNAME} PRIVATE
      -Wall -Wextra -Wshadow
      -Werror=return-type # error on missing return();
      -Wtype-limits -Wcast-align -Wparentheses
      -fPIC
    )
  endif()

  # Optimization:
  # -------------------------
  if((NOT MSVC) AND (NOT CMAKE_CROSSCOMPILING))
    if (NOT CMAKE_BUILD_TYPE STREQUAL "Debug")
      target_compile_options(${TARGETNAME} PRIVATE -O3)
    endif()
  endif()
endfunction()


# -----------------------------------------------------------------------------
# mvsim_add_test(
#	TARGET name
#	SOURCES ${SRC_FILES}
#	[LINK_LIBRARIES lib1 lib2]
#	)
#
# Defines a MVSIM unit test
# -----------------------------------------------------------------------------
function(mvsim_add_test)
    set(oneValueArgs TARGET)
    set(multiValueArgs SOURCES LINK_LIBRARIES)
    cmake_parse_arguments(mvsim_add_test "" "${oneValueArgs}" "${multiValueArgs}" ${ARGN})

    add_executable(${mvsim_add_test_TARGET}
      ${mvsim_add_test_SOURCES}
    )

    # Define common flags:
    mvsim_set_target_build_options(${mvsim_add_test_TARGET})

    # lib Dependencies:
    if (mvsim_add_test_LINK_LIBRARIES)
      target_link_libraries(
      ${mvsim_add_test_TARGET}
      ${mvsim_add_test_LINK_LIBRARIES}
      )
    endif()

    # Macro for source dir path:
    target_compile_definitions(${mvsim_add_test_TARGET} PRIVATE
        MVSIM_TEST_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}\"
        )

    # Run it:
    add_custom_target(run_${mvsim_add_test_TARGET} COMMAND $<TARGET_FILE:${mvsim_add_test_TARGET}>)
    add_test(${mvsim_add_test_TARGET}_build "${CMAKE_COMMAND}" --build ${CMAKE_CURRENT_BINARY_DIR} --target ${mvsim_add_test_TARGET})
    add_test(run_${mvsim_add_test_TARGET} ${EXECUTABLE_OUTPUT_PATH}/${mvsim_add_test_TARGET})
    set_tests_properties(run_${mvsim_add_test_TARGET} PROPERTIES DEPENDS ${mvsim_add_test_TARGET}_build)

    add_dependencies(run_${mvsim_add_test_TARGET} ${mvsim_add_test_TARGET})

endfunction()

